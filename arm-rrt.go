package main

/*
How to pipe output directly to gnuplot (assumes AnimationLength = 0 in the setup):
    go run arm-rrt.go setup0.json | gnuplot -p

When setting AnimationLength > 1, you still pipe it into gnuplot but
the program creates a number of png files in the cwd, which can then
get converted into a movie.

How to create animation using ffmpeg (tweak as required to fit your setup):
    ffmpeg -f image2 -pattern_type glob -i 'blah*.png' blah.mp4
*/

import (
	"fmt"
	"math"
	"math/rand"
	"time"
	"encoding/json"
	"os"
	"io/ioutil"
	"log"
)


const (
	// threshold value for things considered "zero" (could be made configurable)
	epsilon = 1.0e-3
)


type Setup struct {
	Unit string		// angle unit in parsed JSON file
	Dqmax float64		// maximum angle increase during collision checking
	Qstart []float64	// starting configuration
	Qgoal []float64		// goal configuration (will be changed into a set later)
	Qmin, Qmax []float64	// upper and lower bound for configuration components
	Obstacles [][]float64	// obstacle (lines specified using 4 numbers [x0 y0 x1 y1])
	RandomSeed int64        // random seed, in case you need repeatable runs
	Pgoal float64		// bias for sampling from the goal set
	Maxnsteps int		// maximum number of RDT steps
	Plot [][]string		// list of things to plot, with optional custom style
	AnimationLength int	// creates animated output if value > 1
	AnimationFile string
}


type Ray struct {
	px, py, dx, dy float64
}


type Node struct {
	path [][]float64
	robot [][]Ray
	pred *Node
	succ []*Node
}


func DefaultSetup() Setup {
	var setup Setup
	setup.Unit = "rad"
	setup.Dqmax = math.Pi / 90
	setup.Qstart = []float64 { -0.49*math.Pi, -0.49*math.Pi, -0.49*math.Pi,  1.0,  -1.0}
	setup.Qgoal = []float64 {  0.0,           0.49*math.Pi,  0.49*math.Pi,  0.0,  0.0}
	setup.Qmin = []float64 {
		-math.Pi/2.0,
		-math.Pi/2.0,
		-math.Pi/2.0,
		-math.Pi/2.0,
		-math.Pi/2.0,
	}
	setup.Qmax = []float64 {
		math.Pi/2.0,
		math.Pi/2.0,
		math.Pi/2.0,
		math.Pi/2.0,
		math.Pi/2.0,
	}
	setup.Obstacles = [][]float64 {
		{  0.0, 0.6, 0.5, 0.6 },
		{  1.3, 1.3, 1.8, 1.3 },
		{  1.0, 2.2, 1.2, 2.4 },
		{  1.2, 0.7, 1.3, 0.6 },
	}
	setup.RandomSeed = time.Now().UnixNano()
	setup.Pgoal = 0.1
	setup.Maxnsteps = 10000
	setup.Plot = [][]string {
		{ "samples",   "w l t 'samples'"    },
		{ "nodes",     "w l t 'nodes'"      },
		{ "path",      "w l t 'path'"       },
		{ "query",     "w l lw 2 t 'query'" },
		{ "obstacles", "w l lw 2 t 'obst'"  },
	}
	setup.AnimationLength = 1
	setup.AnimationFile = "anim"
	return setup
}


func CreateSetup() Setup {
	setup := DefaultSetup()
	for _, arg := range(os.Args[1:]) {
		msg, err := ioutil.ReadFile(arg)
		if nil != err {
			log.Fatal("reading ", arg, " failed: ", err)
		}
		err = json.Unmarshal(msg, &setup)
		if nil != err {
			log.Fatal("parsing ", arg, " failed: ", err)
		}
		fmt.Println("# parsed ", arg)
	}
	if len(setup.Qstart) != len(setup.Qgoal) ||
		len(setup.Qstart) != len(setup.Qmin) ||
		len(setup.Qstart) != len(setup.Qmax) {
		log.Fatal("dimension mismatch")
	}
	if setup.Unit == "deg" {
		setup.Dqmax *= math.Pi / 180
		for ii := 0; ii < len(setup.Qstart); ii += 1 {
			setup.Qstart[ii] *= math.Pi / 180
		}
		for ii := 0; ii < len(setup.Qgoal); ii += 1 {
			setup.Qgoal[ii] *= math.Pi / 180
		}
		for ii := 0; ii < len(setup.Qmin); ii += 1 {
			setup.Qmin[ii] *= math.Pi / 180
		}
		for ii := 0; ii < len(setup.Qmax); ii += 1 {
			setup.Qmax[ii] *= math.Pi / 180
		}
	} else if setup.Unit != "rad" {
		log.Fatal("invalid unit \"", setup.Unit, "\"")
	}
	if ! IsFree(RobotModel(setup.Qgoal), setup.Obstacles) {
		log.Fatal("start configuration is in collision")
	}
	if ! IsFree(RobotModel(setup.Qgoal), setup.Obstacles) {
		log.Fatal("goal configuration is in collision")
	}
	for ii := 0; ii < len(setup.Qmin); ii += 1 {
		if setup.Qstart[ii] < setup.Qmin[ii] || setup.Qstart[ii] > setup.Qmax[ii] {
			log.Fatal("start violates joint limit")
		}
		if setup.Qgoal[ii] < setup.Qmin[ii] || setup.Qgoal[ii] > setup.Qmax[ii] {
			log.Fatal("goal violates joint limit")
		}
	}
	return setup
}


func LineRayIntersect(p0x, p0y, p1x, p1y, rx, ry, dx, dy float64) float64 {
	v0x := p1x - p0x
	v0y := p1y - p0y
	det := dx * v0y - v0x * dy
	if math.Abs(det) <= epsilon {
		return -1     // parallel, or zero length or direction
	}
	mu0 := (dx * (ry - p0y) - dy * (rx - p0x)) / det
	if (mu0 < 0) || (mu0 > 1) {
		return -1	// outside of segment
	}
	
	// calculate distance from ray origin to intersection
	mu1 := (v0x * (ry - p0y) - v0y * (rx - p0x)) / det
	if mu1 < 0 {
		return -1	// reverse intersection
	}
	
	return mu1
}


func RobotModel(config []float64) []Ray {
	rob := make([]Ray, 0)
	dd := 1.0
	x0 := 0.0
	y0 := 0.0
	th := 0.0
	for _, qq := range(config) {
		th += qq
		ray := Ray{ x0, y0, dd * math.Cos(th), dd * math.Sin(th) }
		rob = append(rob, ray)
		x0 += ray.dx
		y0 += ray.dy
		dd *= 0.8
	}
	return rob
}


func QInterpolate(ss float64, aa, bb []float64) []float64 {
	tt := 1-ss
	var cc []float64
	if len(aa) <= len(bb) {
		cc = make([]float64, len(aa))
	} else {
		cc = make([]float64, len(bb))
	}
	for ii := 0; ii < len(cc); ii += 1 {
		cc[ii] = tt*aa[ii] + ss*bb[ii]
	}
	return cc
}


func QDistance(aa, bb []float64) float64 {
	dist := 0.0
	var sz int
	if len(aa) <= len(bb) {
		sz = len(aa)
	} else {
		sz = len(bb)
	}
	for ii := 0; ii < sz; ii += 1 {
		dd := math.Abs(aa[ii] - bb[ii])
		if dd > dist {
			dist = dd
		}
	}
	return dist
}


func DumpRobot(rob []Ray) {
	for _, ray := range(rob) {
		fmt.Printf("% 5f  % 5f\n", ray.px, ray.py)
	}
	end := rob[len(rob)-1]
	fmt.Printf("% 5f  % 5f\n", end.px + end.dx, end.py + end.dy)
	fmt.Println()
	fmt.Println()
}


func DumpRobots(rob [][]Ray, offset, step int) {
	for ii := offset; ii < len(rob); ii += step {
		DumpRobot(rob[ii])
	}
}


func DumpPath(root *Node) {
	for _, rob := range(root.robot) {
		DumpRobot(rob)
	}
	for _, succ := range(root.succ) {
		DumpPath(succ)
	}
}


func DumpNodes(root *Node) {
	DumpRobot(root.robot[len(root.robot)-1])
	for _, succ := range(root.succ) {
		DumpNodes(succ)
	}
}


func BacktracePath(leaf *Node) ([][]float64, [][]Ray) {
	path := make([][]float64, 0)
	robot := make([][]Ray, 0)
	for nil != leaf {
		path = append(leaf.path, path...)
		robot = append(leaf.robot, robot...)
		leaf = leaf.pred
		fmt.Println("## ", len(path), " ", len(robot))
	}
	return path, robot
}


func FindNearest(root *Node, qsamp []float64) (*Node, float64) {
	nearest := root
	dist := QDistance(qsamp, nearest.path[len(nearest.path)-1])
	for _, succ := range(root.succ) {
		nn, dd := FindNearest(succ, qsamp)
		if dd < dist {
			nearest = nn
			dist = dd
		}
	}
	return nearest, dist
}


func IsFree(robot []Ray, obstacles [][]float64) bool {
	for _, ee := range(obstacles) {
		for _, rr := range(robot) {
			dd := LineRayIntersect(ee[0], ee[1], ee[2], ee[3],
				rr.px, rr.py, rr.dx, rr.dy)
			if 0 <= dd && 1 >= dd {
				return false
			}
		}
	}
	return true
}


func Grow(qstart, qend []float64, dqmax float64, obstacles [][]float64) ([][]float64, [][]Ray) {
	dist := QDistance(qstart, qend)
	var nsteps int
	if dist < dqmax {
		nsteps = 1
	} else {
		nsteps = int(math.Ceil(dist / dqmax))
	}
	ds := 1.0 / float64(nsteps)
	
	path := make([][]float64, 0)
	robot := make([][]Ray, 0)
	for ii := 1; ii <= nsteps; ii += 1 {
		qtry := QInterpolate(float64(ii) * ds, qstart, qend)
		rob := RobotModel(qtry)
		if IsFree(rob, obstacles) {
			path = append(path, qtry)
			robot = append(robot, rob)
		} else {
			break
		}
	}
	
	return path, robot
}


func QSample(qmin, qmax []float64, pgoal float64, qgoal []float64) []float64 {
	if rand.Float64() < pgoal {
		return qgoal
	}
	qq := make([]float64, len(qmin))
	for ii := 0; ii < len(qq); ii += 1 {
		qq[ii] = qmin[ii] + rand.Float64() * (qmax[ii] - qmin[ii])
	}
	return qq
}


func main() {
	
	//////////////////////////////////////////////////
	// initializations
	
	setup := CreateSetup()
	rand.Seed(setup.RandomSeed)
	
	var root Node
	root.path = append(make([][]float64, 0), setup.Qstart)
	root.robot = append(make([][]Ray, 0), RobotModel(root.path[0]))
	root.succ = make([]*Node, 0)
	
	//////////////////////////////////////////////////
	// RRT algorithm
	
	samples := make([][]float64, 0)
	var goal *Node
	for ii := 0; ii < setup.Maxnsteps; ii += 1 {
		qsamp := QSample(setup.Qmin, setup.Qmax, setup.Pgoal, setup.Qgoal)
		samples = append(samples, qsamp)
		nearest, _ := FindNearest(&root, qsamp)
		path, rob := Grow(nearest.path[len(nearest.path)-1], qsamp,
			setup.Dqmax, setup.Obstacles)
		if len(path) > 0 {
			leaf := Node { path, rob, nearest, make([]*Node, 0) }
			nearest.succ = append(nearest.succ, &leaf)
			if QDistance(setup.Qgoal, path[len(path)-1]) < epsilon {
				goal = &leaf
				break;
			}
		}
	}
	
	//////////////////////////////////////////////////
	// output to gnuplot
	
	var robot [][]Ray;
	if nil != goal {
		_, robot = BacktracePath(goal)
	}
	var x0, y0, x1, y1 float64
	for ii, ee := range(setup.Obstacles) {
		if 0 == ii || ee[0] < x0 { x0 = ee[0] }
		if 0 == ii || ee[1] < y0 { y0 = ee[1] }
		if 0 == ii || ee[0] > x1 { x1 = ee[0] }
		if 0 == ii || ee[1] > y1 { y1 = ee[1] }
		if 0 == ii || ee[2] < x0 { x0 = ee[2] }
		if 0 == ii || ee[3] < y0 { y0 = ee[3] }
		if 0 == ii || ee[2] > x1 { x1 = ee[2] }
		if 0 == ii || ee[3] > y1 { y1 = ee[3] }
	}
	for _, rob := range(robot) {
		for _, ray := range(rob) {
			if ray.px < x0 { x0 = ray.px }
			if ray.py < y0 { y0 = ray.py }
			if ray.px > x1 { x1 = ray.px }
			if ray.py > y1 { y1 = ray.py }
		}
		ray := rob[len(rob)-1]
		if ray.px + ray.dx < x0 { x0 = ray.px + ray.dx }
		if ray.py + ray.dy < y0 { y0 = ray.py + ray.dy }
		if ray.px + ray.dx > x1 { x1 = ray.px + ray.dx }
		if ray.py + ray.dy > y1 { y1 = ray.py + ray.dy }
	}
	
	if setup.AnimationLength == 0 {
		setup.AnimationLength = 1
	} else if setup.AnimationLength < 0 {
		setup.AnimationLength = len(robot) / -setup.AnimationLength
	}
	
	for aa := 0; aa < setup.AnimationLength; aa += 1 {
		fmt.Println("set view equal xy")
		fmt.Printf("set xrange [%f:%f]\n", x0, x1)
		fmt.Printf("set yrange [%f:%f]\n", y0, y1)
		if setup.AnimationLength > 1 {
			fmt.Println("set term png")
			fmt.Printf("set output '%s%06d.png'\n", setup.AnimationFile, aa)
		}
		for ii := 0; ii < len(setup.Plot); ii += 1 {
			if 0 == ii {
				fmt.Print("plot '-' u 1:2 ")
			} else {
				fmt.Print(", '-' u 1:2 ")
			}
			if len(setup.Plot[ii]) > 1 {
				fmt.Print(setup.Plot[ii][1])
			} else {
				fmt.Print("w l t '", setup.Plot[ii][0], "'")
			}
		}
		fmt.Println()
		for ii := 0; ii < len(setup.Plot); ii += 1 {
			if "samples" == setup.Plot[ii][0] {
				fmt.Println("# samples");
				for _, qq := range(samples) {
					DumpRobot(RobotModel(qq))
				}
				fmt.Println("e")
			} else if "nodes" == setup.Plot[ii][0] {
				fmt.Println("# nodes");
				DumpNodes(&root)
				fmt.Println("e")
			} else if "path" == setup.Plot[ii][0] {
				fmt.Println("# path");
				if nil == goal {
					fmt.Println("## no path found")
				} else {
					if setup.AnimationLength > 1 {
						DumpRobots(robot, aa, setup.AnimationLength)
					} else {
						DumpRobots(robot, 0, 1)
					}
				}
				fmt.Println("e")
			} else if "query" == setup.Plot[ii][0] {
				fmt.Println("# query (start and goal)");
				DumpRobot(RobotModel(setup.Qstart))
				DumpRobot(RobotModel(setup.Qgoal))
				fmt.Println("e")
			} else if "obstacles" == setup.Plot[ii][0] {
				fmt.Println("# obstacles");
				for _, ee := range(setup.Obstacles) {
					fmt.Printf("% 5f  % 5f\n% 5f  % 5f\n\n\n",
						ee[0], ee[1], ee[2], ee[3])
				}
				fmt.Println("e")
			} else {
				log.Fatal("invalid Plot \"", setup.Plot[ii][0], "\"")
			}
		}
		if nil == goal || setup.AnimationLength <= 1 {
			break
		}
	}
}
