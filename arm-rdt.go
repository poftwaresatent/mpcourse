package main

import (
	"fmt"
	"math"
	"math/rand"
	"time"
)

const (
	epsilon = 1.0e-3
	dqmax = math.Pi / 90
)


type Line struct {
	p0x, p0y, p1x, p1y float64
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


func Grow(qstart, qend []float64, obstacles []Line) ([][]float64, [][]Ray) {
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
outer:
	for ii := 1; ii <= nsteps; ii += 1 {
		qtry := QInterpolate(float64(ii) * ds, qstart, qend)
		rob := RobotModel(qtry)
		for _, ee := range(obstacles) {
			for _, rr := range(rob) {
				dd := LineRayIntersect(ee.p0x, ee.p0y, ee.p1x, ee.p1y,
					rr.px, rr.py, rr.dx, rr.dy)
				if 0 <= dd && 1 >= dd {
					break outer
				}
			}
		}
		path = append(path, qtry)
		robot = append(robot, rob)
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
	rand.Seed(time.Now().UnixNano())
	
	maxnsteps := 100
	
	qmin := []float64 {
		-math.Pi/2.0,
		-math.Pi/2.0,
		-math.Pi/2.0,
		-math.Pi/2.0,
		-math.Pi/2.0,
	}
	
	qmax := []float64 {
		math.Pi/2.0,
		math.Pi/2.0,
		math.Pi/2.0,
		math.Pi/2.0,
		math.Pi/2.0,
	}
	
	environment := []Line {
		{  0.0, 0.6, 0.5, 0.6 },
		{  1.3, 1.3, 1.8, 1.3 },
		{  1.0, 2.2, 1.2, 2.4 },
		{  1.2, 0.7, 1.3, 0.6 },
	}
	
	var root Node
//	root.path = append(make([][]float64, 0), []float64 {  0.5, -0.5, -0.5,  0.5,  0.5 })
	root.path = append(make([][]float64, 0), []float64 {  0.0, 0.49*math.Pi, 0.49*math.Pi,  0.0,  0.0 })
	root.robot = append(make([][]Ray, 0), RobotModel(root.path[0]))
	root.succ = make([]*Node, 0)
	
	pgoal := 0.0
	qgoal := []float64 {  1.0,  0.5,  1.0,  0.5,  1.0 }
	
	dbgsamples := make([][]float64, 0)
	
	for ii := 0; ii < maxnsteps; ii += 1 {
		qsamp := QSample(qmin, qmax, pgoal, qgoal)
		dbgsamples = append(dbgsamples, qsamp)
		nearest, _ := FindNearest(&root, qsamp)
		path, rob := Grow(nearest.path[len(nearest.path)-1], qsamp, environment)
		if len(path) > 0 {
			nearest.succ = append(nearest.succ,
				&Node { path, rob, nearest, make([]*Node, 0) })
		}
	}
	
	fmt.Println("set view equal xy")
	fmt.Println("plot '-' u 1:2 w l t 'paths', '-' u 1:2 w l t 'samples', '-' u 1:2 w l t 'nodes', '-' u 1:2 w l lw 2 t 'obst'")
	
	fmt.Println("# paths");
	DumpPath(&root)
	
	fmt.Println("e")
	fmt.Println("# samples");
	for _, qq := range(dbgsamples) {
		DumpRobot(RobotModel(qq))
	}
	
	fmt.Println("e")
	fmt.Println("# robot");
	DumpNodes(&root)
	
	fmt.Println("e")
 	fmt.Println("# environment");
 	for _, ee := range(environment) {
 		fmt.Printf("% 5f  % 5f\n% 5f  % 5f\n\n\n", ee.p0x, ee.p0y, ee.p1x, ee.p1y)
 	}
	
}
