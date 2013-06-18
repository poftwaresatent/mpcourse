package main

import (
	"fmt"
	"math"
)

const (
	AXLEDIST = 1
	PHMAX = math. Pi/4
	TRJ_DURATION = 0.1
	SIM_NSTEPS = 10
)

type state struct {
	x,y,th float64
}

type control struct {
	s, ph float64
}

func fwd(q state, u control) state {
	var qd state
	qd.x = u.s*math.Cos(q.th)
	qd.y = u.s*math.Sin(q.th)
	qd.th = u.s*math.Tan(u.ph)/AXLEDIST
	return qd
}

func integrate(q state, u control, t float64) []state {
	trj := make([]state, SIM_NSTEPS+1)
	trj[0] = q
	t /= SIM_NSTEPS
	for i := 1; i <= SIM_NSTEPS; i += 1 {
		qd := fwd(q,u)
		q.x += t*qd.x
		q.y += t*qd.y
		q.th += t*qd.th
		trj[i] = q
	}
	return trj
}

type edge struct {
	q state
	depth int
}

func main() {
	queue := make([]edge, 1)
	u := []control{
		{ 1.0, -PHMAX/2},
		{ 1.0,  PHMAX/2},
		{ 1.0,  0.0},
		// {-0.5, -PHMAX},
		// {-0.5,  PHMAX},
	}
	
	treedepth := 5
	
	fmt.Println("set view equal xy")
	fmt.Println("plot '-' u 1:2 w l t 'rollout'")
	for queue[0].depth <= treedepth {
		ee := queue[0]
		queue = queue[1:]
		for _, ut := range(u) {
			trj := integrate(ee.q, ut, TRJ_DURATION)
			queue = append(queue, edge{trj[len(trj)-1], ee.depth+1})
			for _,r := range(trj) {
				fmt.Printf("% 5f  % 5f  % 5f\n", r.x, r.y, r.th)
			}
			fmt.Println()
			fmt.Println()
		}
	}
}
