package main

import (
	"fmt"
	"math"
)

const (
	AXLEDIST = 1
	PHMAX = math.Pi/4
	TRJ_DURATION = 0.5
	SIM_NSTEPS = 10
	MAXDIST = 20
)

type state struct {
	x,y,th float64
}

type control struct {
	s, ph float64
}

type heap_item struct {
	q state
	dist, key float64
}

func insert(heap []heap_item, item heap_item) []heap_item {
	if nil == heap || 1 >= len(heap) {
		heap = make([]heap_item, 2) // heap[0] is skipped
		heap[1] = item
		return heap
	}
	
	heap = append(heap, item)
	//bubble_up(heap, len(heap)-1)
	index := len(heap) - 1
	parent := index / 2
	for parent > 0 && heap[index].key < heap[parent].key {
		heap[index], heap[parent] = heap[parent], heap[index]
		index = parent
		parent = index / 2
	}
	
	return heap
}

// the top item is returned as heap[0] unless the function returns nil
// in which case the heap was empty to begin with
func extract(heap []heap_item) []heap_item {
	if nil == heap || 1 >= len(heap) {
		return nil
	}
	if 2 == len(heap) {
		heap := []heap_item{ heap[1] }
		return heap
	}
	
	heap[0] = heap[1]		// remember that heap[0] is skipped
	heap[1] = heap[len(heap)-1]
	heap = heap[:len(heap)-1]
	//bubble_down(heap, 1)
	index := 1
	target := 1
	for {
		left  := 2 * index
		right := left + 1
		if left < len(heap) && heap[left].key < heap[target].key {
			target = left
		}
		if right < len(heap) && heap[right].key < heap[target].key {
			target = right
		}
		if target == index {
			return heap
		}
		heap[index], heap[target] = heap[target], heap[index]
		index = target
	}
	
	return heap		// never reached though
}

func empty(heap []heap_item) bool {
	return nil == heap || 1 >= len(heap)
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

func main() {
	var heap []heap_item
	heap = insert(heap, heap_item{ state{0, 0, 0}, 0, 0 })
	u := []control{
		{ 1.0, -PHMAX/2},
		{ 1.0,  PHMAX/2},
		// { 1.0,  0.0},
		// {-0.5, -PHMAX},
		// {-0.5,  PHMAX},
	}
	goal := state{ 1.0, 1.0, 0.0 }
	goaldist := 0.5
	
outer:
	for !empty(heap) {
		heap = extract(heap)
		ee := heap[0]
		if ee.dist > MAXDIST {
			fmt.Println("# max dist reached")
			break
		}
		for _, ut := range(u) {
			trj := integrate(ee.q, ut, TRJ_DURATION)
			dist := ee.dist
////			fmt.Printf("# distance: % 5f  key: % 5f\n", dist, ee.key)
			for i, r := range(trj) {
				if i > 0 {
					dist += math.Sqrt(
						math.Pow(trj[i-1].x - r.x, 2) +
						math.Pow(trj[i-1].y - r.y, 2))
				}
////				fmt.Printf("% 5f  % 5f  % 5f\n", r.x, r.y, r.th)
			}
			qend := trj[len(trj)-1]
			heuristic := math.Sqrt(
				math.Pow(qend.x - goal.x, 2) + math.Pow(qend.y - goal.y, 2))
			if heuristic < goaldist {
				fmt.Println("# goal reached")
				break outer
			}
			heap = insert(heap, heap_item{qend, dist, dist + heuristic})
			fmt.Printf("% 5f  % 5f  % 5f  % 5f\n", qend.x, qend.y, qend.th, dist)
////			fmt.Println()
////			fmt.Println()
		}
	}
}
