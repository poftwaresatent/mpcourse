// Example code for the Halmstad PhD Course on Motion Planning (2013)
//
// Copyright (C) 2013 Roland Philippsen. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.


package main

import (
	"fmt"
	"math"
	"strconv"
)

const (
	epsilon = 1.0e-3
)

type Line struct {
	p0x, p0y, p1x, p1y float64
}

type Ray struct {
	px, py, dx, dy float64
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


func interp(ss float64, aa, bb []float64) []float64 {
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


func dump(robs [][]Ray) {
	for _, rob := range(robs) {
		for _, ray := range(rob) {
			fmt.Printf("% 5f  % 5f\n", ray.px, ray.py)
		}
		end := rob[len(rob)-1]
		fmt.Printf("% 5f  % 5f\n", end.px + end.dx, end.py + end.dy)
		fmt.Println()
		fmt.Println()
	}
}


func its(ii int) string {
	return strconv.FormatInt(int64(ii), 10)
}


func main() {
	environment := []Line {
		{  0.0, 0.6, 0.5, 0.6 },
		{  1.3, 1.3, 1.8, 1.3 },
		{  3.0, 0.0, 3.0, 0.4 },
		{  1.0, 2.2, 1.2, 2.4 },
		{  1.2, 0.7, 1.3, 0.6 },
	}
	
	free := make([][]Ray, 0)
	obst := make([][]Ray, 0)
	
	for ss := 0.0; ss <= 1.0; ss += 0.01 {
		
		rob := RobotModel(interp(ss,
			[]float64{  0.5, -0.5, -0.5,  0.5,  0.5 },
			[]float64{  1.0,  0.5,  1.0,  0.5,  1.0 },
////			[]float64{  0.0, -1.0},
////			[]float64{  1.0,  1.0},
//			[]float64{  0.5, -0.5},//, -0.5,  0.5,  0.5 },
//			[]float64{  1.0,  0.5},//,  1.0,  0.5,  1.0 }
		))
		collision := false
	detect:
		for _, ee := range(environment) {
			for _, rr := range(rob) {
				dd := LineRayIntersect(ee.p0x, ee.p0y, ee.p1x, ee.p1y,
					rr.px, rr.py, rr.dx, rr.dy)
				if 0 <= dd && 1 >= dd {
					collision = true
					break detect
				}
			}
		}
		if collision {
			obst = append(obst, rob)
		} else {	
			free = append(free, rob)
		}
	}
	
	fmt.Println("set view equal xy")
	cmd := make([]string, 0)
	if 0 < len(free) {
		cmd = append(cmd, "'-' u 1:2 w l t 'free'")
	}
	if 0 < len(obst) {
		cmd = append(cmd, "'-' u 1:2 w l t 'obst'")
        }
        if 0 < len(environment) {
                cmd = append(cmd, "'-' u 1:2 w l t 'env'")
        }
	for ii, cc := range(cmd) {
		if 0 == ii {
			fmt.Print("plot ", cc)
		} else {
			fmt.Print(", ", cc)
		}
	}
	fmt.Println()
	fmt.Println("# free configurations");
	dump(free)
	
	fmt.Println("e")
	fmt.Println("# obstacle configurations");
	dump(obst)
	
	fmt.Println("e")
 	fmt.Println("# environment");
 	for _, ee := range(environment) {
 		fmt.Printf("% 5f  % 5f\n% 5f  % 5f\n\n\n", ee.p0x, ee.p0y, ee.p1x, ee.p1y)
 	}
	
}
