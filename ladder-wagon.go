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
)

const (
	epsilon = 1.0e-3
	wagon_width = 0.5
	ladder_length = 2.0
)

type Line struct {
	p0x, p0y, p1x, p1y float64
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


func main() {
	environment := []Line {
		{ 0.0, 0.0, 0.0, 1.8 },
		{ 0.0, 1.8, 2.5, 1.8 },
		{ 2.5, 1.8, 2.5, 1.4 },
		{ 2.5, 1.4, 2.7, 1.4 },
		{ 2.7, 1.4, 2.7, 2.0 }}
	
	for th := math.Pi; th >= 0; th -= math.Pi / 40 {
		//fmt.Printf("%4.2f ", th)
		cth := math.Cos(th)
		sth := math.Sin(th)
		for xx := 0.0; xx <= 5.0; xx += 0.03 {
			free := true
			for _, ll := range(environment) {
				dd := LineRayIntersect(
					ll.p0x, ll.p0y, ll.p1x, ll.p1y,
					xx, 0, cth, sth)
				if dd >= 0 && dd <= ladder_length {
					free = false
				} else {
					dd = LineRayIntersect(
						ll.p0x, ll.p0y, ll.p1x, ll.p1y,
						xx - wagon_width / 2, 0, 1, 0)
					if dd >= 0 && dd <= wagon_width {
						free = false
					}
				}
			}
			if free {
				fmt.Printf(".")
			} else {
				fmt.Printf("*")
			}
		}
		fmt.Println()
	}
}
