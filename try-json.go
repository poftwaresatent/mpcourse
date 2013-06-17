package main

import (
	"fmt"
	"encoding/json"
	"io/ioutil"
	"log"
)

type Setup struct {
	Qstart, Qgoal []float64
	Environment [][]float64
	Plotspec []string
}

func main() {
	blah := Setup {
		[]float64 { 1, 2, 3, 4 },
		[]float64 { 5, 6, 7, 8 },
		[][]float64 {
			{  0.0, 0.6, 0.5, 0.6 },
			{  1.3, 1.3, 1.8, 1.3 },
			{  1.0, 2.2, 1.2, 2.4 },
			{  1.2, 0.7, 1.3, 0.6 },
		},
		[]string { "blah", "blih", "blupp" },
	}
	
	msg, err := json.Marshal(blah)
	if nil != err {
		log.Fatal("marshal failed: ", err)
	}
	fmt.Printf("marshal ok\n%s\n", msg)
	
	var foo Setup
	msg, err = ioutil.ReadFile("foo.json")
	if nil != err {
		log.Fatal(err)
	}
	
	err = json.Unmarshal(msg, &foo)
	if nil != err {
		log.Fatal("unmarshaling foo.json: ", err)
	}
	fmt.Println("unmarshal ok\n", foo.Qstart, " ", foo.Qgoal, " ", foo.Environment)
}
