//go:build hasSed && !js
// +build hasSed,!js

package gnuplot

import (
	"time"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
)

func ExampleGnuplot() {
	// Delete multiple blank lines since Go testable example can't handle them.
	g := Must(NewWithCommand("sed", ":l;/^\\n*$/{s/\\n//;N;bl}"))
	// g := Must(New()) // for actual plot

	pp1 := pc.Vec3Slice{
		mat.Vec3{0, 2, 0},
		mat.Vec3{5, 1, 1},
		mat.Vec3{2, -3, 1},
	}
	pp2 := pc.Vec3Slice{
		mat.Vec3{0, 2, 1},
		mat.Vec3{5, 1, 2},
		mat.Vec3{2, -3, 1.5},
	}
	g.Write("set title \"test\"")
	g.Splot(
		&PointsPlot{Points: pp1},
		&PointsPlot{Points: pp2},
		&PointPairsPlot{Points: [2]pc.Vec3RandomAccessor{pp1, pp2}},
	)

	time.Sleep(50 * time.Millisecond)
	g.Close()

	// Output:
	// set grid
	// set size ratio -1
	// set view equal xyz
	// set ticslevel 0
	// set title "test"
	// splot"-" u 1:2:3 notitle nohidden3d,"-" u 1:2:3 notitle nohidden3d,"-" u 1:2:3 w l notitle
	// 0.000000 2.000000 0.000000
	// 5.000000 1.000000 1.000000
	// 2.000000 -3.000000 1.000000
	// e
	// 0.000000 2.000000 1.000000
	// 5.000000 1.000000 2.000000
	// 2.000000 -3.000000 1.500000
	// e
	// 0.000000 2.000000 0.000000
	// 0.000000 2.000000 1.000000
	//
	//
	// 5.000000 1.000000 1.000000
	// 5.000000 1.000000 2.000000
	//
	//
	// 2.000000 -3.000000 1.000000
	// 2.000000 -3.000000 1.500000
	//
	//
	// e
}
