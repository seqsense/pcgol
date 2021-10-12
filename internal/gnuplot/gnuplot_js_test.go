//go:build js
// +build js

package gnuplot

import (
	"testing"
	"time"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
)

func TestGnuplot(t *testing.T) {
	g := Must(New())

	pp1 := pc.Vec3Slice{
		mat.Vec3{0, 2, 0},
	}
	pp2 := pc.Vec3Slice{
		mat.Vec3{0, 2, 1},
	}
	g.Write("set title \"test\"")
	g.Splot(
		&PointsPlot{Points: pp1},
		&PointsPlot{Points: pp2},
		&PointPairsPlot{Points: [2]pc.Vec3RandomAccessor{pp1, pp2}},
	)

	time.Sleep(100 * time.Millisecond)
	g.Close()

	// Everything must be ignored and nothing happened on js
}
