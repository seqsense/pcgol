package gnuplot

import (
	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
)

func ExampleGnuplot() {
	g := Must(New())

	pp1 := pc.Vec3Slice{
		mat.Vec3{-2, 0, 0},
		mat.Vec3{-1, 1, 0},
		mat.Vec3{0, 2, 0},
		mat.Vec3{1, 1, 1},
		mat.Vec3{2, 0, 0},
	}
	pp2 := pc.Vec3Slice{
		mat.Vec3{-2, 0, 1},
		mat.Vec3{-1, 1, 1},
		mat.Vec3{0, 2, 1},
		mat.Vec3{1, 1, 2},
		mat.Vec3{2, 0, 1},
	}
	g.Write("set xrange [-5:5]")
	g.Write("set yrange [-5:5]")
	g.Write("set zrange [-5:5]")
	g.Splot(
		&PointsPlot{Points: pp1},
		&PointsPlot{Points: pp2},
		&PointPairsPlot{Points: [2]pc.Vec3RandomAccessor{pp1, pp2}},
	)

	select {}
}
