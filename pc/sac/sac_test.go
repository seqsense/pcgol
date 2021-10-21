package sac

import (
	"reflect"
	"testing"
	"time"

	"github.com/seqsense/pcgol/internal/gnuplot"
	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/storage/voxelgrid"
)

func TestSAC(t *testing.T) {
	pp := pc.Vec3Slice{
		mat.Vec3{0.0, 0.0, 0.0},
		mat.Vec3{0.1, 0.0, 0.1},
		mat.Vec3{0.2, 0.0, 0.2},
		mat.Vec3{0.2, 0.1, 0.6}, // outlier
		mat.Vec3{0.0, 0.1, 0.0},
		mat.Vec3{0.1, 0.1, 0.1},
		mat.Vec3{0.2, 0.1, 0.2},
		mat.Vec3{0.0, 0.2, 0.0},
		mat.Vec3{0.1, 0.2, 0.1},
		mat.Vec3{0.2, 0.2, 0.2},
		mat.Vec3{0.3, -0.1, 0.0}, // outlier
		mat.Vec3{0.6, 0.7, 0.0},  // outlier
		mat.Vec3{0.6, 0.3, 0.0},  // outlier
	}
	vg := voxelgrid.New(0.1, [3]int{8, 8, 8}, mat.Vec3{})
	for i, p := range pp {
		vg.Add(p, i)
	}
	m := NewVoxelGridSurfaceModel(vg, pp)

	s := New(NewRandomSampler(len(pp)), m)
	if ok := s.Compute(30); !ok {
		t.Fatal("SAC.Compute should succeed")
	}

	indice := s.Coefficients().Inliers(0.1)
	expectedIndice := []int{0, 1, 2, 4, 5, 6, 7, 8, 9}
	if !reflect.DeepEqual(expectedIndice, indice) {
		t.Errorf("Expected inlier: %v, got: %v", expectedIndice, indice)

		if debugPlot {
			g.Write("set xrange [-1:2]")
			g.Write("set yrange [-1:2]")
			g.Write("set zrange [-1:2]")
			cf := s.Coefficients().(*voxelGridSurfaceModelCoefficients)
			g.Splot(
				&gnuplot.PointsPlot{Points: pp},
				&gnuplot.PointsPlot{
					Options: "w lp notitle",
					Points: pc.Vec3Slice{
						cf.origin.Add(cf.v1),
						cf.origin,
						cf.origin.Add(cf.v2),
					},
				},
			)
			time.Sleep(debugPlotWait)
			g.Close()
		}
	}
}
