package icp

import (
	"fmt"
	"testing"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/storage/kdtree"
)

func TestPointToPointICPGradient(t *testing.T) {
	base := pc.Vec3Slice{
		mat.Vec3{0, 0, 0},
		mat.Vec3{1, 1, 0},
		mat.Vec3{2, 2, 0},
		mat.Vec3{3, 1, 1},
		mat.Vec3{4, 0, 0},
	}
	for _, delta := range []mat.Vec3{
		{0, 0, 0},
		{0.25, 0.125, -0.125},
		{0.5, 0.5, 1.0},
		{-0.5, -0.5, 1.0},
	} {
		t.Run(fmt.Sprintf("(%0.3f,%0.3f,%0.3f)", delta[0], delta[1], delta[2]),
			func(t *testing.T) {
				expectedTrans := mat.Translate(-delta[0], -delta[1], -delta[2])
				target := pc.Vec3Slice{
					base[2].Add(delta),
					base[3].Add(delta),
					base[4].Add(delta),
				}
				kdt := kdtree.New(base)
				ppicp := &PointToPointICPGradient{
					Evaluator: &PointToPointEvaluator{
						Corresponder: NewNearestPointCorresponder(kdt, 2),
						MinPairs:     3,
					},
				}

				trans, err := ppicp.Fit(target)
				if err != nil {
					t.Fatal(err)
				}
				residual := trans.Transform(delta).Norm()
				if !(0.025 >= residual) { // checking NaN
					t.Errorf("Expected transform:\n%v\nGot:\n%v\n(residual: %f)", expectedTrans, trans, residual)
				}
			},
		)
	}
}
