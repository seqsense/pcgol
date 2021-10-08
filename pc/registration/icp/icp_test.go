package icp

import (
	"testing"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/storage/kdtree"
)

func TestPointToPointICPGradient(t *testing.T) {
	base := pc.Vec3Slice{
		mat.Vec3{-2, 0, 0},
		mat.Vec3{-1, 1, 0},
		mat.Vec3{0, 2, 0},
		mat.Vec3{1, 1, 1},
		mat.Vec3{2, 0, 0},
	}
	for name, delta := range map[string]mat.Mat4{
		"Trans(0,0,0)":                  mat.Translate(0, 0, 0),
		"Trans(0.25,0.125,-0.125)":      mat.Translate(0.25, 0.125, -0.125),
		"Trans(0.5,0.5,1)":              mat.Translate(0.5, 0.5, 1.0),
		"Trans(-0.5,-0.5,0)":            mat.Translate(-0.5, -0.5, 0.0),
		"Rot(1,0,0,0.2)":                mat.Rotate(1, 0, 0, 0.2),
		"Rot(1,0,0,-0.2)":               mat.Rotate(1, 0, 0, -0.2),
		"Rot(1,0,0,0.1)Trans(0.2,0,0)":  mat.Rotate(1, 0, 0, 0.1).Mul(mat.Translate(0.2, 0, 0)),
		"Rot(1,0,0,0.1)Trans(-0.2,0,0)": mat.Rotate(1, 0, 0, 0.1).Mul(mat.Translate(-0.2, 0, 0)),
	} {
		delta := delta
		t.Run(name, func(t *testing.T) {
			target := pc.Vec3Slice{
				delta.Transform(base[0]),
				delta.Transform(base[1]),
				delta.Transform(base[2]),
				delta.Transform(base[3]),
				delta.Transform(base[4]),
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
			residual := trans.Mul(delta).Transform(mat.Vec3{1, 0, 0}).
				Sub(mat.Vec3{1, 0, 0}).Norm()
			if !(0.05 >= residual) { // checking NaN
				t.Errorf("Expected transform:\n%v\nGot:\n%v\n(residual: %f)", delta.Inv(), trans, residual)
			}
		})
	}
}
