package icp

import (
	"fmt"
	"testing"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/storage/kdtree"
)

func TestPointToPointICPGradient(t *testing.T) {
	base0 := pc.Vec3Slice{
		mat.Vec3{-2, 0, 0},
		mat.Vec3{-1, 1, 0},
		mat.Vec3{0, 2, 0},
		mat.Vec3{1, 1, 1},
		mat.Vec3{2, 0, 0},
	}
	base1 := pc.Vec3Slice{
		mat.Vec3{-2, 0, 5},
		mat.Vec3{-1, 1, 5},
		mat.Vec3{0, 2, 5},
		mat.Vec3{1, 1, 6},
		mat.Vec3{2, 0, 5},
	}
	for name, base := range map[string]pc.Vec3RandomAccessor{
		"CloseOrigin": base0,
		"FarOrigin":   base1,
	} {
		base := base
		if debugPlot {
			vMin, vMax, err := pc.MinMaxVec3RandomAccessor(base)
			if err != nil {
				t.Fatal(err)
			}
			g.Write(fmt.Sprintf("set xrange [%f:%f]", vMin[0]-1, vMax[0]+1))
			g.Write(fmt.Sprintf("set yrange [%f:%f]", vMin[1]-1, vMax[1]+1))
			g.Write(fmt.Sprintf("set zrange [%f:%f]", vMin[2]-1, vMax[2]+1))
		}

		t.Run(name, func(t *testing.T) {
			for name, delta := range map[string]mat.Mat4{
				"Trans(0,0,0)":                  mat.Translate(0, 0, 0),
				"Trans(0.25,0.125,-0.125)":      mat.Translate(0.25, 0.125, -0.125),
				"Trans(0.5,0.5,1)":              mat.Translate(0.5, 0.5, 1.0),
				"Trans(-0.5,-0.5,0)":            mat.Translate(-0.5, -0.5, 0.0),
				"Rot(1,0,0,0.2)":                mat.Rotate(1, 0, 0, 0.2),
				"Rot(1,0,0,-0.2)":               mat.Rotate(1, 0, 0, -0.2),
				"Rot(1,0,0,0.1)Trans(0.2,0,0)":  mat.Rotate(1, 0, 0, 0.1).Mul(mat.Translate(0.2, 0, 0)),
				"Rot(1,0,0,0.1)Trans(-0.2,0,0)": mat.Rotate(1, 0, 0, 0.1).Mul(mat.Translate(-0.2, 0, 0)),
				"Trans(0.2,0,0)Rot(1,0,0,0.1)":  mat.Translate(0.2, 0, 0).Mul(mat.Rotate(1, 0, 0, 0.1)),
				"Trans(-0.2,0,0)Rot(1,0,0,0.1)": mat.Translate(-0.2, 0, 0).Mul(mat.Rotate(1, 0, 0, 0.1)),
				"Rot(0,1,0,0.1)Trans(0.2,0,0)":  mat.Rotate(0, 1, 0, 0.1).Mul(mat.Translate(0.2, 0, 0)),
				"Rot(0,1,0,0.1)Trans(-0.2,0,0)": mat.Rotate(0, 1, 0, 0.1).Mul(mat.Translate(-0.2, 0, 0)),
				"Trans(0.2,0,0)Rot(0,1,0,0.1)":  mat.Translate(0.2, 0, 0).Mul(mat.Rotate(0, 1, 0, 0.1)),
				"Trans(-0.2,0,0)Rot(0,1,0,0.1)": mat.Translate(-0.2, 0, 0).Mul(mat.Rotate(0, 1, 0, 0.1)),
			} {
				delta := delta
				t.Run(name, func(t *testing.T) {
					indices := []int{
						3, 1, 4, 0, 2,
					}
					target := make(pc.Vec3Slice, len(indices))
					for i, id := range indices {
						target[i] = delta.Transform(base.Vec3At(id))
					}
					kdt := kdtree.New(base)
					ppicp := &PointToPointICPGradient{
						Evaluator: &PointToPointEvaluator{
							Corresponder: &NearestPointCorresponder{MaxDist: 2},
							MinPairs:     3,
						},
					}

					trans, err := ppicp.Fit(kdt, target)
					if err != nil {
						t.Fatal(err)
					}
					transformed := make(pc.Vec3Slice, len(indices))
					for i := range target {
						transformed[i] = trans.Transform(target[i])
					}
					var residual float32
					for i, id := range indices {
						residual += transformed[i].Sub(base.Vec3At(id)).NormSq()
					}
					residual /= float32(len(indices))
					if !(0.05 >= residual) { // checking NaN
						t.Errorf("Expected transform:\n%v\nGot:\n%v\n(residual: %f)", delta.Inv(), trans, residual)
					}
				})
			}
		})
	}
}
