package icp

import (
	"testing"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/storage/kdtree"
)

func TestPointToPointICP(t *testing.T) {
	base := pc.Vec3Slice{
		mat.Vec3{0, 0, 0},
		mat.Vec3{1, 1, 0},
		mat.Vec3{2, 2, 0},
		mat.Vec3{3, 1, 1},
		mat.Vec3{4, 0, 0},
	}
	delta := mat.Vec3{0.25, 0.125, -0.125}
	expectedTrans := mat.Translate(delta[0], delta[1], delta[2])
	target := pc.Vec3Slice{
		base[2].Add(delta),
		base[3].Add(delta),
		base[4].Add(delta),
	}
	kdt := kdtree.New(base)
	ppicp := &PointToPointICP{
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
	if 0.01 < residual {
		t.Errorf("Expected transform:\n%v\nGot:\n%v\n(residual: %f)", expectedTrans, trans, residual)
	}
}
