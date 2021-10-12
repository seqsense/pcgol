package icp

import (
	"reflect"
	"testing"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/storage/kdtree"
)

func TestNearestPointCorresponder(t *testing.T) {
	base := pc.Vec3Slice{
		mat.Vec3{4, 1, 0},
		mat.Vec3{1, 1, 0},
		mat.Vec3{8, 1, 1},
		mat.Vec3{-5, 0, 1},
		mat.Vec3{0, 1, 0},
	}
	kdt := kdtree.New(base)
	corr := &NearestPointCorresponder{MaxDist: 3}

	targets := pc.Vec3Slice{
		mat.Vec3{8, 1, 1},  // nearest=2
		mat.Vec3{-8, 1, 1}, // no corresponding point
		mat.Vec3{2, 1, 0},  // nearest=1
	}
	pairs := corr.Pairs(kdt, targets)

	expected := []PointToPointCorrespondence{
		{2, 0, 0},
		{1, 2, 1},
	}
	if !reflect.DeepEqual(expected, pairs) {
		t.Errorf("Expected pairs: %v, got: %v", expected, pairs)
	}
}
