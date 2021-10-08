package icp

import (
	"reflect"
	"testing"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc/storage/kdtree"
	"github.com/seqsense/pcgol/pc/testutil"
)

func TestNearestPointCorresponder(t *testing.T) {
	base := testutil.Vec3Array{
		mat.Vec3{4, 1, 0},
		mat.Vec3{1, 1, 0},
		mat.Vec3{8, 1, 1},
		mat.Vec3{-5, 0, 1},
		mat.Vec3{0, 1, 0},
	}
	kdt := kdtree.New(base)
	corr := NewNearestPointCorresponder(kdt, 3)

	targets := testutil.Vec3Array{
		mat.Vec3{8, 1, 1},  // nearest=2
		mat.Vec3{-8, 1, 1}, // no corresponding point
		mat.Vec3{2, 1, 0},  // nearest=1
	}
	pairs := corr.Pairs(targets)

	expected := []Correspondence{
		{2, 0, 0},
		{1, 2, 1},
	}
	if !reflect.DeepEqual(expected, pairs) {
		t.Errorf("Expected pairs: %v, got: %v", expected, pairs)
	}
}
