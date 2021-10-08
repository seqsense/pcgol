package icp

import (
	"testing"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
)

func TestCenter(t *testing.T) {
	targets := pc.Vec3Slice{
		mat.Vec3{1, 1, 4},
		mat.Vec3{2, 1, 2},
		mat.Vec3{3, 1, 6},
	}
	out := center(targets)
	expected := mat.Vec3{2, 1, 4}
	if !expected.Equal(out) {
		t.Errorf("Expected: %v, got: %v", expected, out)
	}
}
