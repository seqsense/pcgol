package testutil

import (
	"testing"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
)

func TestVec3Array(t *testing.T) {
	var a pc.Vec3RandomAccessor = Vec3Array{
		mat.Vec3{1, 2, 3},
		mat.Vec3{4, 5, 6},
	}

	if a.Len() != 2 {
		t.Fatalf("Expected len: %d, got: %d", 2, a.Len())
	}
	for i, expected := range []mat.Vec3{
		mat.Vec3{1, 2, 3},
		mat.Vec3{4, 5, 6},
	} {
		if v := a.Vec3At(i); !v.Equal(expected) {
			t.Errorf("Expected vec[%d]: %s, got: %s", i, expected, v)
		}
	}
}
