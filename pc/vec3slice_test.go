package pc

import (
	"testing"

	"github.com/seqsense/pcgol/mat"
)

func TestVec3Slice(t *testing.T) {
	var a Vec3RandomAccessor = Vec3Slice{
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
