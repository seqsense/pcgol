package icp

import (
	"testing"

	"github.com/seqsense/pcgol/mat"
)

func TestRodriguesToRotation(t *testing.T) {
	const eps = 0.001
	for vx := float32(-1.0); vx < 1; vx += 0.02 {
		for vy := float32(-1.0); vy < 1; vy += 0.02 {
			for vz := float32(-1.0); vz < 1; vz += 0.02 {
				v := mat.Vec3{vx, vy, vz}
				r := rodriguesToRotation(v)

				vn := v.Normalized()
				// TODO: fix mat.Rotate that is transposed
				expected := mat.Rotate(vn[0], vn[1], vn[2], -v.Norm())

				for i := range r {
					diff := r[i] - expected[i]
					if diff < -eps || eps < diff {
						t.Fatalf("Expected:\n%v\nGot:\n%v", expected, r)
					}
				}
			}
		}
	}
}
