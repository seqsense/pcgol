package pc

import (
	"testing"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc/internal/float"
)

func TestMinMaxVec3(t *testing.T) {
	pp := PointCloud{
		PointCloudHeader: PointCloudHeader{
			Fields: []string{"x", "y", "z"},
			Size:   []int{4, 4, 4},
			Count:  []int{1, 1, 1},
			Width:  3,
			Height: 1,
		},
		Points: 3,
		Data: float.Float32SliceAsByteSlice([]float32{
			10.1, -20.2, 3.3,
			1.1, 2.2, 4.3,
			15.1, 21.2, 0.3,
		}),
	}

	expectedMin := mat.Vec3{1.1, -20.2, 0.3}
	expectedMax := mat.Vec3{15.1, 21.2, 4.3}

	min, max, err := MinMaxVec3(&pp)
	if err != nil {
		t.Fatal(err)
	}

	if !expectedMin.Equal(min) {
		t.Errorf("Expected min: %v, got: %v", expectedMin, min)
	}
	if !expectedMax.Equal(max) {
		t.Errorf("Expected max: %v, got: %v", expectedMax, max)
	}
}
