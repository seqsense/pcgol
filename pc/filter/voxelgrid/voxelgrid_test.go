package voxelgrid

import (
	"math"
	"testing"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/internal/float"
)

func TestVoxelGrid(t *testing.T) {
	testCases := map[string]struct {
		opts           []Option
		expected       []mat.Vec3
		expectedLabels []uint32
	}{
		"Default": {
			expected: []mat.Vec3{
				{0.0000, 3.0000, 0.0000},
				{0.6375, 1.8750, 0.1375},
				{1.2500, 0.0000, 1.2500},
				{1.2500, 1.2625, 1.2500},
			},
			expectedLabels: []uint32{
				6, 1, 4, 2,
			},
		},
		"WithChunkSize881": {
			opts: []Option{WithChunkSize([3]int{8, 8, 1})},
			expected: []mat.Vec3{
				{0.0000, 3.0000, 0.0000},
				{0.6375, 1.8750, 0.1375},
				{1.2500, 0.0000, 1.2500},
				{1.2500, 1.2625, 1.2500},
			},
			expectedLabels: []uint32{
				6, 1, 4, 2,
			},
		},
		"WithChunkSize333": {
			// Output order may vary
			opts: []Option{WithChunkSize([3]int{3, 3, 3})},
			expected: []mat.Vec3{
				{0.6375, 1.8750, 0.1375},
				{0.0000, 3.0000, 0.0000},
				{1.2500, 0.0000, 1.2500},
				{1.2500, 1.2625, 1.2500},
			},
			expectedLabels: []uint32{
				1, 6, 4, 2,
			},
		},
	}

	for name, tt := range testCases {
		tt := tt
		t.Run(name, func(t *testing.T) {
			pp := pc.PointCloud{
				PointCloudHeader: pc.PointCloudHeader{
					Fields: []string{"x", "y", "z", "label"},
					Size:   []int{4, 4, 4, 4},
					Count:  []int{1, 1, 1, 1},
					Width:  6,
					Height: 1,
				},
				Points: 6,
				Data: float.Float32SliceAsByteSlice([]float32{
					0.625, 1.875, 0.125, math.Float32frombits(1),
					1.250, 1.250, 1.250, math.Float32frombits(2),
					0.650, 1.875, 0.150, math.Float32frombits(3),
					1.250, 0.000, 1.250, math.Float32frombits(4),
					1.250, 1.275, 1.250, math.Float32frombits(5),
					0.000, 3.000, 0.000, math.Float32frombits(6),
				}),
			}

			vg := New(mat.Vec3{0.125, 0.125, 0.125}, tt.opts...)
			out, err := vg.Filter(&pp)
			if err != nil {
				t.Fatal(err)
			}

			if len(tt.expected) != out.Points {
				t.Fatalf("Wrong number of points, expected: %d, got: %d", len(tt.expected), out.Points)
			}
			it, err := out.Vec3Iterator()
			if err != nil {
				t.Fatal(err)
			}
			lt, err := out.Uint32Iterator("label")
			if err != nil {
				t.Fatal(err)
			}

			for i, e := range tt.expected {
				p := it.Vec3()
				if !p.Equal(e) {
					t.Errorf("Expected point: %v, got: %v", e, p)
				}
				l := lt.Uint32()
				if l != tt.expectedLabels[i] {
					t.Errorf("Expected label: %x, got: %x", tt.expectedLabels[i], l)
				}
				it.Incr()
				lt.Incr()
			}
		})
	}
}
