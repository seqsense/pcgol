package pc

import (
	"bytes"
	"testing"

	"github.com/seqsense/pcgol/mat"
)

func TestPointCloudHeader_TypeEqual(t *testing.T) {
	ph0 := PointCloudHeader{
		Fields: []string{"x", "y", "i"},
		Size:   []int{4, 4, 2},
		Type:   []string{"F", "F", "U"},
		Count:  []int{1, 1, 1},
	}
	ph1 := PointCloudHeader{
		Fields: []string{"x", "y", "i"},
		Size:   []int{4, 4, 2},
		Type:   []string{"F", "F", "U"},
		Count:  []int{1, 1, 1},
	}
	ph2 := PointCloudHeader{
		Fields: []string{"x", "y"},
		Size:   []int{4, 4},
		Type:   []string{"F", "F"},
		Count:  []int{1, 1},
	}

	testCases := map[string]struct {
		ph0, ph1 *PointCloudHeader
		expected bool
	}{
		"SameFileds": {
			ph0:      &ph0,
			ph1:      &ph1,
			expected: true,
		},
		"SameFileds_Reversed": {
			ph0:      &ph1,
			ph1:      &ph0,
			expected: true,
		},
		"SamePtr": {
			ph0:      &ph0,
			ph1:      &ph0,
			expected: true,
		},
		"DifferentFields": {
			ph0:      &ph0,
			ph1:      &ph2,
			expected: false,
		},
		"DifferentFields_Reversed": {
			ph0:      &ph2,
			ph1:      &ph0,
			expected: false,
		},
	}
	for name, tt := range testCases {
		tt := tt
		t.Run(name, func(t *testing.T) {
			ret := tt.ph0.TypeEqual(tt.ph1)
			if ret != tt.expected {
				t.Errorf("Expected %v, got %v", tt.expected, ret)
			}
		})
	}
}

func TestPointCloud_CopyTo(t *testing.T) {
	pp0 := &PointCloud{
		PointCloudHeader: PointCloudHeader{
			Fields: []string{"x", "y", "z"},
			Size:   []int{4, 4, 4},
			Type:   []string{"F", "F", "F"},
			Count:  []int{1, 1, 1},
			Width:  3,
			Height: 1,
		},
		Points: 3,
		Data:   make([]byte, 3*4*3),
	}
	it, err := pp0.Vec3Iterator()
	if err != nil {
		t.Fatal(err)
	}
	it.SetVec3(mat.Vec3{1, 2, 3})
	it.Incr()
	it.SetVec3(mat.Vec3{4, 5, 6})
	it.Incr()
	it.SetVec3(mat.Vec3{7, 8, 9})

	pp1 := &PointCloud{
		PointCloudHeader: pp0.Clone(),
		Points:           2,
		Data:             make([]byte, 2*4*3),
	}
	pp1.Width = 2
	pp0.CopyTo(pp1, 0, 1, 2)

	bytesExpected := []byte{
		0x00, 0x00, 0x80, 0x40, // 4.0
		0x00, 0x00, 0xA0, 0x40, // 5.0
		0x00, 0x00, 0xC0, 0x40, // 6.0
		0x00, 0x00, 0xE0, 0x40, // 7.0
		0x00, 0x00, 0x00, 0x41, // 8.0
		0x00, 0x00, 0x10, 0x41, // 9.0
	}
	if !bytes.Equal(bytesExpected, pp1.Data) {
		t.Errorf("Expected data: %v, got: %v", bytesExpected, pp1.Data)
	}
}
