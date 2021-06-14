package pc

import (
	"bytes"
	"testing"

	"github.com/seqsense/pcgol/mat"
)

func TestVec3Iterator(t *testing.T) {
	pp := PointCloud{
		PointCloudHeader: PointCloudHeader{
			Fields: []string{"x", "y", "z"},
			Size:   []int{4, 4, 4},
			Count:  []int{1, 1, 1},
			Width:  3,
			Height: 1,
		},
		Points: 3,
		Data:   make([]byte, 3*4*3),
	}

	if ok := t.Run("SetVec3", func(t *testing.T) {
		it, err := pp.Vec3Iterator()
		if err != nil {
			t.Fatal(err)
		}
		it.SetVec3(mat.Vec3{1, 2, 3})
		it.Incr()
		it.SetVec3(mat.Vec3{4, 5, 6})
		it.Incr()
		it.SetVec3(mat.Vec3{7, 8, 9})

		bytesExpected := []byte{
			0x00, 0x00, 0x80, 0x3F, // 1.0
			0x00, 0x00, 0x00, 0x40, // 2.0
			0x00, 0x00, 0x40, 0x40, // 3.0
			0x00, 0x00, 0x80, 0x40, // 4.0
			0x00, 0x00, 0xA0, 0x40, // 5.0
			0x00, 0x00, 0xC0, 0x40, // 6.0
			0x00, 0x00, 0xE0, 0x40, // 7.0
			0x00, 0x00, 0x00, 0x41, // 8.0
			0x00, 0x00, 0x10, 0x41, // 9.0
		}
		if !bytes.Equal(bytesExpected, pp.Data) {
			t.Errorf("Expected data: %v, got: %v", bytesExpected, pp.Data)
		}
	}); !ok {
		t.FailNow()
	}

	t.Run("Vec3", func(t *testing.T) {
		it, err := pp.Vec3Iterator()
		if err != nil {
			t.Fatal(err)
		}
		expectedVecs := []mat.Vec3{
			{1, 2, 3},
			{4, 5, 6},
			{7, 8, 9},
		}
		for i, expectedVec := range expectedVecs {
			if !it.IsValid() {
				t.Fatalf("Iterator is invalid at position %d", i)
			}
			if v := it.Vec3(); !v.Equal(expectedVec) {
				t.Errorf("Expected Vec3: %v, got: %v", expectedVec, v)
			}
			it.Incr()
		}
	})

	pp.PointCloudHeader = PointCloudHeader{
		Fields: []string{"xyz"},
		Size:   []int{4},
		Count:  []int{3},
		Width:  3,
		Height: 1,
	}
	t.Run("Vec3_XYZ", func(t *testing.T) {
		it, err := pp.Vec3Iterator()
		if err != nil {
			t.Fatal(err)
		}
		expectedVecs := []mat.Vec3{
			{1, 2, 3},
			{4, 5, 6},
			{7, 8, 9},
		}
		for i, expectedVec := range expectedVecs {
			if !it.IsValid() {
				t.Fatalf("Iterator is invalid at position %d", i)
			}
			if v := it.Vec3(); !v.Equal(expectedVec) {
				t.Errorf("Expected Vec3: %v, got: %v", expectedVec, v)
			}
			it.Incr()
		}
	})

}
