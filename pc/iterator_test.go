package pc

import (
	"bytes"
	"testing"

	"github.com/seqsense/pcgol/mat"
)

func TestColorIterator(t *testing.T) {
	type colorPoint struct {
		v     mat.Vec3
		color Color
	}

	points := []colorPoint{
		{mat.Vec3{1.0, 2.0, 3.0}, Color{R: 255, G: 0, B: 0}},
		{mat.Vec3{4.0, 5.0, 6.0}, Color{R: 0, G: 255, B: 0}},
		{mat.Vec3{7.0, 8.0, 9.0}, Color{R: 0, G: 0, B: 255}},
	}

	pp := &PointCloud{
		PointCloudHeader: PointCloudHeader{
			Fields: []string{"x", "y", "z", "rgb"},
			Size:   []int{4, 4, 4, 4},
			Type:   []string{"F", "F", "F", "F"},
			Count:  []int{1, 1, 1, 1},
			Width:  3,
			Height: 1,
		},
		Points: 3,
		Data:   make([]byte, 3*16),
	}

	if ok := t.Run("SetColor", func(t *testing.T) {
		vt, err := pp.Vec3Iterator()
		if err != nil {
			t.Fatal(err)
		}
		ct, err := pp.ColorIterator()
		if err != nil {
			t.Fatal(err)
		}
		for _, p := range points {
			vt.SetVec3(p.v)
			ct.SetColor(p.color)
			vt.Incr()
			ct.Incr()
		}
	}); !ok {
		t.FailNow()
	}

	t.Run("Color", func(t *testing.T) {
		ct, err := pp.ColorIterator()
		if err != nil {
			t.Fatal(err)
		}
		for i, e := range points {
			if !ct.IsValid() {
				t.Fatalf("ColorIterator not valid at point %d", i)
			}
			if c := ct.Color(); c != e.color {
				t.Errorf("Point %d: expected %v, got %v", i, e.color, c)
			}
			ct.Incr()
		}
		if ct.IsValid() {
			t.Error("ColorIterator should be exhausted")
		}
	})

	t.Run("ColorAt", func(t *testing.T) {
		ct, err := pp.ColorIterator()
		if err != nil {
			t.Fatal(err)
		}
		for i, e := range points {
			if c := ct.ColorAt(i); c != e.color {
				t.Errorf("Point %d: expected %v, got %v", i, e.color, c)
			}
		}
	})

	t.Run("RawIndex", func(t *testing.T) {
		ct, err := pp.ColorIterator()
		if err != nil {
			t.Fatal(err)
		}
		for i := 0; ct.IsValid(); ct.Incr() {
			if ri := ct.RawIndex(); ri != i {
				t.Errorf("%d: Expected RawIndex: %d, got: %d", i, i, ri)
			}
			i++
		}
	})
}

func TestColorIteratorRGBAField(t *testing.T) {
	pp := &PointCloud{
		PointCloudHeader: PointCloudHeader{
			Version: 0.7,
			Fields:  []string{"x", "y", "z", "rgba"},
			Size:    []int{4, 4, 4, 4},
			Type:    []string{"F", "F", "F", "U"},
			Count:   []int{1, 1, 1, 1},
			Width:   1,
			Height:  1,
		},
		Points: 1,
		Data:   make([]byte, 16),
	}

	ct, err := pp.ColorIterator()
	if err != nil {
		t.Fatal(err)
	}

	expected := Color{R: 255, G: 128, B: 64, A: 200}
	ct.SetColor(expected)

	ct2, err := pp.ColorIterator()
	if err != nil {
		t.Fatal(err)
	}
	got := ct2.Color()
	if got != expected {
		t.Errorf("Expected %v, got %v", expected, got)
	}
}

func TestColorIteratorNoField(t *testing.T) {
	pp := &PointCloud{
		PointCloudHeader: PointCloudHeader{
			Fields: []string{"x", "y", "z"},
			Size:   []int{4, 4, 4},
			Type:   []string{"F", "F", "F"},
			Count:  []int{1, 1, 1},
		},
	}

	_, err := pp.ColorIterator()
	if err == nil {
		t.Error("Expected error for missing rgb/rgba field")
	}
}

func TestVec3Iterator(t *testing.T) {
	testCases := map[string]struct {
		in            *PointCloud
		expectedBytes []byte
	}{
		"BinaryIterator": {
			in: &PointCloud{
				PointCloudHeader: PointCloudHeader{
					Fields: []string{"x", "y", "z"},
					Size:   []int{4, 4, 4},
					Count:  []int{1, 1, 1},
					Width:  3,
					Height: 1,
				},
				Points: 3,
				Data:   make([]byte, 3*4*3),
			},
			expectedBytes: []byte{
				0x00, 0x00, 0x80, 0x3F, // 1.0
				0x00, 0x00, 0x00, 0x40, // 2.0
				0x00, 0x00, 0x40, 0x40, // 3.0
				0x00, 0x00, 0x80, 0x40, // 4.0
				0x00, 0x00, 0xA0, 0x40, // 5.0
				0x00, 0x00, 0xC0, 0x40, // 6.0
				0x00, 0x00, 0xE0, 0x40, // 7.0
				0x00, 0x00, 0x00, 0x41, // 8.0
				0x00, 0x00, 0x10, 0x41, // 9.0
			},
		},
		"NaiveIterator": {
			in: &PointCloud{
				PointCloudHeader: PointCloudHeader{
					Fields: []string{"y", "z", "x"},
					Size:   []int{4, 4, 4},
					Count:  []int{1, 1, 1},
					Width:  3,
					Height: 1,
				},
				Points: 3,
				Data:   make([]byte, 3*4*3),
			},
			expectedBytes: []byte{
				0x00, 0x00, 0x00, 0x40, // 2.0
				0x00, 0x00, 0x40, 0x40, // 3.0
				0x00, 0x00, 0x80, 0x3F, // 1.0
				0x00, 0x00, 0xA0, 0x40, // 5.0
				0x00, 0x00, 0xC0, 0x40, // 6.0
				0x00, 0x00, 0x80, 0x40, // 4.0
				0x00, 0x00, 0x00, 0x41, // 8.0
				0x00, 0x00, 0x10, 0x41, // 9.0
				0x00, 0x00, 0xE0, 0x40, // 7.0
			},
		},
	}
	for name, tt := range testCases {
		tt := tt
		t.Run(name, func(t *testing.T) {
			pp := tt.in

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

				if !bytes.Equal(tt.expectedBytes, pp.Data) {
					t.Errorf("Expected data: %v, got: %v", tt.expectedBytes, pp.Data)
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

			t.Run("Vec3At", func(t *testing.T) {
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
					if v := it.Vec3At(i); !v.Equal(expectedVec) {
						t.Errorf("%d: Expected Vec3: %v, got: %v", i, expectedVec, v)
					}
				}
			})

			t.Run("RawIndex", func(t *testing.T) {
				it, err := pp.Vec3Iterator()
				if err != nil {
					t.Fatal(err)
				}
				for i := 0; it.IsValid(); it.Incr() {
					if ri := it.RawIndex(); ri != i {
						t.Errorf("%d: Expected RawIndex: %d, got: %d", i, i, ri)
					}
					i++
				}
			})

			t.Run("RawIndexAt", func(t *testing.T) {
				it, err := pp.Vec3Iterator()
				if err != nil {
					t.Fatal(err)
				}
				for i := 0; i < 3; i++ {
					if ri := it.RawIndexAt(i); ri != i {
						t.Errorf("%d: Expected RawIndex: %d, got: %d", i, i, ri)
					}
				}
			})
		})
	}
	t.Run("Vec3_XYZ", func(t *testing.T) {
		pp := &PointCloud{
			PointCloudHeader: PointCloudHeader{
				Fields: []string{"xyz"},
				Size:   []int{4},
				Count:  []int{3},
				Width:  3,
				Height: 1,
			},
			Points: 3,
			Data: []byte{
				0x00, 0x00, 0x80, 0x3F, // 1.0
				0x00, 0x00, 0x00, 0x40, // 2.0
				0x00, 0x00, 0x40, 0x40, // 3.0
				0x00, 0x00, 0x80, 0x40, // 4.0
				0x00, 0x00, 0xA0, 0x40, // 5.0
				0x00, 0x00, 0xC0, 0x40, // 6.0
				0x00, 0x00, 0xE0, 0x40, // 7.0
				0x00, 0x00, 0x00, 0x41, // 8.0
				0x00, 0x00, 0x10, 0x41, // 9.0
			},
		}
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

func TestUint32Iterator(t *testing.T) {
	pp := PointCloud{
		PointCloudHeader: PointCloudHeader{
			Fields: []string{"label"},
			Type:   []string{"U"},
			Size:   []int{4},
			Count:  []int{1},
			Width:  3,
			Height: 1,
		},
		Points: 3,
		Data:   make([]byte, 3*4),
	}
	if ok := t.Run("SetUint32", func(t *testing.T) {
		it, err := pp.Uint32Iterator("label")
		if err != nil {
			t.Fatal(err)
		}
		it.SetUint32(1)
		it.Incr()
		it.SetUint32(2)
		it.Incr()
		it.SetUint32(3)

		bytesExpected := []byte{
			0x01, 0x00, 0x00, 0x00, // 1
			0x02, 0x00, 0x00, 0x00, // 2
			0x03, 0x00, 0x00, 0x00, // 3
		}
		if !bytes.Equal(bytesExpected, pp.Data) {
			t.Errorf("Expected data: %v, got: %v", bytesExpected, pp.Data)
		}
	}); !ok {
		t.FailNow()
	}

	t.Run("Uint32", func(t *testing.T) {
		it, err := pp.Uint32Iterator("label")
		if err != nil {
			t.Fatal(err)
		}
		expectedLabels := []uint32{1, 2, 3}
		for i, expectedLabel := range expectedLabels {
			if !it.IsValid() {
				t.Fatalf("Iterator is invalid at position %d", i)
			}
			if v := it.Uint32(); v != expectedLabel {
				t.Errorf("Expected: %v, got: %v", expectedLabel, v)
			}
			it.Incr()
		}
	})

	t.Run("Uint32At", func(t *testing.T) {
		it, err := pp.Uint32Iterator("label")
		if err != nil {
			t.Fatal(err)
		}
		expectedLabels := []uint32{1, 2, 3}
		for i, expectedLabel := range expectedLabels {
			if !it.IsValid() {
				t.Fatalf("Iterator is invalid at position %d", i)
			}
			if v := it.Uint32At(i); v != expectedLabel {
				t.Errorf("Expected: %v, got: %v", expectedLabel, v)
			}
		}
	})

	t.Run("RawIndex", func(t *testing.T) {
		it, err := pp.Uint32Iterator("label")
		if err != nil {
			t.Fatal(err)
		}
		for i := 0; i < 3; i++ {
			if ri := it.RawIndex(); ri != i {
				t.Errorf("%d: Expected RawIndex: %d, got: %d", i, i, ri)
			}
			it.Incr()
		}
	})

	t.Run("RawIndexAt", func(t *testing.T) {
		it, err := pp.Uint32Iterator("label")
		if err != nil {
			t.Fatal(err)
		}
		for i := 0; i < 3; i++ {
			if ri := it.RawIndexAt(i); ri != i {
				t.Errorf("%d: Expected RawIndex: %d, got: %d", i, i, ri)
			}
		}
	})
}

func TestFloat32IteratorAndUint32Iterator(t *testing.T) {
	pp := PointCloud{
		PointCloudHeader: PointCloudHeader{
			Fields: []string{"x", "label"},
			Type:   []string{"F", "U"},
			Size:   []int{4, 4},
			Count:  []int{1, 1},
			Width:  3,
			Height: 1,
		},
		Points: 3,
		Data:   make([]byte, 3*4*2),
	}
	if ok := t.Run("SetFloat32SetUint32", func(t *testing.T) {
		it, err := pp.Float32Iterator("x")
		if err != nil {
			t.Fatal(err)
		}
		lt, err := pp.Uint32Iterator("label")
		if err != nil {
			t.Fatal(err)
		}
		it.SetFloat32(1.0)
		it.Incr()
		it.SetFloat32(2.0)
		it.Incr()
		it.SetFloat32(3.0)

		lt.SetUint32(1)
		lt.Incr()
		lt.SetUint32(2)
		lt.Incr()
		lt.SetUint32(3)

		bytesExpected := []byte{
			0x00, 0x00, 0x80, 0x3F, // 1.0
			0x01, 0x00, 0x00, 0x00, // 1
			0x00, 0x00, 0x00, 0x40, // 2.0
			0x02, 0x00, 0x00, 0x00, // 2
			0x00, 0x00, 0x40, 0x40, // 3.0
			0x03, 0x00, 0x00, 0x00, // 3
		}
		if !bytes.Equal(bytesExpected, pp.Data) {
			t.Errorf("Expected data: %v, got: %v", bytesExpected, pp.Data)
		}
	}); !ok {
		t.FailNow()
	}

	t.Run("Float32Uint32", func(t *testing.T) {
		it, err := pp.Float32Iterator("x")
		if err != nil {
			t.Fatal(err)
		}
		lt, err := pp.Uint32Iterator("label")
		if err != nil {
			t.Fatal(err)
		}
		expectedXs := []float32{1.0, 2.0, 3.0}
		for i, expectedX := range expectedXs {
			if !it.IsValid() {
				t.Fatalf("Iterator is invalid at position %d", i)
			}
			if v := it.Float32(); v != expectedX {
				t.Errorf("Expected: %v, got: %v", expectedX, v)
			}
			it.Incr()
		}

		expectedLabels := []uint32{1, 2, 3}
		for i, expectedLabel := range expectedLabels {
			if !lt.IsValid() {
				t.Fatalf("Iterator is invalid at position %d", i)
			}
			if v := lt.Uint32(); v != expectedLabel {
				t.Errorf("Expected: %v, got: %v", expectedLabel, v)
			}
			lt.Incr()
		}
	})

	t.Run("Float32Uint32At", func(t *testing.T) {
		it, err := pp.Float32Iterator("x")
		if err != nil {
			t.Fatal(err)
		}
		lt, err := pp.Uint32Iterator("label")
		if err != nil {
			t.Fatal(err)
		}
		expectedXs := []float32{1.0, 2.0, 3.0}
		for i, expectedX := range expectedXs {
			if !it.IsValid() {
				t.Fatalf("Iterator is invalid at position %d", i)
			}
			if v := it.Float32At(i); v != expectedX {
				t.Errorf("Expected: %v, got: %v", expectedX, v)
			}
		}

		expectedLabels := []uint32{1, 2, 3}
		for i, expectedLabel := range expectedLabels {
			if !lt.IsValid() {
				t.Fatalf("Iterator is invalid at position %d", i)
			}
			if v := lt.Uint32At(i); v != expectedLabel {
				t.Errorf("Expected: %v, got: %v", expectedLabel, v)
			}
		}
	})
}

func TestUint32IteratorAndFloat32Iterator(t *testing.T) {
	pp := PointCloud{
		PointCloudHeader: PointCloudHeader{
			Fields: []string{"label", "x"},
			Type:   []string{"U", "F"},
			Size:   []int{4, 4},
			Count:  []int{1, 1},
			Width:  3,
			Height: 1,
		},
		Points: 3,
		Data:   make([]byte, 3*4*2),
	}
	if ok := t.Run("SetUint32SetFloat32", func(t *testing.T) {
		lt, err := pp.Uint32Iterator("label")
		if err != nil {
			t.Fatal(err)
		}
		it, err := pp.Float32Iterator("x")
		if err != nil {
			t.Fatal(err)
		}
		lt.SetUint32(1)
		lt.Incr()
		lt.SetUint32(2)
		lt.Incr()
		lt.SetUint32(3)

		it.SetFloat32(1.0)
		it.Incr()
		it.SetFloat32(2.0)
		it.Incr()
		it.SetFloat32(3.0)

		bytesExpected := []byte{
			0x01, 0x00, 0x00, 0x00, // 1
			0x00, 0x00, 0x80, 0x3F, // 1.0
			0x02, 0x00, 0x00, 0x00, // 2
			0x00, 0x00, 0x00, 0x40, // 2.0
			0x03, 0x00, 0x00, 0x00, // 3
			0x00, 0x00, 0x40, 0x40, // 3.0
		}
		if !bytes.Equal(bytesExpected, pp.Data) {
			t.Errorf("Expected data: %v, got: %v", bytesExpected, pp.Data)
		}
	}); !ok {
		t.FailNow()
	}

	t.Run("Uint32Float32", func(t *testing.T) {
		it, err := pp.Float32Iterator("x")
		if err != nil {
			t.Fatal(err)
		}
		lt, err := pp.Uint32Iterator("label")
		if err != nil {
			t.Fatal(err)
		}
		expectedLabels := []uint32{1, 2, 3}
		for i, expectedLabel := range expectedLabels {
			if !lt.IsValid() {
				t.Fatalf("Iterator is invalid at position %d", i)
			}
			if v := lt.Uint32(); v != expectedLabel {
				t.Errorf("Expected: %v, got: %v", expectedLabel, v)
			}
			lt.Incr()
		}
		expectedXs := []float32{1.0, 2.0, 3.0}
		for i, expectedX := range expectedXs {
			if !it.IsValid() {
				t.Fatalf("Iterator is invalid at position %d", i)
			}
			if v := it.Float32(); v != expectedX {
				t.Errorf("Expected: %v, got: %v", expectedX, v)
			}
			it.Incr()
		}
	})

	t.Run("Uint32Float32At", func(t *testing.T) {
		lt, err := pp.Uint32Iterator("label")
		if err != nil {
			t.Fatal(err)
		}
		it, err := pp.Float32Iterator("x")
		if err != nil {
			t.Fatal(err)
		}
		expectedLabels := []uint32{1, 2, 3}
		for i, expectedLabel := range expectedLabels {
			if !lt.IsValid() {
				t.Fatalf("Iterator is invalid at position %d", i)
			}
			if v := lt.Uint32At(i); v != expectedLabel {
				t.Errorf("Expected: %v, got: %v", expectedLabel, v)
			}
		}
		expectedXs := []float32{1.0, 2.0, 3.0}
		for i, expectedX := range expectedXs {
			if !it.IsValid() {
				t.Fatalf("Iterator is invalid at position %d", i)
			}
			if v := it.Float32At(i); v != expectedX {
				t.Errorf("Expected: %v, got: %v", expectedX, v)
			}
		}
	})
}

func TestBinaryFloat32Iterator(t *testing.T) {

	data := make([]byte, 3*4)

	if ok := t.Run("SetFloat32", func(t *testing.T) {
		it := binaryFloat32Iterator{
			binaryIterator{
				data:   data,
				pos:    0,
				stride: 4,
			},
		}
		it.SetFloat32(1.0)
		it.Incr()
		it.SetFloat32(2.0)
		it.Incr()
		it.SetFloat32(3.0)
		bytesExpected := []byte{
			0x00, 0x00, 0x80, 0x3F, // 1.0
			0x00, 0x00, 0x00, 0x40, // 2.0
			0x00, 0x00, 0x40, 0x40, // 3.0
		}
		if !bytes.Equal(bytesExpected, data) {
			t.Errorf("Expected data: %v, got: %v", bytesExpected, data)
		}
	}); !ok {
		t.FailNow()
	}

	t.Run("Float32", func(t *testing.T) {
		it := binaryFloat32Iterator{
			binaryIterator{
				data:   data,
				pos:    0,
				stride: 4,
			},
		}
		expectedXs := []float32{1.0, 2.0, 3.0}
		for i, expectedX := range expectedXs {
			if !it.IsValid() {
				t.Fatalf("Iterator is invalid at position %d", i)
			}
			if v := it.Float32(); v != expectedX {
				t.Errorf("Expected: %v, got: %v", expectedX, v)
			}
			it.Incr()
		}
	})

	t.Run("Float32At", func(t *testing.T) {
		it := binaryFloat32Iterator{
			binaryIterator{
				data:   data,
				pos:    0,
				stride: 4,
			},
		}
		expectedXs := []float32{1.0, 2.0, 3.0}
		for i, expectedX := range expectedXs {
			if !it.IsValid() {
				t.Fatalf("Iterator is invalid at position %d", i)
			}
			if v := it.Float32At(i); v != expectedX {
				t.Errorf("Expected: %v, got: %v", expectedX, v)
			}
		}
	})

	t.Run("RawIndex", func(t *testing.T) {
		it := binaryFloat32Iterator{
			binaryIterator{
				data:   data,
				pos:    0,
				stride: 4,
			},
		}
		for i := 0; i < 3; i++ {
			if ri := it.RawIndex(); ri != i {
				t.Errorf("%d: Expected RawIndex: %d, got: %d", i, i, ri)
			}
			it.Incr()
		}
	})

	t.Run("RawIndexAt", func(t *testing.T) {
		it := binaryFloat32Iterator{
			binaryIterator{
				data:   data,
				pos:    0,
				stride: 4,
			},
		}
		for i := 0; i < 3; i++ {
			if ri := it.RawIndexAt(i); ri != i {
				t.Errorf("%d: Expected RawIndex: %d, got: %d", i, i, ri)
			}
		}
	})
}

func BenchmarkFloat32Iterator(b *testing.B) {
	const num = 1024
	testCases := map[string]struct {
		data func() Vec3Iterator
	}{
		"binaryIterator": {
			data: func() Vec3Iterator {
				data := make([]byte, 3*4*num)
				return naiveVec3Iterator{
					&binaryFloat32Iterator{
						binaryIterator: binaryIterator{
							data:   data,
							pos:    0,
							stride: 3 * 4,
						},
					},
					&binaryFloat32Iterator{
						binaryIterator: binaryIterator{
							data:   data,
							pos:    4,
							stride: 3 * 4,
						},
					},
					&binaryFloat32Iterator{
						binaryIterator: binaryIterator{
							data:   data,
							pos:    8,
							stride: 3 * 4,
						},
					},
				}
			},
		},
		"float32Iterator": {
			data: func() Vec3Iterator {
				return &float32Iterator{
					data:   make([]float32, 3*num),
					pos:    0,
					stride: 3,
				}
			},
		},
	}
	b.Run("Vec3", func(b *testing.B) {
		for name, tt := range testCases {
			b.Run(name, func(b *testing.B) {
				var in, out Vec3Iterator
				for i := 0; i < b.N; i++ {
					if i%num == 0 {
						b.StopTimer()
						in, out = tt.data(), tt.data()
						b.StartTimer()
					}
					out.SetVec3(in.Vec3())
					in.Incr()
					out.Incr()
				}
			})
		}
	})
}
