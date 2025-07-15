package pc

import (
	"testing"

	"github.com/seqsense/pcgol/mat"
)

func TestVec3RandomAccessorIterator(t *testing.T) {
	vs := Vec3Slice([]mat.Vec3{
		{0.0, 1.0, 0.0},
		{0.0, 2.0, 0.0},
		{0.0, 3.0, 0.0},
	})
	it := NewVec3RandomAccessorIterator(vs)
	for i := 0; i < 3; i++ {
		if !it.IsValid() {
			t.Fatalf("Iterator is invalid at position %d", i)
		}
		if v := it.Vec3(); !v.Equal(vs.Vec3At(i)) {
			t.Errorf("%d: Expected Vec3: %v, got: %v", i, vs.Vec3At(i), v)
		}
		if ri := it.(Vec3ForwardIteraterRawIndexer).RawIndex(); ri != i {
			t.Errorf("%d: Expected RawIndex: %d, got: %d", i, i, ri)
		}
		it.Incr()
	}
	if it.IsValid() {
		t.Fatalf("Iterator must be invalid at the end")
	}
}
