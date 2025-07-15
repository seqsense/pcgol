package pc

import (
	"fmt"
	"reflect"
	"testing"

	"github.com/seqsense/pcgol/mat"
)

func TestIndiceVec3RandomAccessor(t *testing.T) {
	var ra Vec3RandomAccessor = Vec3Slice{
		mat.Vec3{1, 0, 0},
		mat.Vec3{2, 0, 0},
		mat.Vec3{3, 0, 0},
		mat.Vec3{4, 0, 0},
	}

	testCases := []struct {
		indices       []int
		expectedLen   int
		expectedVec3s Vec3Slice
	}{
		{
			indices:       []int{},
			expectedLen:   0,
			expectedVec3s: Vec3Slice{},
		},
		{
			indices:     []int{0, 1, 2, 3},
			expectedLen: 4,
			expectedVec3s: Vec3Slice{
				mat.Vec3{1, 0, 0},
				mat.Vec3{2, 0, 0},
				mat.Vec3{3, 0, 0},
				mat.Vec3{4, 0, 0},
			},
		},
		{
			indices:     []int{0, 2},
			expectedLen: 2,
			expectedVec3s: Vec3Slice{
				mat.Vec3{1, 0, 0},
				mat.Vec3{3, 0, 0},
			},
		},
	}

	for _, tt := range testCases {
		tt := tt
		t.Run(fmt.Sprintf("%v", tt.indices),
			func(t *testing.T) {
				iRa := NewIndiceVec3RandomAccessor(ra, tt.indices)
				if iRa.Len() != tt.expectedLen {
					t.Fatalf("Expected: %d, got %d", tt.expectedLen, iRa.Len())
				}

				vec3s := Vec3Slice{}
				indexes := []int{}
				for i := 0; i < iRa.Len(); i++ {
					vec3s = append(vec3s, iRa.Vec3At(i))
					indexes = append(indexes, iRa.RawIndexAt(i))
				}
				if !reflect.DeepEqual(tt.expectedVec3s, vec3s) {
					t.Fatalf("Expected: %v, got %v", tt.expectedVec3s, vec3s)
				}
				if !reflect.DeepEqual(tt.indices, indexes) {
					t.Fatalf("Expected indexes: %v, got %v", tt.indices, indexes)
				}
			},
		)
	}
}

type dummyUint32RandomAccessor struct {
	values []uint32
}

func (ra *dummyUint32RandomAccessor) Len() int {
	return len(ra.values)
}

func (ra *dummyUint32RandomAccessor) Uint32At(i int) uint32 {
	return ra.values[i]
}

func (ra *dummyUint32RandomAccessor) RawIndexAt(i int) int {
	return i
}

func TestIndiceUint32RandomAccessor(t *testing.T) {
	ra := &dummyUint32RandomAccessor{
		values: []uint32{1, 2, 3, 4},
	}

	testCases := []struct {
		indices         []int
		expectedLen     int
		expectedUint32s []uint32
	}{
		{
			indices:         []int{},
			expectedLen:     0,
			expectedUint32s: []uint32{},
		},
		{
			indices:         []int{0, 1, 2, 3},
			expectedLen:     4,
			expectedUint32s: []uint32{1, 2, 3, 4},
		},
		{
			indices:         []int{0, 2},
			expectedLen:     2,
			expectedUint32s: []uint32{1, 3},
		},
	}

	for _, tt := range testCases {
		tt := tt
		t.Run(fmt.Sprintf("%v", tt.indices),
			func(t *testing.T) {
				iRa := NewIndiceUint32RandomAccessor(ra, tt.indices)
				if iRa.Len() != tt.expectedLen {
					t.Fatalf("Expected: %d, got %d", tt.expectedLen, iRa.Len())
				}

				uint32s := []uint32{}
				indexes := []int{}
				for i := 0; i < iRa.Len(); i++ {
					uint32s = append(uint32s, iRa.Uint32At(i))
					indexes = append(indexes, iRa.RawIndexAt(i))
				}
				if !reflect.DeepEqual(tt.expectedUint32s, uint32s) {
					t.Fatalf("Expected: %v, got %v", tt.expectedUint32s, uint32s)
				}
				if !reflect.DeepEqual(tt.indices, indexes) {
					t.Fatalf("Expected indexes: %v, got %v", tt.indices, indexes)
				}
			},
		)
	}
}
