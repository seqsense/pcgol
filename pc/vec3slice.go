package pc

import (
	"github.com/seqsense/pcgol/mat"
)

// Vec3Slice wraps []mat.Vec3 and implements Vec3RandomAccessor.
type Vec3Slice []mat.Vec3

func (v Vec3Slice) Len() int {
	return len(v)
}

func (v Vec3Slice) Vec3At(i int) mat.Vec3 {
	return v[i]
}
