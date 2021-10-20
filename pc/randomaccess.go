package pc

import (
	"github.com/seqsense/pcgol/mat"
)

type Vec3RandomAccessor interface {
	Vec3At(int) mat.Vec3
	Len() int
}

type Uint32RandomAccessor interface {
	Uint32At(int) uint32
	Len() int
}
