package pc

import (
	"github.com/seqsense/pcgol/mat"
)

type Vec3RandomAccessor interface {
	Vec3At(int) mat.Vec3
	Len() int
}
