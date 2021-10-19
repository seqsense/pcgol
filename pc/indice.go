package pc

import (
	"github.com/seqsense/pcgol/mat"
)

type indiceVec3RandomAccessor struct {
	indice []int
	ra     Vec3RandomAccessor
}

func (i *indiceVec3RandomAccessor) Len() int {
	return len(i.indice)
}

func (i *indiceVec3RandomAccessor) Vec3At(j int) mat.Vec3 {
	return i.ra.Vec3At(i.indice[j])
}

func NewIndiceVec3RandomAccessor(ra Vec3RandomAccessor, indice []int) Vec3RandomAccessor {
	return &indiceVec3RandomAccessor{
		ra:     ra,
		indice: indice,
	}
}

type indiceUint32RandomAccessor struct {
	indice []int
	ra     Uint32RandomAccessor
}

func (i *indiceUint32RandomAccessor) Len() int {
	return len(i.indice)
}

func (i *indiceUint32RandomAccessor) Uint32At(j int) uint32 {
	return i.ra.Uint32At(i.indice[j])
}

func NewIndiceUint32RandomAccessor(ra Uint32RandomAccessor, indice []int) Uint32RandomAccessor {
	return &indiceUint32RandomAccessor{
		ra:     ra,
		indice: indice,
	}
}
