package pc

import (
	"github.com/seqsense/pcgol/mat"
)

type Vec3RandomAccessor interface {
	Vec3At(int) mat.Vec3
	Len() int
}

type Vec3RandomAccesserRawIndexer interface {
	RawIndexAt(int) int
}

type Uint32RandomAccessor interface {
	Uint32At(int) uint32
	Len() int
}

type vec3RandomAccessorIterator struct {
	Vec3RandomAccessor
	pos int
}

func NewVec3RandomAccessorIterator(ra Vec3RandomAccessor) Vec3ConstForwardIterator {
	return &vec3RandomAccessorIterator{
		Vec3RandomAccessor: ra,
	}
}

func (i *vec3RandomAccessorIterator) Incr() {
	i.pos++
}

func (i *vec3RandomAccessorIterator) IsValid() bool {
	return i.pos < i.Len()
}

func (i *vec3RandomAccessorIterator) Vec3() mat.Vec3 {
	return i.Vec3At(i.pos)
}

// RawIndex implements Vec3ForwardIteraterRawIndexer.
// This is mainly for internal use and the parent Vec3RandomAccessor must also implement Vec3RandomAccesserRawIndexer.
func (i *vec3RandomAccessorIterator) RawIndex() int {
	return i.Vec3RandomAccessor.(Vec3RandomAccesserRawIndexer).RawIndexAt(i.pos)
}
