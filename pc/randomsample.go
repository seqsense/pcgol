package pc

import (
	"math"
	"math/rand"

	"github.com/seqsense/pcgol/mat"
)

// NewVec3RandomSampleIterator creates Vec3 iterator with random sampling.
// Sampled data won't duplicate.
func NewVec3RandomSampleIterator(ra Vec3RandomAccessor, ratio float32) Vec3ConstForwardIterator {
	switch {
	case ratio <= 0:
		return &vec3EmptyIterator{}
	case ratio >= 1:
		if it, ok := ra.(Vec3ConstForwardIterator); ok {
			return it
		}
		return &vec3RandomAccessToIterator{ra: ra}
	}
	expectedInterval := 1 / ratio
	return &vec3RandomSampleIterator{
		ra:     ra,
		Lambda: 1 / (expectedInterval - 1),
	}
}

type vec3RandomSampleIterator struct {
	ra Vec3RandomAccessor

	Lambda float32
	pos    float32
}

func (s *vec3RandomSampleIterator) Incr() {
	// Jump position by A-ExpJ.
	inc := -float32(math.Log(float64(1-rand.Float32()))) / s.Lambda
	s.pos += 1 + inc
}

func (s *vec3RandomSampleIterator) IsValid() bool {
	return int(s.pos) < s.ra.Len()
}

func (s *vec3RandomSampleIterator) Vec3() mat.Vec3 {
	return s.ra.Vec3At(int(s.pos))
}

type vec3RandomAccessToIterator struct {
	ra Vec3RandomAccessor

	pos float32
}

func (s *vec3RandomAccessToIterator) Incr()          { s.pos++ }
func (s *vec3RandomAccessToIterator) IsValid() bool  { return int(s.pos) < s.ra.Len() }
func (s *vec3RandomAccessToIterator) Vec3() mat.Vec3 { return s.ra.Vec3At(int(s.pos)) }

type vec3EmptyIterator struct{}

func (vec3EmptyIterator) Incr()          {}
func (vec3EmptyIterator) IsValid() bool  { return false }
func (vec3EmptyIterator) Vec3() mat.Vec3 { panic("invalid access to empty iterator") }
