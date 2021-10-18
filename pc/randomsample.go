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
	case ratio < 0:
		ratio = 0
	case ratio > 1:
		ratio = 1
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
