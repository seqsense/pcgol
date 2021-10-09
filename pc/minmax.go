package pc

import (
	"errors"

	"github.com/seqsense/pcgol/mat"
)

func MinMaxVec3(ra Vec3RandomAccessor) (mat.Vec3, mat.Vec3, error) {
	if ra.Len() == 0 {
		return mat.Vec3{}, mat.Vec3{}, errors.New("no point")
	}
	min, max := ra.Vec3At(0), ra.Vec3At(0)
	for i := 1; i < ra.Len(); i++ {
		v := ra.Vec3At(i)
		for i := range v {
			if v[i] < min[i] {
				min[i] = v[i]
			}
			if v[i] > max[i] {
				max[i] = v[i]
			}
		}
	}
	return min, max, nil
}
