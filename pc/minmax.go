package pc

import (
	"errors"
	"math"

	"github.com/seqsense/pcgol/mat"
)

func MinMaxVec3(pp *PointCloud) (mat.Vec3, mat.Vec3, error) {
	it, err := pp.Vec3Iterator()
	if err != nil {
		return mat.Vec3{}, mat.Vec3{}, err
	}
	if !it.IsValid() {
		return mat.Vec3{}, mat.Vec3{}, errors.New("no point")
	}
	min := mat.Vec3{math.MaxFloat32, math.MaxFloat32, math.MaxFloat32}
	max := mat.Vec3{-math.MaxFloat32, -math.MaxFloat32, -math.MaxFloat32}
	for ; it.IsValid(); it.Incr() {
		v := it.Vec3()
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

func MinMaxVec3RandomAccessor(ra Vec3RandomAccessor) (mat.Vec3, mat.Vec3, error) {
	if ra.Len() == 0 {
		return mat.Vec3{}, mat.Vec3{}, errors.New("no point")
	}
	min := ra.Vec3At(0)
	max := ra.Vec3At(0)
	for i := 0; i < ra.Len(); i++ {
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
