package testutil

import (
	"github.com/seqsense/pcgol/mat"
)

type Vec3Array []mat.Vec3

func (v Vec3Array) Len() int {
	return len(v)
}

func (v Vec3Array) Vec3At(i int) mat.Vec3 {
	return v[i]
}
