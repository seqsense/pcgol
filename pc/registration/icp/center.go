package icp

import (
	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
)

func center(p pc.Vec3RandomAccessor) mat.Vec3 {
	var out mat.Vec3
	for i := 0; i < p.Len(); i++ {
		out = out.Add(p.Vec3At(i))
	}
	return out.Mul(1 / float32(p.Len()))
}
