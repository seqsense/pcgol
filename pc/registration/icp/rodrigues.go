package icp

import (
	"math"

	"github.com/seqsense/pcgol/mat"
)

func rodriguesToRotation(v mat.Vec3) mat.Mat4 {
	ang := v.Norm()
	r := mat.Mat4{
		0, -v[2], v[1], 0,
		v[2], 0, -v[0], 0,
		-v[1], v[0], 0, 0,
		0, 0, 0, 0,
	}
	i := mat.Mat4{
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	}
	var f0, f1 float32
	if ang < 0.1 {
		f0, f1 = 1, 0.5
	} else {
		f0 = float32(math.Sin(float64(ang))) / ang
		f1 = float32(1-math.Cos(float64(ang))) / (ang * ang)
	}
	return i.Add(r.Factor(f0)).Add(r.Mul(r).Factor(f1))
}
