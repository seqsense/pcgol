package icp

import (
	"errors"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
)

var (
	ErrNotEnoughPairs = errors.New("not enough correspondence pairs")
)

type Evaluated struct {
	Value    float32
	Gradient mat.Vec6
	Hessian  mat.Mat6
}

type Evaluator interface {
	Evaluate(target pc.Vec3RandomAccessor) (*Evaluated, error)
	HasGradient() bool
	HasHessian() bool
}

// PointToPointEvaluator implements Evaluator based on PointToPointCorresponder.
//
// pb: base point {
//   x1
//   y1
//   z1
// }
// pt: target point {
//   x0
//   y0
//   z0
// }
// dR: {
//      1 -dWz  dWy
//    dWz    1 -dWx
//   -dWy  dWx    1
// } // Primary term of Rodrigues' rotation around zero
// dT: {
//   dx
//   dy
//   dz
// }
// f(pt) = ((dR*pt + dT) - pb).NormSq()
// grad f(pt) = {
//   2*(x0 - x1)
//   2*(y0 - y1)
//   2*(z0 - z1)
//   (2*y0*z1 - 2*y1*z0) / (2*z0*z0 + 2*y0*y0 - 1)
//   -(2*x0*z1 - 2*x1*z0) / (2*z0*z0 + 2*x0*x0 - 1)
//   (2*x0*y1 - 2*x1*y0) / (2*y0*y0 + 2*x0*x0 - 1)
// }
type PointToPointEvaluator struct {
	Corresponder PointToPointCorresponder
	MinPairs     int
}

func (PointToPointEvaluator) HasGradient() bool { return true }
func (PointToPointEvaluator) HasHessian() bool  { return false }

const (
	dX = iota
	dY
	dZ
	dWx
	dWy
	dWz
)

func (e *PointToPointEvaluator) Evaluate(target pc.Vec3RandomAccessor) (*Evaluated, error) {
	minPairs := e.MinPairs
	if minPairs == 0 {
		minPairs = 6
	}
	pairs := e.Corresponder.Pairs(target)
	if len(pairs) < minPairs {
		return nil, ErrNotEnoughPairs
	}
	out := &Evaluated{}
	for _, pair := range pairs {
		pb := e.Corresponder.Vec3At(pair.BaseID)
		pt := target.Vec3At(pair.TargetID)
		x0, y0, z0 := pt[0], pt[1], pt[2]
		x1, y1, z1 := pb[0], pb[1], pb[2]
		out.Value += pair.SquaredDistance
		out.Gradient[0] += 2 * (x0 - x1)
		out.Gradient[1] += 2 * (y0 - y1)
		out.Gradient[2] += 2 * (z0 - z1)
		out.Gradient[3] += (2*y0*z1 - 2*y1*z0) / (2*z0*z0 + 2*y0*y0 - 1)
		out.Gradient[4] += -(2*x0*z1 - 2*x1*z0) / (2*z0*z0 + 2*x0*x0 - 1)
		out.Gradient[5] += (2*x0*y1 - 2*x1*y0) / (2*y0*y0 + 2*x0*x0 - 1)
	}
	out.Value /= float32(len(pairs))
	return out, nil
}
