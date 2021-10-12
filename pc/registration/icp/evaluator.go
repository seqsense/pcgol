package icp

import (
	"errors"
	"math"
	"time"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/storage"

	"github.com/seqsense/pcgol/internal/gnuplot"
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
	Evaluate(base storage.Search, target pc.Vec3RandomAccessor) (*Evaluated, error)
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
//   (2*z0*x1 - 2*z1*x0) / (2*z0*z0 + 2*x0*x0 - 1)
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

func isNaN(v float32) bool {
	return math.IsNaN(float64(v))
}

func (e *PointToPointEvaluator) Evaluate(base storage.Search, target pc.Vec3RandomAccessor) (*Evaluated, error) {
	minPairs := e.MinPairs
	if minPairs == 0 {
		minPairs = 6
	}
	pairs := e.Corresponder.Pairs(base, target)
	if len(pairs) < minPairs {
		return nil, ErrNotEnoughPairs
	}
	out := &Evaluated{}
	var num int

	pairVecs := [2]pc.Vec3RandomAccessor{
		make(pc.Vec3Slice, len(pairs)),
		make(pc.Vec3Slice, len(pairs)),
	}
	for i, pair := range pairs {
		pb := base.Vec3At(pair.BaseID)
		pt := target.Vec3At(pair.TargetID)
		pairVecs[0].(pc.Vec3Slice)[i] = pb
		pairVecs[1].(pc.Vec3Slice)[i] = pt

		x0, y0, z0 := pt[0], pt[1], pt[2]
		x1, y1, z1 := pb[0], pb[1], pb[2]
		out.Value += pair.SquaredDistance

		out.Gradient[0] += 2 * (x0 - x1)
		out.Gradient[1] += 2 * (y0 - y1)
		out.Gradient[2] += 2 * (z0 - z1)
		xd := 2 * (y0*z1 - y1*z0) / (z0*z0 + y0*y0 - 0.5)
		yd := 2 * (z0*x1 - z1*x0) / (z0*z0 + x0*x0 - 0.5)
		zd := 2 * (x0*y1 - x1*y0) / (y0*y0 + x0*x0 - 0.5)
		if !isNaN(xd) && !isNaN(yd) && !isNaN(zd) {
			out.Gradient[3] += xd
			out.Gradient[4] += yd
			out.Gradient[5] += zd
			num++
		}
	}

	if debugPlot {
		g.Splot(
			&gnuplot.PointsPlot{Points: base},
			&gnuplot.PointsPlot{Points: target},
			&gnuplot.PointPairsPlot{Points: pairVecs},
		)
		time.Sleep(debugPlotInterval)
	}

	f := 1 / float32(len(pairs))
	out.Value *= f
	for i := 0; i < 3; i++ {
		out.Gradient[i] *= f
	}
	var fn float32
	if num != 0 {
		fn = 1 / float32(num)
	}
	for i := 3; i < 6; i++ {
		out.Gradient[i] *= fn
	}
	return out, nil
}
