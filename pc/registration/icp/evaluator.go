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

type EvaluateWeightFn func(distSq float32) float32

var DefaultEvaluateWeightFn = func(_ float32) float32 {
	return 1
}

type Evaluated struct {
	Value    float32
	Gradient mat.Vec6
	Hessian  mat.Mat6
	DistRMS  float32
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
//   2*y0*(z0-z1) - 2*z0*(y0-y1)
//   2*z0*(x0-x1) - 2*x0*(z0-z1)
//   2*x0*(y0-y1) - 2*y0*(x0-x1)
// }
type PointToPointEvaluator struct {
	Corresponder PointToPointCorresponder
	MinPairs     int
	WeightFn     EvaluateWeightFn
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
		if debugPlot {
			g.Splot(
				&gnuplot.PointsPlot{Points: base},
				&gnuplot.PointsPlot{Points: target},
			)
			time.Sleep(debugPlotInterval)
		}
		return nil, ErrNotEnoughPairs
	}
	out := &Evaluated{}

	var sumWeight float32
	weightFn := e.WeightFn
	if weightFn == nil {
		weightFn = DefaultEvaluateWeightFn
	}

	pairVecs := [2]pc.Vec3RandomAccessor{
		make(pc.Vec3Slice, len(pairs)),
		make(pc.Vec3Slice, len(pairs)),
	}
	for i, pair := range pairs {
		pb := base.Vec3At(pair.BaseID)
		pt := target.Vec3At(pair.TargetID)
		pairVecs[0].(pc.Vec3Slice)[i] = pb
		pairVecs[1].(pc.Vec3Slice)[i] = pt

		w := weightFn(pair.SquaredDistance)

		out.Value += w * pair.SquaredDistance
		sumWeight += w

		x0, y0, z0 := pt[0], pt[1], pt[2]
		x1, y1, z1 := pb[0], pb[1], pb[2]
		out.Gradient[0] += w * 2 * (x0 - x1)
		out.Gradient[1] += w * 2 * (y0 - y1)
		out.Gradient[2] += w * 2 * (z0 - z1)
		out.Gradient[3] += w * 2 * (z0*(y0-y1) - y0*(z0-z1))
		out.Gradient[4] += w * 2 * (x0*(z0-z1) - z0*(x0-x1))
		out.Gradient[5] += w * 2 * (y0*(x0-x1) - x0*(y0-y1))

		out.DistRMS += w * pt.NormSq()
	}

	if debugPlot {
		g.Splot(
			&gnuplot.PointsPlot{Points: base},
			&gnuplot.PointsPlot{Points: target},
			&gnuplot.PointPairsPlot{Points: pairVecs},
		)
		time.Sleep(debugPlotInterval)
	}

	f := float32(1)
	if sumWeight > 1 {
		f = 1 / sumWeight
	}
	out.Value *= f
	for i := 0; i < 6; i++ {
		out.Gradient[i] *= f
	}
	out.DistRMS = float32(math.Sqrt(float64(out.DistRMS * f)))

	// As rotation formula is approxymated near zero,
	// non-linear large rotation gradient may be calculated
	// when the clouds are not yet aligned well.
	// Limit translation caused by rotation.
	rotLimit := float32(1)
	dist := float32(math.Sqrt(float64(out.Value)))
	for i := 3; i < 6; i++ {
		d := out.Gradient[i] * out.DistRMS
		if d < 0 {
			d = -d
		}
		if dist < d {
			l := dist / d
			if rotLimit > l {
				rotLimit = l
			}
		}
	}
	for i := 3; i < 6; i++ {
		out.Gradient[i] *= rotLimit
	}

	return out, nil
}
