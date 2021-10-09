package icp

import (
	"errors"
	"time"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/storage"

	"github.com/seqsense/pcgol/internal/gnuplot"
)

var (
	ErrNeedGradient = errors.New("need gradient output of Evaluator")
)

var DefaultGradientWeight = mat.Vec6{0.3, 0.3, 0.3, 0.3, 0.3, 0.3}

type PointToPointICPGradient struct {
	Evaluator      Evaluator
	MaxIteration   int
	GradientWeight mat.Vec6
}

func (r *PointToPointICPGradient) Fit(base storage.Search, target pc.Vec3RandomAccessor) (mat.Mat4, error) {
	if !r.Evaluator.HasGradient() {
		return mat.Mat4{}, ErrNeedGradient
	}
	targetTransformed := make(pc.Vec3Slice, target.Len())
	for i := 0; i < target.Len(); i++ {
		targetTransformed[i] = target.Vec3At(i)
	}

	if debugPlot {
		g.Splot(
			&gnuplot.PointsPlot{Points: base},
			&gnuplot.PointsPlot{Points: target},
		)
		time.Sleep(debugPlotInterval)
	}

	maxIteration := r.MaxIteration
	if maxIteration == 0 {
		maxIteration = 20
	}
	weight := r.GradientWeight
	if weight.Equal(mat.Vec6{}) {
		weight = DefaultGradientWeight
	}

	trans := mat.Translate(0, 0, 0)
	for i := 0; i < maxIteration; i++ {
		ev, err := r.Evaluator.Evaluate(base, targetTransformed)
		if err != nil {
			return trans, err
		}

		factorIter := -(1 - (float32(i) / float32(maxIteration)))
		deltaTrans := mat.Translate(
			factorIter*weight[0]*ev.Gradient[dX],
			factorIter*weight[1]*ev.Gradient[dY],
			factorIter*weight[2]*ev.Gradient[dZ],
		)
		deltaRot := rodriguesToRotation(mat.Vec3{
			factorIter * weight[3] * ev.Gradient[dWx],
			factorIter * weight[4] * ev.Gradient[dWy],
			factorIter * weight[5] * ev.Gradient[dWz],
		})
		trans = deltaRot.Mul(deltaTrans.Mul(trans))
		for i := 0; i < target.Len(); i++ {
			targetTransformed[i] = trans.Transform(target.Vec3At(i))
		}
	}
	return trans, nil
}
