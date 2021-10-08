package icp

import (
	"errors"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
)

var (
	ErrNeedGradient = errors.New("need gradient output of Evaluator")
)

type PointToPointICPGradient struct {
	Evaluator    Evaluator
	MaxIteration int
}

func (r *PointToPointICPGradient) Fit(target pc.Vec3RandomAccessor) (mat.Mat4, error) {
	if !r.Evaluator.HasGradient() {
		return mat.Mat4{}, ErrNeedGradient
	}
	targetTransformed := make(pc.Vec3Slice, target.Len())
	for i := 0; i < target.Len(); i++ {
		targetTransformed[i] = target.Vec3At(i)
	}
	maxIteration := r.MaxIteration
	if maxIteration == 0 {
		maxIteration = 20
	}
	trans := mat.Translate(0, 0, 0)
	tTrans := mat.Translate(0, 0, 0)
	tRot := mat.Translate(0, 0, 0)
	for i := 0; i < maxIteration; i++ {
		ev, err := r.Evaluator.Evaluate(targetTransformed)
		if err != nil {
			return trans, err
		}
		factor := -0.3 * (1 - (float32(i) / float32(maxIteration)))
		deltaTrans := mat.Translate(
			factor*ev.Gradient[dX],
			factor*ev.Gradient[dY],
			factor*ev.Gradient[dZ],
		)
		deltaRot := rodriguesToRotation(mat.Vec3{
			factor * ev.Gradient[dWx],
			factor * ev.Gradient[dWy],
			factor * ev.Gradient[dWz],
		})
		tRot = deltaRot.Mul(tRot)
		tTrans = deltaTrans.Mul(tTrans)
		trans = tRot.Mul(tTrans)
		for i := 0; i < target.Len(); i++ {
			targetTransformed[i] = trans.Transform(target.Vec3At(i))
		}
	}
	return trans, nil
}
