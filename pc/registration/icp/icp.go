package icp

import (
	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
)

type PointToPointICP struct {
	Evaluator    *PointToPointEvaluator
	MaxIteration int
}

func (r *PointToPointICP) Fit(target pc.Vec3RandomAccessor) (mat.Mat4, error) {
	targetTransformed := make(pc.Vec3Slice, target.Len())
	for i := 0; i < target.Len(); i++ {
		targetTransformed[i] = target.Vec3At(i)
	}
	maxIteration := r.MaxIteration
	if maxIteration == 0 {
		maxIteration = 10
	}
	trans := mat.Translate(0, 0, 0)
	for i := 0; i < maxIteration; i++ {
		ev, err := r.Evaluator.Evaluate(targetTransformed)
		if err != nil {
			return trans, err
		}
		factor := -(1 - (float32(maxIteration) / float32(i)))
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
		trans = trans.Mul(deltaRot).Mul(deltaTrans)
		for i := 0; i < target.Len(); i++ {
			targetTransformed[i] = trans.Transform(target.Vec3At(i))
		}
	}
	return trans, nil
}
