package icp

import (
	"github.com/seqsense/pcgol/mat"
)

type UpdaterGradientFactory interface {
	New() UpdaterGradient
}

type UpdaterGradient interface {
	Update(trans mat.Mat4, ev *Evaluated) (mat.Mat4, bool)
}

var DefaultGradientWeight = mat.Vec6{0.3, 0.3, 0.3, 0.3, 0.3, 0.3}
var DefaultGradientThreshold = mat.Vec6{0.01, 0.01, 0.01, 0.01, 0.01, 0.01}

type GradientDescentUpdaterFactory struct {
	Weight       mat.Vec6
	Threshold    mat.Vec6
	MaxIteration int
}

func (f GradientDescentUpdaterFactory) New() UpdaterGradient {
	if f.Weight.Equal(mat.Vec6{}) {
		f.Weight = DefaultGradientWeight
	}
	if f.Threshold.Equal(mat.Vec6{}) {
		f.Threshold = DefaultGradientThreshold
	}
	if f.MaxIteration == 0 {
		f.MaxIteration = 20
	}
	return &gradientDescentUpdater{
		f: f,
	}
}

type gradientDescentUpdater struct {
	f GradientDescentUpdaterFactory
	i int
}

func (u *gradientDescentUpdater) Update(trans mat.Mat4, ev *Evaluated) (mat.Mat4, bool) {
	flat := true
	for j, g := range ev.Gradient {
		if g < -u.f.Threshold[j] || u.f.Threshold[j] < g {
			flat = false
			break
		}
	}
	if flat {
		return trans, true
	}

	delta := mat.Vec6{
		u.f.Weight[0] * ev.Gradient[dX],
		u.f.Weight[1] * ev.Gradient[dY],
		u.f.Weight[2] * ev.Gradient[dZ],
		u.f.Weight[3] * ev.Gradient[dWx],
		u.f.Weight[4] * ev.Gradient[dWy],
		u.f.Weight[5] * ev.Gradient[dWz],
	}
	factorIter := -(1 - (float32(u.i) / float32(u.f.MaxIteration)))
	deltaTrans := mat.Translate(
		factorIter*delta[0],
		factorIter*delta[1],
		factorIter*delta[2],
	)
	deltaRot := rodriguesToRotation(mat.Vec3{
		factorIter * delta[3],
		factorIter * delta[4],
		factorIter * delta[5],
	})

	trans = deltaTrans.Mul(deltaRot.Mul(trans))
	u.i++
	return trans, u.i >= u.f.MaxIteration
}
