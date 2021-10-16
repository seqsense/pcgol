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

type PointToPointICPGradient struct {
	Evaluator      Evaluator
	UpdaterFactory UpdaterGradientFactory
}

func (r *PointToPointICPGradient) Fit(base storage.Search, target pc.Vec3RandomAccessor) (mat.Mat4, Stat, error) {
	if !r.Evaluator.HasGradient() {
		return mat.Mat4{}, Stat{}, ErrNeedGradient
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

	updaterFactory := r.UpdaterFactory
	if updaterFactory == nil {
		updaterFactory = &GradientDescentUpdaterFactory{}
	}
	updater := updaterFactory.New()

	var stat Stat
	trans := mat.Translate(0, 0, 0)
	for {
		ev, err := r.Evaluator.Evaluate(base, targetTransformed)
		stat.NumIteration++
		if err != nil {
			return trans, stat, err
		}
		stat.Evaluated = *ev

		var converged bool
		trans, converged = updater.Update(trans, ev)
		if converged {
			break
		}

		for i := 0; i < target.Len(); i++ {
			targetTransformed[i] = trans.Transform(target.Vec3At(i))
		}
	}
	return trans, stat, nil
}
