//go:build debugPlot
// +build debugPlot

package sac

import (
	"github.com/seqsense/pcgol/internal/gnuplot"
	"time"
)

const debugPlot = true
const debugPlotWait = time.Minute

var g gnuplot.Gnuplot

func init() {
	g = gnuplot.Must(gnuplot.New())
}
