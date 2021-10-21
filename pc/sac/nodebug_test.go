//go:build !debugPlot
// +build !debugPlot

package sac

import (
	"github.com/seqsense/pcgol/internal/gnuplot"
)

const debugPlot = false
const debugPlotWait = 0

var g gnuplot.Gnuplot
