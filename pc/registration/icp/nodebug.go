//go:build !debugPlot
// +build !debugPlot

package icp

import (
	"github.com/seqsense/pcgol/internal/gnuplot"
)

const debugPlot = false
const debugPlotInterval = 0

var g *gnuplot.Gnuplot
