//go:build debugPlot
// +build debugPlot

package icp

import (
	"github.com/seqsense/pcgol/internal/gnuplot"
	"time"
)

const debugPlot = true
const debugPlotInterval = 500 * time.Millisecond

var g gnuplot.Gnuplot

func init() {
	g = gnuplot.Must(gnuplot.New())
	g.Write("set xrange [-5:5]")
	g.Write("set yrange [-5:5]")
	g.Write("set zrange [-5:5]")
}
