package gnuplot

import (
	"io"

	"github.com/seqsense/pcgol/pc"
)

type Gnuplot interface {
	Write(s string)
	Splot(pp ...Plot)
	Close()
}

type Plot interface {
	Args() string
	WriteData(w io.Writer)
}

type PointsPlot struct {
	Options string
	Dims    []int
	Points  pc.Vec3RandomAccessor
}

func New() (Gnuplot, error) {
	return NewWithCommand("gnuplot", "-p")
}

func Must(g Gnuplot, err error) Gnuplot {
	if err != nil {
		panic(err)
	}
	return g
}
