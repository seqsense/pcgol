package gnuplot

import (
	"fmt"
	"io"
	"strconv"
	"strings"

	"github.com/seqsense/pcgol/pc"
)

func (p *PointsPlot) Args() string {
	if len(p.Dims) == 0 {
		p.Dims = []int{1, 2, 3}
	}
	if p.Options == "" {
		p.Options = "notitle"
	}
	dims := make([]string, len(p.Dims))
	for i, d := range p.Dims {
		dims[i] = strconv.Itoa(d)
	}
	return fmt.Sprintf(
		`"-" u %s %s nohidden3d`,
		strings.Join(dims, ":"), p.Options,
	)
}

func (p *PointsPlot) WriteData(w io.Writer) {
	for i := 0; i < p.Points.Len(); i++ {
		p := p.Points.Vec3At(i)
		w.Write([]byte(fmt.Sprintf("%f %f %f\n", p[0], p[1], p[2])))
	}
}

type PointPairsPlot struct {
	Options string
	Dims    []int
	Points  [2]pc.Vec3RandomAccessor
}

func (p *PointPairsPlot) Args() string {
	if len(p.Dims) == 0 {
		p.Dims = []int{1, 2, 3}
	}
	if p.Options == "" {
		p.Options = "w l notitle"
	}
	dims := make([]string, len(p.Dims))
	for i, d := range p.Dims {
		dims[i] = strconv.Itoa(d)
	}
	return fmt.Sprintf(
		`"-" u %s %s`,
		strings.Join(dims, ":"), p.Options,
	)
}

func (p *PointPairsPlot) WriteData(w io.Writer) {
	for i := 0; i < p.Points[0].Len() && i < p.Points[1].Len(); i++ {
		p0 := p.Points[0].Vec3At(i)
		p1 := p.Points[1].Vec3At(i)
		w.Write([]byte(fmt.Sprintf(
			"%f %f %f\n%f %f %f\n\n\n",
			p0[0], p0[1], p0[2],
			p1[0], p1[1], p1[2],
		)))
	}
}
