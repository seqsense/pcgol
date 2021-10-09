package gnuplot

type gnuplot struct{}

func NewWithCommand(args ...string) (Gnuplot, error) {
	return &gnuplot{}, nil
}

func (gnuplot) Write(s string)   {}
func (gnuplot) Splot(pp ...Plot) {}
func (gnuplot) Close()           {}
