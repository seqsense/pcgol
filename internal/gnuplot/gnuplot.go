//go:build !js
// +build !js

package gnuplot

import (
	"io"
	"os"
	"os/exec"
	"strings"
	"sync"
)

type gnuplot struct {
	cmd *exec.Cmd
	w   io.WriteCloser
	wg  sync.WaitGroup
}

func NewWithCommand(args ...string) (Gnuplot, error) {
	cmd := exec.Command(args[0], args[1:]...)
	w, err := cmd.StdinPipe()
	if err != nil {
		return nil, err
	}

	cout, err := cmd.StdoutPipe()
	if err != nil {
		return nil, err
	}
	cerr, err := cmd.StderrPipe()
	if err != nil {
		return nil, err
	}
	println("init")

	g := &gnuplot{
		cmd: cmd,
		w:   w,
	}
	g.wg.Add(2)
	go func() {
		println("output stdout")
		io.Copy(os.Stdout, cout)
		println("/output stdout")
		g.wg.Done()
	}()
	go func() {
		println("output stderr")
		io.Copy(os.Stderr, cerr)
		println("/output stderr")
		g.wg.Done()
	}()

	println("starting")
	if err := cmd.Start(); err != nil {
		return nil, err
	}
	g.Write("set grid")
	g.Write("set size ratio -1")
	g.Write("set view equal xyz")
	g.Write("set ticslevel 0")
	return g, nil
}

func (g *gnuplot) Write(s string) {
	println("writing")
	g.w.Write([]byte(s + "\n"))
}

func (g *gnuplot) Splot(pp ...Plot) {
	args := make([]string, len(pp))
	for i, p := range pp {
		args[i] = p.Args()
	}
	g.Write("splot" + strings.Join(args, ","))
	for _, p := range pp {
		p.WriteData(g.w)
		g.Write("e")
	}
}

func (g *gnuplot) Close() {
	println("closing")
	g.w.Close()
	g.cmd.Wait()
	g.wg.Wait()
}
