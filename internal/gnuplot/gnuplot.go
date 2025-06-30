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

	g := &gnuplot{
		cmd: cmd,
		w:   w,
	}
	g.wg.Add(2)
	go func() {
		io.Copy(os.Stdout, cout)
		g.wg.Done()
	}()
	go func() {
		io.Copy(os.Stderr, cerr)
		g.wg.Done()
	}()

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
	g.w.Close()
	g.wg.Wait()
	g.cmd.Wait()
}
