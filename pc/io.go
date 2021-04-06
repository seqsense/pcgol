package pc

import (
	"bufio"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"strconv"
	"strings"

	"github.com/zhuyie/golzf"
)

type Format int

const (
	Ascii Format = iota
	Binary
	BinaryCompressed
)

func Unmarshal(r io.Reader) (*PointCloud, error) {
	rb := bufio.NewReader(r)
	pp := &PointCloud{}
	var fmt Format

L_HEADER:
	for {
		line, _, err := rb.ReadLine()
		if err != nil {
			return nil, err
		}
		args := strings.Fields(string(line))
		if len(args) < 2 {
			return nil, errors.New("header field must have value")
		}
		switch args[0] {
		case "VERSION":
			f, err := strconv.ParseFloat(args[1], 32)
			if err != nil {
				return nil, err
			}
			pp.Version = float32(f)
		case "FIELDS":
			pp.Fields = args[1:]
		case "SIZE":
			pp.Size = make([]int, len(args)-1)
			for i, s := range args[1:] {
				pp.Size[i], err = strconv.Atoi(s)
				if err != nil {
					return nil, err
				}
			}
		case "TYPE":
			pp.Type = args[1:]
		case "COUNT":
			pp.Count = make([]int, len(args)-1)
			for i, s := range args[1:] {
				pp.Count[i], err = strconv.Atoi(s)
				if err != nil {
					return nil, err
				}
			}
		case "WIDTH":
			pp.Width, err = strconv.Atoi(args[1])
			if err != nil {
				return nil, err
			}
		case "HEIGHT":
			pp.Height, err = strconv.Atoi(args[1])
			if err != nil {
				return nil, err
			}
		case "VIEWPOINT":
			pp.Viewpoint = make([]float32, len(args)-1)
			for i, s := range args[1:] {
				f, err := strconv.ParseFloat(s, 32)
				if err != nil {
					return nil, err
				}
				pp.Viewpoint[i] = float32(f)
			}
		case "POINTS":
			pp.Points, err = strconv.Atoi(args[1])
			if err != nil {
				return nil, err
			}
		case "DATA":
			switch args[1] {
			case "ascii":
				fmt = Ascii
			case "binary":
				fmt = Binary
			case "binary_compressed":
				fmt = BinaryCompressed
			default:
				return nil, errors.New("unknown data format")
			}
			break L_HEADER
		}
	}
	// validate
	if len(pp.Fields) != len(pp.Size) {
		return nil, errors.New("size field size is wrong")
	}
	if len(pp.Fields) != len(pp.Type) {
		return nil, errors.New("type field size is wrong")
	}
	if len(pp.Fields) != len(pp.Count) {
		return nil, errors.New("count field size is wrong")
	}

	switch fmt {
	case Ascii:
		panic("not implemented yet")
	case Binary:
		b := make([]byte, pp.Points*pp.Stride())
		if _, err := io.ReadFull(rb, b); err != nil {
			return nil, err
		}
		pp.Data = b
	case BinaryCompressed:
		var nCompressed, nUncompressed int32
		if err := binary.Read(rb, binary.LittleEndian, &nCompressed); err != nil {
			return nil, err
		}
		if err := binary.Read(rb, binary.LittleEndian, &nUncompressed); err != nil {
			return nil, err
		}

		b := make([]byte, nCompressed)
		if _, err := io.ReadFull(rb, b); err != nil {
			return nil, err
		}

		dec := make([]byte, nUncompressed)
		n, err := lzf.Decompress(b[:nCompressed], dec)
		if err != nil {
			return nil, err
		}
		if int(nUncompressed) != n {
			return nil, errors.New("wrong uncompressed size")
		}

		head := make([]int, len(pp.Fields))
		offset := make([]int, len(pp.Fields))
		var pos, off int
		for i := range pp.Fields {
			head[i] = pos
			offset[i] = off
			pos += pp.Size[i] * pp.Count[i] * pp.Points
			off += pp.Size[i] * pp.Count[i]
		}

		stride := pp.Stride()
		pp.Data = make([]byte, n)
		for p := 0; p < pp.Points; p++ {
			for i := range head {
				size := pp.Size[i]
				to := p*stride + offset[i]
				from := head[i] + p*size
				copy(pp.Data[to:to+size], dec[from:from+size])
			}
		}
	}

	return pp, nil
}

func Marshal(pp *PointCloud, w io.Writer) error {
	intToStringSlice := func(d []int) []string {
		var ret []string
		for _, v := range d {
			ret = append(ret, strconv.Itoa(v))
		}
		return ret
	}
	floatToStringSlice := func(d []float32) []string {
		var ret []string
		for _, v := range d {
			ret = append(ret, strconv.FormatFloat(float64(v), 'f', 4, 32))
		}
		return ret
	}
	header := fmt.Sprintf(
		`VERSION %0.1f
FIELDS %s
SIZE %s
TYPE %s
COUNT %s
WIDTH %d
HEIGHT %d
VIEWPOINT %s
POINTS %d
DATA binary
`,
		pp.Version,
		strings.Join(pp.Fields, " "),
		strings.Join(intToStringSlice(pp.Size), " "),
		strings.Join(pp.Type, " "),
		strings.Join(intToStringSlice(pp.Count), " "),
		pp.Width,
		pp.Height,
		strings.Join(floatToStringSlice(pp.Viewpoint), " "),
		pp.Points,
	)
	if _, err := w.Write([]byte(header)); err != nil {
		return err
	}
	if _, err := w.Write(pp.Data); err != nil {
		return err
	}
	return nil
}
