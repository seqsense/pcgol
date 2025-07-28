package pc

import (
	"bufio"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"math"
	"strconv"
	"strings"

	lzf "github.com/zhuyie/golzf"
)

type Format int

const (
	Ascii Format = iota
	Binary
	BinaryCompressed
)

func UnmarshalHeader(r io.Reader) (*PointCloudHeader, error) {
	rb := bufio.NewReader(r)
	ph := &PointCloudHeader{}
	if _, _, err := unmarshalHeaderTo(rb, ph); err != nil {
		return nil, err
	}
	return ph, nil
}

func Unmarshal(r io.Reader) (*PointCloud, error) {
	rb := bufio.NewReader(r)
	pp := &PointCloud{}
	nPoints, ppFmt, err := unmarshalHeaderTo(rb, &pp.PointCloudHeader)
	if err != nil {
		return nil, err
	}
	pp.Points = nPoints
	if err := unmarshalTo(rb, pp, ppFmt); err != nil {
		return nil, err
	}
	return pp, nil
}

func unmarshalHeaderTo(rb *bufio.Reader, pp *PointCloudHeader) (int, Format, error) {
	var ppFmt Format
	var nPoints int
L_HEADER:
	for {
		line, _, err := rb.ReadLine()
		if err != nil {
			return 0, 0, err
		}
		args := strings.Fields(string(line))
		if len(args) < 2 {
			return 0, 0, errors.New("header field must have value")
		}
		switch args[0] {
		case "VERSION":
			f, err := strconv.ParseFloat(args[1], 32)
			if err != nil {
				return 0, 0, err
			}
			pp.Version = float32(f)
		case "FIELDS":
			pp.Fields = args[1:]
		case "SIZE":
			pp.Size = make([]int, len(args)-1)
			for i, s := range args[1:] {
				pp.Size[i], err = strconv.Atoi(s)
				if err != nil {
					return 0, 0, err
				}
			}
		case "TYPE":
			pp.Type = args[1:]
		case "COUNT":
			pp.Count = make([]int, len(args)-1)
			for i, s := range args[1:] {
				pp.Count[i], err = strconv.Atoi(s)
				if err != nil {
					return 0, 0, err
				}
			}
		case "WIDTH":
			pp.Width, err = strconv.Atoi(args[1])
			if err != nil {
				return 0, 0, err
			}
		case "HEIGHT":
			pp.Height, err = strconv.Atoi(args[1])
			if err != nil {
				return 0, 0, err
			}
		case "VIEWPOINT":
			pp.Viewpoint = make([]float32, len(args)-1)
			for i, s := range args[1:] {
				f, err := strconv.ParseFloat(s, 32)
				if err != nil {
					return 0, 0, err
				}
				pp.Viewpoint[i] = float32(f)
			}
		case "POINTS":
			nPoints, err = strconv.Atoi(args[1])
			if err != nil {
				return 0, 0, err
			}
		case "DATA":
			switch args[1] {
			case "ascii":
				ppFmt = Ascii
			case "binary":
				ppFmt = Binary
			case "binary_compressed":
				ppFmt = BinaryCompressed
			default:
				return 0, 0, errors.New("unknown data format")
			}
			break L_HEADER
		}
	}
	// validate
	if len(pp.Fields) != len(pp.Size) {
		return 0, 0, errors.New("size field size is wrong")
	}
	if len(pp.Fields) != len(pp.Type) {
		return 0, 0, errors.New("type field size is wrong")
	}
	if len(pp.Fields) != len(pp.Count) {
		return 0, 0, errors.New("count field size is wrong")
	}
	return nPoints, ppFmt, nil
}

func unmarshalTo(rb *bufio.Reader, pp *PointCloud, ppFmt Format) error {
	switch ppFmt {
	case Ascii:
		pp.Data = make([]byte, pp.Points*pp.Stride())
		dataOffset := 0
		for {
			line, _, err := rb.ReadLine()
			if err != nil && err != io.EOF {
				return err
			}
			if err == io.EOF {
				break
			}
			pointData := strings.Fields(string(line))
			lineOffset := 0
			for i, f := range pp.Type {
				for j := 0; j < pp.Count[i]; j++ {
					switch f {
					case "F":
						v, err := strconv.ParseFloat(pointData[lineOffset+j], 32)
						if err != nil {
							return err
						}
						b := math.Float32bits(float32(v))
						binary.LittleEndian.PutUint32(
							pp.Data[dataOffset:dataOffset+4], b,
						)
					case "U":
						v, err := strconv.ParseUint(pointData[lineOffset+j], 10, 32)
						if err != nil {
							return err
						}
						binary.LittleEndian.PutUint32(
							pp.Data[dataOffset:dataOffset+4], uint32(v),
						)
					}
					dataOffset += pp.Size[i]
				}
				lineOffset += pp.Count[i]
			}
		}
	case Binary:
		b := make([]byte, pp.Points*pp.Stride())
		if _, err := io.ReadFull(rb, b); err != nil {
			return err
		}
		pp.Data = b
	case BinaryCompressed:
		var nCompressed, nUncompressed int32
		if err := binary.Read(rb, binary.LittleEndian, &nCompressed); err != nil {
			return err
		}
		if err := binary.Read(rb, binary.LittleEndian, &nUncompressed); err != nil {
			return err
		}

		b := make([]byte, nCompressed)
		if _, err := io.ReadFull(rb, b); err != nil {
			return err
		}

		dec := make([]byte, nUncompressed)
		n, err := lzf.Decompress(b[:nCompressed], dec)
		if err != nil {
			return err
		}
		if int(nUncompressed) != n {
			return errors.New("wrong uncompressed size")
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
	return nil
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

	// Set a default value to Viewpoint if it was not provided.
	// Viewpoint is optional for all computation so it might not be always filled in
	// when manually creating PointCloud object but it is required by pcl_viewer and
	// pcdeditor to successfully load a pcd file.
	if len(pp.Viewpoint) == 0 {
		pp.Viewpoint = []float32{0, 0, 0, 1, 0, 0, 0}
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
