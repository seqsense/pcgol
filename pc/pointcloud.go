package pc

import (
	"errors"

	"github.com/seqsense/pcgol/pc/internal/float"
)

type PointCloudHeader struct {
	Version   float32
	Fields    []string
	Size      []int
	Type      []string
	Count     []int
	Width     int
	Height    int
	Viewpoint []float32
}

func (h *PointCloudHeader) Clone() PointCloudHeader {
	return PointCloudHeader{
		Version:   h.Version,
		Fields:    append([]string{}, h.Fields...),
		Size:      append([]int{}, h.Size...),
		Type:      append([]string{}, h.Type...),
		Count:     append([]int{}, h.Count...),
		Width:     h.Width,
		Height:    h.Height,
		Viewpoint: append([]float32{}, h.Viewpoint...),
	}
}

// TypeEqual checks that the PointClouds have same field structure.
func (h *PointCloudHeader) TypeEqual(pch *PointCloudHeader) bool {
	if len(h.Fields) != len(pch.Fields) ||
		len(h.Size) != len(pch.Size) ||
		len(h.Type) != len(pch.Type) ||
		len(h.Count) != len(pch.Count) {
		return false
	}
	for i, f := range h.Fields {
		if pch.Fields[i] != f {
			return false
		}
	}
	for i, s := range h.Size {
		if pch.Size[i] != s {
			return false
		}
	}
	for i, t := range h.Type {
		if pch.Type[i] != t {
			return false
		}
	}
	for i, c := range h.Count {
		if pch.Count[i] != c {
			return false
		}
	}
	return true
}

func (pp *PointCloudHeader) Stride() int {
	var stride int
	for i := range pp.Fields {
		stride += pp.Count[i] * pp.Size[i]
	}
	return stride
}

type PointCloud struct {
	PointCloudHeader
	Points int

	Data      []byte
	dataFloat []float32
}

// CopyTo copies n points to dst.
// Source and destination PointClouds must have same field structure.
func (pp *PointCloud) CopyTo(dst *PointCloud, dstIndex, srcIndex, n int) {
	stride := pp.Stride()
	si := srcIndex * stride
	di := dstIndex * stride
	nb := n * stride
	copy(dst.Data[di:di+nb], pp.Data[si:si+nb])
}

func (pp *PointCloud) Float32Iterator(name string) (Float32Iterator, error) {
	offset := 0
	for i, fn := range pp.Fields {
		if fn == name {
			if pp.Stride()&3 == 0 && offset&3 == 0 {
				// Aligned
				if pp.dataFloat == nil || float.IsShadowing(pp.Data, pp.dataFloat) {
					pp.dataFloat = float.ByteSliceAsFloat32Slice(pp.Data)
				}
				return &float32Iterator{
					data:   pp.dataFloat,
					pos:    offset / 4,
					stride: pp.Stride() / 4,
				}, nil
			}
			return &binaryFloat32Iterator{
				binaryIterator: binaryIterator{
					data:   pp.Data,
					pos:    offset,
					stride: pp.Stride(),
				},
			}, nil
		}
		offset += pp.Size[i] * pp.Count[i]
	}
	return nil, errors.New("invalid field name")
}

func (pp *PointCloud) Float32Iterators(names ...string) ([]Float32Iterator, error) {
	var its []Float32Iterator
	for _, name := range names {
		it, err := pp.Float32Iterator(name)
		if err != nil {
			return nil, err
		}
		its = append(its, it)
	}
	return its, nil
}

func (pp *PointCloud) Vec3Iterator() (Vec3Iterator, error) {
	var xyz int
	var fieldName string
	for _, name := range pp.Fields {
		if name == "xyz" {
			xyz = 3
			fieldName = name
			break
		}
		if name == "x" && xyz == 0 {
			xyz = 1
			fieldName = name
		} else if name == "y" && xyz == 1 {
			xyz = 2
		} else if name == "z" && xyz == 2 {
			xyz = 3
			break
		} else {
			xyz = 0
		}
	}
	if xyz != 3 {
		return pp.naiveVec3Iterator()
	}
	it, err := pp.Float32Iterator(fieldName)
	if err != nil {
		return nil, err
	}
	vit, ok := it.(*float32Iterator)
	if !ok {
		return pp.naiveVec3Iterator()
	}
	return vit, nil
}

func (pp *PointCloud) naiveVec3Iterator() (Vec3Iterator, error) {
	its, err := pp.Float32Iterators("x", "y", "z")
	if err != nil {
		return nil, err
	}
	return naiveVec3Iterator{its[0], its[1], its[2]}, nil
}

func (pp *PointCloud) Uint32Iterator(name string) (Uint32Iterator, error) {
	offset := 0
	for i, fn := range pp.Fields {
		if fn == name {
			return &binaryUint32Iterator{
				binaryIterator: binaryIterator{
					data:   pp.Data,
					pos:    offset,
					stride: pp.Stride(),
				},
			}, nil
		}
		offset += pp.Size[i] * pp.Count[i]
	}
	return nil, errors.New("invalid field name")
}
