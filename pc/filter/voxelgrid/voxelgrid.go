package voxelgrid

import (
	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/filter"
)

type Options struct {
	LeafSize mat.Vec3
}

type voxelGrid struct {
	Options
}

type voxel struct {
	sum   mat.Vec3
	num   int
	index int
}

func New(leafSize mat.Vec3) filter.Filter {
	vg := &voxelGrid{
		Options: Options{
			LeafSize: leafSize,
		},
	}
	return vg
}

func (f *voxelGrid) Filter(pp *pc.PointCloud) (*pc.PointCloud, error) {
	it, err := pp.Vec3Iterator()
	if err != nil {
		return nil, err
	}
	min, max, err := pc.MinMaxVec3(it)
	if err != nil {
		return nil, err
	}

	size := max.Sub(min)
	xs, ys, zs := int(size[0]/f.LeafSize[0]), int(size[1]/f.LeafSize[1]), int(size[2]/f.LeafSize[2])
	voxels := make([]voxel, (xs+1)*(ys+1)*(zs+1))

	var n int
	for i := 0; it.IsValid(); it.Incr() {
		p := it.Vec3().Sub(min)
		x, y, z := int(p[0]/f.LeafSize[0]), int(p[1]/f.LeafSize[1]), int(p[2]/f.LeafSize[2])
		v := &voxels[x+xs*(y+ys*z)]
		if v.num == 0 {
			v.index = i
			n++
		}
		v.num++
		v.sum = v.sum.Add(p)
		i++
	}

	newPc := &pc.PointCloud{
		PointCloudHeader: pp.Clone(),
		Points:           n,
		Data:             make([]byte, pp.Stride()*n),
	}
	newPc.Width = n
	newPc.Height = 1
	jt, err := newPc.Vec3Iterator()
	if err != nil {
		return nil, err
	}
	var jStart int
	stride := pp.Stride()
	for i := range voxels {
		v := &voxels[i]
		if n := v.num; n > 0 {
			iStart := v.index * stride
			copy(newPc.Data[jStart:jStart+stride], pp.Data[iStart:iStart+stride])
			if n > 1 {
				jt.SetVec3(v.sum.Mul(1.0 / float32(n)).Add(min))
			}
			jt.Incr()
			jStart += stride
		}
	}

	return newPc, nil
}
