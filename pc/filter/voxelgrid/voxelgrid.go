package voxelgrid

import (
	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/filter"
)

const initialSliceCap = 256

type voxelGrid struct {
	Options

	voxels []voxel
}

type voxel struct {
	sum   mat.Vec3
	num   int
	index int
}

func New(leafSize mat.Vec3, opts ...Option) filter.Filter {
	vg := &voxelGrid{
		Options: Options{
			LeafSize: leafSize,
		},
	}
	for _, o := range opts {
		o(&vg.Options)
	}
	return vg
}

func (f *voxelGrid) Filter(pp *pc.PointCloud) (*pc.PointCloud, error) {
	it, err := pp.Vec3Iterator()
	if err != nil {
		return nil, err
	}

	vMin, vMax, err := pc.MinMaxVec3(it)
	if err != nil {
		return nil, err
	}
	if f.ChunkSize[0]*f.ChunkSize[1]*f.ChunkSize[2] == 0 {
		return f.filterChunk(vMin, vMax, it, pp)
	}

	size := vMax.Sub(vMin)
	chunkSize := mat.Vec3{
		f.LeafSize[0] * float32(f.ChunkSize[0]),
		f.LeafSize[1] * float32(f.ChunkSize[1]),
		f.LeafSize[2] * float32(f.ChunkSize[2]),
	}
	nx, ny, nz := int(size[0]/chunkSize[0])+1, int(size[1]/chunkSize[1])+1, int(size[2]/chunkSize[2])+1
	nChunks := nx * ny * nz

	outs := make([]*pc.PointCloud, 0, nChunks)
	indices := make([][]int, nChunks)
	nIndices := make([]int, nChunks)

	cid2xyz := func(cid int) mat.Vec3 {
		x := cid % nx
		cid = cid / nx
		y := cid % ny
		z := cid / ny
		return mat.Vec3{float32(x), float32(y), float32(z)}
	}
	vec2cid := func(p mat.Vec3) int {
		x, y, z := int(p[0]/chunkSize[0]), int(p[1]/chunkSize[1]), int(p[2]/chunkSize[2])
		return ((z*ny)+y)*nx + x
	}

	defer func() {
		// Make large slice GC-ed ASAP
		f.voxels = nil
	}()

	// Count points in each chunk and allocate indices
	for i := 0; i < it.Len(); i++ {
		cid := vec2cid(it.Vec3At(i).Sub(vMin))
		nIndices[cid]++
	}
	for i := range indices {
		indices[i] = make([]int, 0, nIndices[i])
	}

	// Build indice for each chunk
	for i := 0; i < it.Len(); i++ {
		cid := vec2cid(it.Vec3At(i).Sub(vMin))
		indices[cid] = append(indices[cid], i)
	}

	// Apply filter to the chunks
	for cid, indice := range indices {
		if len(indice) == 0 {
			continue
		}
		iit := pc.NewVec3RandomAccessorIterator(
			pc.NewIndiceVec3RandomAccessor(it, indice),
		)
		cp := cid2xyz(cid)
		vcMin := vMin.Add(cp.ElementMul(chunkSize))
		out, err := f.filterChunk(vcMin, chunkSize, iit, pp)
		if err != nil {
			return nil, err
		}
		outs = append(outs, out)
	}

	// Combine the outputs
	var n int
	for _, out := range outs {
		n += out.Width * out.Height
	}
	newPc := &pc.PointCloud{
		PointCloudHeader: pp.Clone(),
		Points:           n,
		Data:             make([]byte, 0, pp.Stride()*n),
	}
	newPc.Width = n
	newPc.Height = 1
	for _, out := range outs {
		newPc.Data = append(newPc.Data, out.Data...)
	}
	return newPc, nil
}

func (f *voxelGrid) filterChunk(vMin, size mat.Vec3, it pc.Vec3ConstForwardIterator, pp *pc.PointCloud) (*pc.PointCloud, error) {
	xs, ys, zs := int(size[0]/f.LeafSize[0]), int(size[1]/f.LeafSize[1]), int(size[2]/f.LeafSize[2])
	nVoxels := (xs + 1) * (ys + 1) * (zs + 1)
	if len(f.voxels) < nVoxels {
		f.voxels = make([]voxel, nVoxels)
	} else {
		for i := range f.voxels {
			f.voxels[i] = voxel{}
		}
	}

	var n int
	for ; it.IsValid(); it.Incr() {
		p := it.Vec3().Sub(vMin)
		x, y, z := int(p[0]/f.LeafSize[0]), int(p[1]/f.LeafSize[1]), int(p[2]/f.LeafSize[2])
		v := &f.voxels[x+xs*(y+ys*z)]
		if v.num == 0 {
			v.index = it.RawIndex()
			n++
		}
		v.num++
		v.sum = v.sum.Add(p)
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
	for i := range f.voxels {
		v := &f.voxels[i]
		if n := v.num; n > 0 {
			iStart := v.index * stride
			copy(newPc.Data[jStart:jStart+stride], pp.Data[iStart:iStart+stride])
			if n > 1 {
				jt.SetVec3(v.sum.Mul(1.0 / float32(n)).Add(vMin))
			}
			jt.Incr()
			jStart += stride
		}
	}

	return newPc, nil
}
