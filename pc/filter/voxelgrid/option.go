package voxelgrid

import (
	"github.com/seqsense/pcgol/mat"
)

type Options struct {
	LeafSize  mat.Vec3
	ChunkSize [3]int
}

type Option func(*Options)

func WithChunkSize(s [3]int) Option {
	return Option(func(o *Options) {
		o.ChunkSize = s
	})
}
