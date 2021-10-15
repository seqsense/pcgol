package storage

import (
	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
)

type Neighbor struct {
	ID     int
	DistSq float32
}

type Search interface {
	pc.Vec3RandomAccessor
	Nearest(p mat.Vec3, maxRange float32) Neighbor
	Range(p mat.Vec3, maxRange float32) []Neighbor
}
