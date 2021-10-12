package storage

import (
	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
)

type Search interface {
	pc.Vec3RandomAccessor
	Nearest(p mat.Vec3, maxRange float32) (int, float32)
}
