package storage

import (
	"github.com/seqsense/pcgol/mat"
)

type Search interface {
	Nearest(p mat.Vec3, maxRange float32) (int, float32)
}
