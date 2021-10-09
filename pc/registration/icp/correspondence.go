package icp

import (
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/storage"
)

type PointToPointCorrespondence struct {
	BaseID          int
	TargetID        int
	SquaredDistance float32
}

type PointToPointCorresponder interface {
	Pairs(base storage.Search, target pc.Vec3RandomAccessor) []PointToPointCorrespondence
}

type NearestPointCorresponder struct {
	MaxDist float32
}

func (c *NearestPointCorresponder) Pairs(base storage.Search, target pc.Vec3RandomAccessor) []PointToPointCorrespondence {
	n := target.Len()
	out := make([]PointToPointCorrespondence, 0, n)
	for i := 0; i < n; i++ {
		id, dsq := base.Nearest(target.Vec3At(i), c.MaxDist)
		if id < 0 {
			continue
		}
		out = append(out, PointToPointCorrespondence{
			BaseID:          id,
			TargetID:        i,
			SquaredDistance: dsq,
		})
	}
	return out
}
