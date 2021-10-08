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
	Pairs(target pc.Vec3RandomAccessor) []PointToPointCorrespondence
}

func NewNearestPointCorresponder(base storage.Search, maxDist float32) PointToPointCorresponder {
	return &nearestPointCorresponder{
		base:    base,
		maxDist: maxDist,
	}
}

type nearestPointCorresponder struct {
	base    storage.Search
	maxDist float32
}

func (c *nearestPointCorresponder) Pairs(target pc.Vec3RandomAccessor) []PointToPointCorrespondence {
	n := target.Len()
	out := make([]PointToPointCorrespondence, 0, n)
	for i := 0; i < n; i++ {
		id, dsq := c.base.Nearest(target.Vec3At(i), c.maxDist)
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
