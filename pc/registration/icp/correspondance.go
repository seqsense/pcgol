package icp

import (
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/storage"
)

type Correspondance struct {
	BaseID          int
	TargetID        int
	SquaredDistance float32
}

type Corresponder interface {
	Pairs(target pc.Vec3RandomAccessor) []Correspondance
}

func NewNearestPointCorresponder(base storage.Search, maxDist float32) Corresponder {
	return &nearestPointCorresponder{
		base:    base,
		maxDist: maxDist,
	}
}

type nearestPointCorresponder struct {
	base    storage.Search
	maxDist float32
}

func (c *nearestPointCorresponder) Pairs(target pc.Vec3RandomAccessor) []Correspondance {
	n := target.Len()
	out := make([]Correspondance, 0, n)
	for i := 0; i < n; i++ {
		id, dsq := c.base.Nearest(target.Vec3At(i), c.maxDist)
		if id < 0 {
			continue
		}
		out = append(out, Correspondance{
			BaseID:          id,
			TargetID:        i,
			SquaredDistance: dsq,
		})
	}
	return out
}
