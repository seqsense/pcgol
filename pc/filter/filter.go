package filter

import (
	"github.com/seqsense/pcgol/pc"
)

type Filter interface {
	Filter(*pc.PointCloud) (*pc.PointCloud, error)
}
