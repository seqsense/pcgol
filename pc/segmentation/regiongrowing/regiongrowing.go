package regiongrowing

import (
	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	storage "github.com/seqsense/pcgol/pc/storage/kdtree"
)

const initialSliceCap = 8192

type RegionGrowing struct {
	*storage.KDTree
	propertyIter pc.Uint32Iterator
}

func New(ra pc.Vec3RandomAccessor, propertyIter pc.Uint32Iterator) *RegionGrowing {
	return &RegionGrowing{
		KDTree:       storage.New(ra),
		propertyIter: propertyIter,
	}
}

func (r *RegionGrowing) Segment(p mat.Vec3, maxRange float32) []int {
	indice := make([]int, 0, initialSliceCap)

	neighbors := r.Range(p, maxRange)
	if len(neighbors) == 0 {
		return indice
	}

	targetVal := r.propertyIter.Uint32At(neighbors[0].ID)
	next := make([]int, 0, initialSliceCap)
	toVisit := map[int]bool{}

	for _, neighbor := range neighbors {
		next = append(next, neighbor.ID)
		toVisit[neighbor.ID] = true
	}

	for len(next) > 0 {
		var id int
		id, next = next[0], next[1:]
		if r.propertyIter.Uint32At(id) == targetVal {
			indice = append(indice, id)
		}
		neighbors := r.Range(r.Vec3At(id), maxRange)
		for _, neighbor := range neighbors {
			if ok := toVisit[neighbor.ID]; !ok {
				next = append(next, neighbor.ID)
				toVisit[neighbor.ID] = true
			}
		}
	}
	return indice
}
