package regiongrowing

import (
	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/storage"
)

const initialSliceCap = 8192

type RegionGrowing struct {
	search       storage.Search
	propertyIter pc.Uint32RandomAccessor
}

func New(search storage.Search, propertyIter pc.Uint32RandomAccessor) *RegionGrowing {
	return &RegionGrowing{
		search:       search,
		propertyIter: propertyIter,
	}
}

func (r *RegionGrowing) Segment(p mat.Vec3, maxRange float32) []int {
	indice := make([]int, 0, initialSliceCap)

	neighbors := r.search.Range(p, maxRange)
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
		if r.propertyIter.Uint32At(id) != targetVal {
			continue
		}
		indice = append(indice, id)
		neighbors := r.search.Range(r.search.Vec3At(id), maxRange)
		for _, neighbor := range neighbors {
			if ok := toVisit[neighbor.ID]; !ok {
				next = append(next, neighbor.ID)
				toVisit[neighbor.ID] = true
			}
		}
	}
	return indice
}
