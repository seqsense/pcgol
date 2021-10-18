package regiongrowing

import (
	"fmt"
	"math/rand"
	"reflect"
	"sort"
	"testing"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
)

func TestRegionGrowingSegment(t *testing.T) {

	createBoxPoints := func(width, length, height, res float32) pc.Vec3Slice {
		points := pc.Vec3Slice{}
		for w := -0.5 * width; w <= 0.5*width; w += res {
			for l := -0.5 * length; l <= 0.5*length; l += res {
				for h := -0.5 * height; h <= 0.5*height; h += res {
					points = append(points, mat.Vec3{w, l, h})
				}
			}
		}
		return points
	}

	objects := map[string]*struct {
		pos    mat.Vec3
		points pc.Vec3Slice
		label  uint32
		indice []int
	}{
		"floor": {
			pos:    mat.Vec3{0, 0, 0},
			points: createBoxPoints(2, 2, 0.01, 0.1),
			label:  0,
		},
		"box1": {
			pos:    mat.Vec3{0, 0, 0.25},
			points: createBoxPoints(0.5, 0.5, 0.5, 0.1),
			label:  1,
		},
		"box2": {
			pos:    mat.Vec3{0, 0.6, 0.4},
			points: createBoxPoints(0.3, 0.3, 0.8, 0.1),
			label:  1,
		},
		"box3": {
			pos:    mat.Vec3{1.5, 0, 0.25},
			points: createBoxPoints(0.25, 0.25, 0.5, 0.05),
			label:  2,
		},
	}

	n := 0
	for _, o := range objects {
		n += len(o.points)
	}

	pp := &pc.PointCloud{
		PointCloudHeader: pc.PointCloudHeader{
			Fields: []string{"x", "y", "z", "label"},
			Size:   []int{4, 4, 4, 4},
			Type:   []string{"F", "F", "F", "U"},
			Count:  []int{1, 1, 1, 1},
			Width:  n,
			Height: 1,
		},
		Points: n,
		Data:   make([]byte, n*4*4),
	}
	it, err := pp.Vec3Iterator()
	if err != nil {
		t.Fatal(err)
	}
	lt, err := pp.Uint32Iterator("label")
	if err != nil {
		t.Fatal(err)
	}

	randomFloat := func(min, max float32) float32 {
		return min + rand.Float32()*(max-min)
	}
	noise := float32(0.01)

	cnt := 0
	for _, o := range objects {
		for _, p := range o.points {
			v := p.Add(o.pos).Add(
				mat.Vec3{
					randomFloat(-noise, noise),
					randomFloat(-noise, noise),
					randomFloat(-noise, noise),
				})
			it.SetVec3(v)
			lt.SetUint32(o.label)
			it.Incr()
			lt.Incr()
			o.indice = append(o.indice, cnt)
			cnt++
		}
	}

	testCases := map[string]struct {
		p        mat.Vec3
		maxRange float32
		indice   []int
	}{
		"Label0": {
			p:        mat.Vec3{0.5, 0.1, 0},
			maxRange: 0.15,
			indice:   objects["floor"].indice,
		},
		"Label1FirstBox": {
			p:        mat.Vec3{0.25, 0.15, 0.15},
			maxRange: 0.15,
			indice:   objects["box1"].indice,
		},
		"Label1SecondBox": {
			p:        mat.Vec3{0, 0.45, 0.4},
			maxRange: 0.15,
			indice:   objects["box2"].indice,
		},
		"Label1BothBoxes": {
			p:        mat.Vec3{0, 0.45, 0.4},
			maxRange: 0.3,
			indice:   append(objects["box1"].indice, objects["box2"].indice...),
		},
		"Label3": {
			p:        mat.Vec3{1.4, 0.125, 0.2},
			maxRange: 0.15,
			indice:   objects["box3"].indice,
		},
		"StillOnlyLabel3": {
			p:        mat.Vec3{1.4, 0.125, 0.2},
			maxRange: 0.5,
			indice:   objects["box3"].indice,
		},
	}

	for name, tt := range testCases {
		tt := tt
		t.Run(fmt.Sprintf(
			"%s: (%.1f,%.1f,%.1f)-%0.1f",
			name, tt.p[0], tt.p[1], tt.p[2], tt.maxRange,
		), func(t *testing.T) {
			it, err := pp.Vec3Iterator()
			if err != nil {
				t.Fatal(err)
			}
			lt, err := pp.Uint32Iterator("label")
			if err != nil {
				t.Fatal(err)
			}
			rg := New(it, lt)

			indice := rg.Segment(tt.p, tt.maxRange)
			sort.Ints(indice)
			if !reflect.DeepEqual(tt.indice, indice) {
				t.Errorf("Expected: %v, got: %v", tt.indice, indice)
			}
		})
	}
}
