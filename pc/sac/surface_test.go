package sac

import (
	"reflect"
	"sort"
	"testing"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/storage/voxelgrid"
)

func TestVoxelGridSurfaceModel(t *testing.T) {
	pc0 := pc.Vec3Slice{
		mat.Vec3{0.0, 0.0, 0.0},
		mat.Vec3{0.1, 0.0, 0.1},
		mat.Vec3{0.2, 0.0, 0.2},
		mat.Vec3{0.2, 0.1, 0.6}, // outlier
		mat.Vec3{0.0, 0.1, 0.0},
		mat.Vec3{0.1, 0.1, 0.1},
		mat.Vec3{0.2, 0.1, 0.2},
		mat.Vec3{0.0, 0.2, 0.0},
		mat.Vec3{0.1, 0.2, 0.1},
		mat.Vec3{0.2, 0.2, 0.2},
	}
	pc1 := pc.Vec3Slice{
		mat.Vec3{0.0, 0.0, 0.0},
		mat.Vec3{0.1, 0.0, 0.1},
		mat.Vec3{0.2, 0.0, 0.2},
		mat.Vec3{0.2, 0.1, 0.6}, // outlier
		mat.Vec3{0.0, 0.1, 0.1},
		mat.Vec3{0.1, 0.1, 0.2},
		mat.Vec3{0.2, 0.1, 0.3},
		mat.Vec3{0.0, 0.2, 0.2},
		mat.Vec3{0.1, 0.2, 0.3},
		mat.Vec3{0.2, 0.2, 0.4},
	}

	for name, tt := range map[string]struct {
		origin mat.Vec3
		pp     pc.Vec3Slice
	}{
		"Zero_XZ": {
			origin: mat.Vec3{0, 0, 0},
			pp:     pc0,
		},
		"NoZero_XZ": {
			origin: mat.Vec3{0, 0, -0.1},
			pp:     pc0,
		},
		"Zero_XYZ": {
			origin: mat.Vec3{0, 0, 0},
			pp:     pc1,
		},
		"NoZero_XYZ": {
			origin: mat.Vec3{0, 0, -0.1},
			pp:     pc1,
		},
	} {
		tt := tt
		t.Run(name, func(t *testing.T) {
			vg := voxelgrid.New(0.1, [3]int{8, 8, 8}, tt.origin)
			for i, p := range tt.pp {
				vg.Add(p, i)
			}

			t.Run("Surface", func(t *testing.T) {
				m := NewVoxelGridSurfaceModel(vg, tt.pp)
				c, ok := m.Fit([]int{1, 5, 7})
				if !ok {
					t.Fatal("Fit failed")
				}

				indice := c.Inliers(0.1)
				sort.Ints(indice)
				expectedIndice := []int{0, 1, 2, 4, 5, 6, 7, 8, 9}
				if !reflect.DeepEqual(expectedIndice, indice) {
					t.Errorf("Expected inlier: %v, got: %v", expectedIndice, indice)
				}

				t.Run("IsIn", func(t *testing.T) {
					if in := c.IsIn(tt.pp[0], 0.1); !in {
						t.Error("Point on the surface must be determined as IsIn")
					}
					if in := c.IsIn(tt.pp[3], 0.1); in {
						t.Error("Point out of the surface must not be determined as IsIn")
					}
				})
			})
			t.Run("InTheSameLine", func(t *testing.T) {
				m := NewVoxelGridSurfaceModel(vg, tt.pp)
				_, ok := m.Fit([]int{0, 1, 2})
				if ok {
					t.Fatal("Expected failure")
				}
			})
			t.Run("SamePoint", func(t *testing.T) {
				m := NewVoxelGridSurfaceModel(vg, tt.pp)
				_, ok := m.Fit([]int{1, 1, 8})
				if ok {
					t.Fatal("Expected failure")
				}
			})
		})
	}
}
