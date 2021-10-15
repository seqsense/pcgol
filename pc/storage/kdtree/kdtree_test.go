package kdtree

import (
	"fmt"
	"math/rand"
	"reflect"
	"sort"
	"testing"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/storage"
)

var _ storage.Search = &KDTree{} // KDTree must implement storage.Search

func createTestPointCloud(t *testing.T) pc.Vec3Iterator {
	t.Helper()
	pp := &pc.PointCloud{
		PointCloudHeader: pc.PointCloudHeader{
			Fields: []string{"x", "y", "z"},
			Size:   []int{4, 4, 4},
			Type:   []string{"F", "F", "F"},
			Count:  []int{1, 1, 1},
			Width:  7,
			Height: 1,
		},
		Points: 7,
		Data:   make([]byte, 7*4*3),
	}
	it, err := pp.Vec3Iterator()
	if err != nil {
		t.Fatal(err)
	}
	it.SetVec3(mat.Vec3{4, 1, 0}) // 0
	it.Incr()
	it.SetVec3(mat.Vec3{2, 2, 1}) // 1
	it.Incr()
	it.SetVec3(mat.Vec3{5, 0, 0}) // 2
	it.Incr()
	it.SetVec3(mat.Vec3{3, 0, 0}) // 3
	it.Incr()
	it.SetVec3(mat.Vec3{0, 1, 0}) // 4
	it.Incr()
	it.SetVec3(mat.Vec3{1, 0, 0}) // 5
	it.Incr()
	it.SetVec3(mat.Vec3{6, 2, 1}) // 6
	//      3
	//     / \
	//    /   \
	//   4     0
	//  / \   / \
	// 5   1 2   6

	it, err = pp.Vec3Iterator()
	if err != nil {
		t.Fatal(err)
	}
	return it
}

func kdtreeDeepExpectEqual(t *testing.T, a, b *KDTree) bool {
	t.Helper()
	return reflect.DeepEqual(a.root, b.root) && reflect.DeepEqual(a.Vec3RandomAccessor, b.Vec3RandomAccessor)
}

func TestKDtree(t *testing.T) {
	it := createTestPointCloud(t)
	kdt := New(it)

	expectedTree := &KDTree{
		Vec3RandomAccessor: it,
		root: &node{
			children: [2]*node{
				&node{
					children: [2]*node{
						&node{id: 5, dim: 2},
						&node{id: 1, dim: 2},
					},
					id:  4,
					dim: 1,
				},
				&node{
					children: [2]*node{
						&node{id: 2, dim: 2},
						&node{id: 6, dim: 2},
					},
					id:  0,
					dim: 1,
				},
			},
			id:  3,
			dim: 0,
		},
	}
	if !kdtreeDeepExpectEqual(t, expectedTree, kdt) {
		t.Fatalf("Expected:\n%v\nGot:\n%v", expectedTree, kdt)
	}

	t.Run("SearchNode", func(t *testing.T) {
		testCases := []struct {
			p      mat.Vec3
			nodeID int
		}{
			{
				p:      mat.Vec3{5, 0, 0},
				nodeID: 2,
			},
			{
				p:      mat.Vec3{2, 2, 1},
				nodeID: 1,
			},
			{
				p:      mat.Vec3{4, 1, 0},
				nodeID: 6,
			},
		}
		for _, tt := range testCases {
			tt := tt
			t.Run(fmt.Sprintf("(%.0f,%.0f,%.0f)", tt.p[0], tt.p[1], tt.p[2]), func(t *testing.T) {
				n := kdt.searchLeafNode(tt.p, []*node{kdt.root})
				if n[len(n)-1].id != tt.nodeID {
					t.Errorf("Expected node.id: %d, got: %d", tt.nodeID, n[len(n)-1].id)
				}
			})
		}
	})
	t.Run("Nearest", func(t *testing.T) {
		testCases := []struct {
			p        mat.Vec3
			nodeID   int
			distSq   float32
			maxRange float32
		}{
			{
				p:        mat.Vec3{5, 0, 0},
				nodeID:   2,
				distSq:   0,
				maxRange: 1.0,
			},
			{
				p:        mat.Vec3{5, 0, 0.1},
				nodeID:   2,
				distSq:   0.1 * 0.1,
				maxRange: 1.0,
			},
			{
				p:        mat.Vec3{4.9, 0.0, 0.0},
				nodeID:   2,
				distSq:   0.1 * 0.1,
				maxRange: 1.0,
			},
			{
				p:        mat.Vec3{3, 0, 0},
				nodeID:   3,
				distSq:   0,
				maxRange: 1.0,
			},
			{
				p:        mat.Vec3{3, 0, 0.1},
				nodeID:   3,
				distSq:   0.1 * 0.1,
				maxRange: 1.0,
			},
			{
				p:        mat.Vec3{2.1, 1.9, 1},
				nodeID:   1,
				distSq:   2 * 0.1 * 0.1,
				maxRange: 1.0,
			},
			{
				p:        mat.Vec3{2.1, 2.1, 1},
				nodeID:   1,
				distSq:   2 * 0.1 * 0.1,
				maxRange: 1.0,
			},
			{
				p:        mat.Vec3{3.9, 1, 0},
				nodeID:   0,
				distSq:   0.1 * 0.1,
				maxRange: 1.0,
			},
			{
				p:        mat.Vec3{4.1, 1, 0},
				nodeID:   0,
				distSq:   0.1 * 0.1,
				maxRange: 1.0,
			},
			{
				p:        mat.Vec3{4.2, 1, 0},
				nodeID:   -1,
				distSq:   0.1 * 0.1,
				maxRange: 0.1,
			},
		}
		const eps = 0.00001
		for _, tt := range testCases {
			tt := tt
			t.Run(fmt.Sprintf(
				"(%.1f,%.1f,%.1f)-%0.1f",
				tt.p[0], tt.p[1], tt.p[2], tt.maxRange,
			), func(t *testing.T) {
				i, dsq := kdt.Nearest(tt.p, tt.maxRange)
				if i != tt.nodeID {
					t.Errorf("Expected id: %d, got: %d", tt.nodeID, i)
				}
				if dsq < tt.distSq-eps || tt.distSq+eps < dsq {
					t.Errorf("Expected distance^2: %0.4f, got: %0.4f", tt.distSq, dsq)
				}
			})
		}
	})

	t.Run("Range", func(t *testing.T) {
		pp := &pc.PointCloud{
			PointCloudHeader: pc.PointCloudHeader{
				Fields: []string{"x", "y", "z"},
				Size:   []int{4, 4, 4},
				Type:   []string{"F", "F", "F"},
				Count:  []int{1, 1, 1},
				Width:  7,
				Height: 1,
			},
			Points: 7,
			Data:   make([]byte, 7*4*3),
		}
		it, err := pp.Vec3Iterator()
		if err != nil {
			t.Fatal(err)
		}
		it.SetVec3(mat.Vec3{0.0, 0.2, 0.0}) // 0
		it.Incr()
		it.SetVec3(mat.Vec3{3.0, 0.0, 0.0}) // 1
		it.Incr()
		it.SetVec3(mat.Vec3{0.2, 0.0, 0.0}) // 2
		it.Incr()
		it.SetVec3(mat.Vec3{0.0, 1.0, 0.0}) // 3
		it.Incr()
		it.SetVec3(mat.Vec3{0.0, 0.0, 5.0}) // 4
		it.Incr()
		it.SetVec3(mat.Vec3{0.5, 0.0, 0.0}) // 5
		it.Incr()
		it.SetVec3(mat.Vec3{0.0, 0.0, 0.4}) // 6

		it, err = pp.Vec3Iterator()
		if err != nil {
			t.Fatal(err)
		}
		kdt := New(it)

		testCases := []struct {
			p        mat.Vec3
			maxRange float32
			ids      []int
			dsqs     []float32
		}{
			{
				p:        mat.Vec3{10, 10, 10},
				maxRange: 1,
				ids:      []int{},
				dsqs:     []float32{},
			},
			{
				p:        mat.Vec3{0, 0.2, 0},
				maxRange: 0.05,
				ids:      []int{0},
				dsqs:     []float32{0.0},
			},
			{
				p:        mat.Vec3{0, 0.2, 0},
				maxRange: 0.3,
				ids:      []int{0, 2},
				dsqs:     []float32{0.0, 0.08},
			},
			{
				p:        mat.Vec3{0, 0.2, 0},
				maxRange: 0.45,
				ids:      []int{0, 2, 6},
				dsqs:     []float32{0.0, 0.08, 0.2},
			},
			{
				p:        mat.Vec3{0, 0.2, 0},
				maxRange: 0.6,
				ids:      []int{0, 2, 6, 5},
				dsqs:     []float32{0.0, 0.08, 0.2, 0.29},
			},
		}
		const eps = 0.00001
		for _, tt := range testCases {
			tt := tt
			t.Run(fmt.Sprintf(
				"(%.1f,%.1f,%.1f)-%0.1f",
				tt.p[0], tt.p[1], tt.p[2], tt.maxRange,
			), func(t *testing.T) {
				ids, dsqs := kdt.Range(tt.p, tt.maxRange)

				if len(ids) != len(tt.ids) {
					t.Fatalf("Expected: %d, got %d", len(tt.ids), len(ids))
				}
				if len(dsqs) != len(tt.dsqs) {
					t.Fatalf("Expected: %d, got %d", len(tt.dsqs), len(dsqs))
				}
				if !reflect.DeepEqual(tt.ids, ids) {
					t.Fatalf("Expected: %d, got %d", tt.ids, ids)
				}

				for i := 0; i < len(dsqs); i++ {
					if dsqs[i] < tt.dsqs[i]-eps || tt.dsqs[i]+eps < dsqs[i] {
						t.Errorf("Expected distance^2: %0.4f, got: %0.4f", tt.dsqs[i], dsqs[i])
					}
				}
			})
		}
	})

	t.Run("FindMinimum", func(t *testing.T) {
		testCases := []struct {
			dim    int
			nodeID int
		}{
			{dim: 0, nodeID: 4},
			{dim: 1, nodeID: 3},
			{dim: 2, nodeID: 3},
			{dim: 3, nodeID: -1},
		}

		for _, tt := range testCases {
			tt := tt
			t.Run(fmt.Sprintf("dim:%d", tt.dim), func(t *testing.T) {
				nID, err := kdt.findMinimumImpl(kdt.root, tt.dim, 0)
				if nID != tt.nodeID {
					t.Errorf("Expected %v, got: %v", tt.nodeID, nID)
				}
				if tt.dim > 2 && err == nil {
					t.Errorf("Expected an error when dim>2")
				}
			})
		}
	})

	t.Run("DeletePoint", func(t *testing.T) {
		it := createTestPointCloud(t)
		testCases := map[string][]struct {
			pID          int
			expectedTree *KDTree
			hasError     bool
		}{
			"LeafThenNodeWithRightSubTree": {
				{
					pID:      5,
					hasError: false,
					expectedTree: &KDTree{
						Vec3RandomAccessor: it,
						root: &node{
							children: [2]*node{
								&node{
									children: [2]*node{
										nil,
										&node{id: 1, dim: 2},
									},
									id:  4,
									dim: 1,
								},
								&node{
									children: [2]*node{
										&node{id: 2, dim: 2},
										&node{id: 6, dim: 2},
									},
									id:  0,
									dim: 1,
								},
							},
							id:  3,
							dim: 0,
						},
					},
				},
				{
					pID:      4,
					hasError: false,
					expectedTree: &KDTree{
						Vec3RandomAccessor: it,
						root: &node{
							children: [2]*node{
								&node{
									id:  1,
									dim: 1,
								},
								&node{
									children: [2]*node{
										&node{id: 2, dim: 2},
										&node{id: 6, dim: 2},
									},
									id:  0,
									dim: 1,
								},
							},
							id:  3,
							dim: 0,
						},
					},
				},
			},
			"RootThenNodeWithLeftSubTree": {
				{
					pID:      3,
					hasError: false,
					expectedTree: &KDTree{
						Vec3RandomAccessor: it,
						root: &node{
							children: [2]*node{
								&node{
									children: [2]*node{
										&node{
											id:  5,
											dim: 2,
										},
										&node{
											id:  1,
											dim: 2,
										},
									},
									id:  4,
									dim: 1,
								},
								&node{
									children: [2]*node{
										&node{
											id:  2,
											dim: 2,
										},
									},
									id:  6,
									dim: 1,
								},
							},
							id:  0,
							dim: 0,
						},
					},
				},
				{
					pID:      6,
					hasError: false,
					expectedTree: &KDTree{
						Vec3RandomAccessor: it,
						root: &node{
							children: [2]*node{
								&node{
									children: [2]*node{
										&node{
											id:  5,
											dim: 2,
										},
										&node{
											id:  1,
											dim: 2,
										},
									},
									id:  4,
									dim: 1,
								},
								&node{
									id:  2,
									dim: 1,
								},
							},
							id:  0,
							dim: 0,
						},
					},
				},
			},
			"NodeWithBothLeftAndRightSubTrees": {
				{
					pID:      0,
					hasError: false,
					expectedTree: &KDTree{
						Vec3RandomAccessor: it,
						root: &node{
							children: [2]*node{
								&node{
									children: [2]*node{
										&node{id: 5, dim: 2},
										&node{id: 1, dim: 2},
									},
									id:  4,
									dim: 1,
								},
								&node{
									children: [2]*node{
										&node{id: 2, dim: 2},
									},
									id:  6,
									dim: 1,
								},
							},
							id:  3,
							dim: 0,
						},
					},
				},
			},
			"TwiceTheSamePoint": {
				{
					pID:      3,
					hasError: false,
					expectedTree: &KDTree{
						Vec3RandomAccessor: it,
						root: &node{
							children: [2]*node{
								&node{
									children: [2]*node{
										&node{
											id:  5,
											dim: 2,
										},
										&node{
											id:  1,
											dim: 2,
										},
									},
									id:  4,
									dim: 1,
								},
								&node{
									children: [2]*node{
										&node{
											id:  2,
											dim: 2,
										},
									},
									id:  6,
									dim: 1,
								},
							},
							id:  0,
							dim: 0,
						},
					},
				},
				{
					pID:      3,
					hasError: false,
					expectedTree: &KDTree{
						Vec3RandomAccessor: it,
						root: &node{
							children: [2]*node{
								&node{
									children: [2]*node{
										&node{
											id:  5,
											dim: 2,
										},
										&node{
											id:  1,
											dim: 2,
										},
									},
									id:  4,
									dim: 1,
								},
								&node{
									children: [2]*node{
										&node{
											id:  2,
											dim: 2,
										},
									},
									id:  6,
									dim: 1,
								},
							},
							id:  0,
							dim: 0,
						},
					},
				},
			},
			"InvalidPointID": {
				{
					pID:      -1,
					hasError: true,
					expectedTree: &KDTree{
						Vec3RandomAccessor: it,
						root: &node{
							children: [2]*node{
								&node{
									children: [2]*node{
										&node{id: 5, dim: 2},
										&node{id: 1, dim: 2},
									},
									id:  4,
									dim: 1,
								},
								&node{
									children: [2]*node{
										&node{id: 2, dim: 2},
										&node{id: 6, dim: 2},
									},
									id:  0,
									dim: 1,
								},
							},
							id:  3,
							dim: 0,
						},
					},
				},
				{
					pID:      123,
					hasError: true,
					expectedTree: &KDTree{
						Vec3RandomAccessor: it,
						root: &node{
							children: [2]*node{
								&node{
									children: [2]*node{
										&node{id: 5, dim: 2},
										&node{id: 1, dim: 2},
									},
									id:  4,
									dim: 1,
								},
								&node{
									children: [2]*node{
										&node{id: 2, dim: 2},
										&node{id: 6, dim: 2},
									},
									id:  0,
									dim: 1,
								},
							},
							id:  3,
							dim: 0,
						},
					},
				},
			},
		}
		for name, steps := range testCases {
			steps := steps
			t.Run(name, func(t *testing.T) {
				kdt := New(it)
				for _, tt := range steps {
					err := kdt.DeletePoint(tt.pID)
					testKDT := tt.expectedTree
					if tt.hasError && err == nil {
						t.Errorf("Expected an error when trying to delete a point that is not in the tree")
					}
					if !kdtreeDeepExpectEqual(t, kdt, testKDT) {
						t.Fatalf("Expected:\n%v\nGot:\n%v", testKDT, kdt)
					}
				}
			})
		}
	})
}

func ExampleKDTree_String() {
	pp := &pc.PointCloud{
		PointCloudHeader: pc.PointCloudHeader{
			Fields: []string{"x", "y", "z"},
			Size:   []int{4, 4, 4},
			Type:   []string{"F", "F", "F"},
			Count:  []int{1, 1, 1},
			Width:  7,
			Height: 1,
		},
		Points: 7,
		Data:   make([]byte, 7*4*3),
	}
	it, _ := pp.Vec3Iterator()
	it.SetVec3(mat.Vec3{4, 1, 0}) // 0
	it.Incr()
	it.SetVec3(mat.Vec3{2, 2, 1}) // 1
	it.Incr()
	it.SetVec3(mat.Vec3{5, 0, 0}) // 2
	it.Incr()
	it.SetVec3(mat.Vec3{3, 0, 0}) // 3
	it.Incr()
	it.SetVec3(mat.Vec3{0, 1, 0}) // 4
	it.Incr()
	it.SetVec3(mat.Vec3{1, 0, 0}) // 5
	it.Incr()
	it.SetVec3(mat.Vec3{6, 2, 1}) // 6
	it, _ = pp.Vec3Iterator()
	kdt := New(it)
	fmt.Println(kdt)
	// Output:
	//                     -> (6,2) {6.000, 2.000, 1.000}
	//           -> (0,1) {4.000, 1.000, 0.000}
	//                     -> (2,2) {5.000, 0.000, 0.000}
	// -> (3,0) {3.000, 0.000, 0.000}
	//                     -> (1,2) {2.000, 2.000, 1.000}
	//           -> (4,1) {0.000, 1.000, 0.000}
	//                     -> (5,2) {1.000, 0.000, 0.000}
}

func testNearestRandomCloud(t *testing.T, it pc.Vec3Iterator, k *KDTree, ns *naiveSearch, nPoints int, width float32) {
	for i := 0; i < nPoints; i++ {
		p := randomPoint(width)
		maxRange := rand.Float32() * width

		idNaive, dsqNaive := ns.Nearest(p, maxRange)
		idKDTree, dsqKDTree := k.Nearest(p, maxRange)
		strNaive := ""
		if idNaive >= 0 {
			strNaive = it.Vec3At(idNaive).String()
		}
		strKDTree := ""
		if idKDTree >= 0 {
			strKDTree = it.Vec3At(idKDTree).String()
		}
		if idNaive != idKDTree || dsqNaive != dsqKDTree {
			t.Fatalf(
				"%d %0.3f %s: Expected: %d %0.3f %s, got: %d %0.3f %s",
				i, maxRange, p, idNaive, dsqNaive, strNaive, idKDTree, dsqKDTree, strKDTree,
			)
		}
	}
}

func TestKDtree_Nearest_randomCloud(t *testing.T) {
	const (
		nPoints = 100
		width   = 10.0
	)

	pp := generateRandomCloud(t, nPoints, width)
	it, err := pp.Vec3Iterator()
	if err != nil {
		t.Fatal(err)
	}
	kdt := New(it)
	ns := newNaiveSearch(it)
	testNearestRandomCloud(t, it, kdt, ns, nPoints, width)
}

func TestKDtree_findMinimum_randomCloud(t *testing.T) {
	const (
		nClouds = 100
		nPoints = 100
		width   = 10.0
	)

	for i := 0; i < nClouds; i++ {
		pp := generateRandomCloud(t, nPoints, width)
		it, err := pp.Vec3Iterator()
		if err != nil {
			t.Fatal(err)
		}
		kdt := New(it)
		ns := newNaiveSearch(it)
		for dim := 0; dim < 3; dim++ {
			nodeIDKDTree, err := kdt.findMinimumImpl(kdt.root, dim, 0)
			if err != nil {
				t.Fatal(err)
			}
			nodeIDNaive := ns.findMinimum(dim)
			if nodeIDKDTree != nodeIDNaive {
				t.Fatalf("dim %d: Expected: %d, got %d", dim, nodeIDNaive, nodeIDKDTree)
			}
		}
	}
}

func TestKDtree_DeletePoint_randomCloud(t *testing.T) {
	const (
		nPoints = 100
		width   = 10.0
	)

	pp := generateRandomCloud(t, nPoints, width)
	it, err := pp.Vec3Iterator()
	if err != nil {
		t.Fatal(err)
	}
	kdt := New(it)
	ns := newNaiveSearch(it)
	for _, i := range rand.Perm(nPoints / 3) {
		err := kdt.DeletePoint(i)
		if err != nil {
			t.Fatal(err)
		}
		ns.deletePoint(i)
	}
	testNearestRandomCloud(t, it, kdt, ns, nPoints, width)
}

func TestKDtree_Range_randomCloud(t *testing.T) {
	const (
		nPoints = 100
		width   = 10.0
	)

	pp := generateRandomCloud(t, nPoints, width)
	it, err := pp.Vec3Iterator()
	if err != nil {
		t.Fatal(err)
	}
	kdt := New(it)
	ns := newNaiveSearch(it)
	for i := 0; i < nPoints; i++ {
		p := randomPoint(width)
		maxRange := rand.Float32() * width

		idsNaive, dsqsNaive := ns.Range(p, maxRange)
		idsKDTree, dsqsKDTree := kdt.Range(p, maxRange)

		if !reflect.DeepEqual(idsNaive, idsKDTree) {
			t.Fatalf(
				"%d %0.3f %s: Expected: %v, got %v", i, maxRange, p, idsNaive, idsKDTree,
			)
		}

		if !reflect.DeepEqual(dsqsNaive, dsqsKDTree) {
			t.Fatalf(
				"%d %0.3f %s: Expected: %v, got %v", i, maxRange, p, dsqsNaive, dsqsKDTree,
			)
		}
	}
}

type naiveSearch struct {
	ra            pc.Vec3RandomAccessor
	deletedPoints []bool
}

func newNaiveSearch(ra pc.Vec3RandomAccessor) *naiveSearch {
	return &naiveSearch{
		ra:            ra,
		deletedPoints: make([]bool, ra.Len()),
	}
}

func (s *naiveSearch) Nearest(p mat.Vec3, maxRange float32) (int, float32) {
	dsq := maxRange * maxRange
	id := -1
	for i := 0; i < s.ra.Len(); i++ {
		if s.deletedPoints[i] {
			continue
		}
		dsq1 := s.ra.Vec3At(i).Sub(p).NormSq()
		if dsq1 < dsq {
			id, dsq = i, dsq1
		}
	}
	return id, dsq
}

func (s *naiveSearch) Range(p mat.Vec3, maxRange float32) ([]int, []float32) {
	dsqTh := maxRange * maxRange
	nghs := newNeighbors()

	for i := 0; i < s.ra.Len(); i++ {
		if s.deletedPoints[i] {
			continue
		}
		dsq := s.ra.Vec3At(i).Sub(p).NormSq()
		if dsq < dsqTh {
			nghs.add(i, dsq)
		}
	}
	sort.Sort(nghs)
	return nghs.ids, nghs.dsqs
}

func (s *naiveSearch) findMinimum(dim int) int {
	id := -1
	var min float32
	for i := 0; i < s.ra.Len(); i++ {
		if s.deletedPoints[i] {
			continue
		}
		p := s.ra.Vec3At(i)
		if id == -1 || p[dim] < min {
			min = p[dim]
			id = i
		}
	}
	return id
}

func (s *naiveSearch) deletePoint(id int) {
	s.deletedPoints[id] = true
}

func randomPoint(width float32) mat.Vec3 {
	return mat.Vec3{
		rand.Float32() * width,
		rand.Float32() * width,
		rand.Float32() * width,
	}
}

func generateRandomCloud(t testing.TB, nPoints int, width float32) *pc.PointCloud {
	t.Helper()
	pp := &pc.PointCloud{
		PointCloudHeader: pc.PointCloudHeader{
			Fields: []string{"x", "y", "z"},
			Size:   []int{4, 4, 4},
			Type:   []string{"F", "F", "F"},
			Count:  []int{1, 1, 1},
			Width:  nPoints,
			Height: 1,
		},
		Points: nPoints,
		Data:   make([]byte, nPoints*4*3),
	}
	it, err := pp.Vec3Iterator()
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < nPoints; i++ {
		it.SetVec3(randomPoint(width))
		it.Incr()
	}
	return pp
}

func BenchmarkKDTree_Nearest(b *testing.B) {
	const (
		width    = 10.0
		nTargets = 100
	)
	for _, nPoints := range []int{100, 1000, 10000, 100000} {
		b.Run(fmt.Sprintf("%dpoints", nPoints), func(b *testing.B) {
			pp := generateRandomCloud(b, nPoints, width)
			targets := generateRandomCloud(b, nTargets, width)

			it, err := pp.Vec3Iterator()
			if err != nil {
				b.Fatal(err)
			}
			kdt := New(it)
			ns := newNaiveSearch(it)

			itTargets, err := targets.Vec3Iterator()
			if err != nil {
				b.Fatal(err)
			}

			b.Run("KDTree", func(b *testing.B) {
				for i := 0; i < b.N; i++ {
					target := itTargets.Vec3At(i % nTargets)
					_, _ = kdt.Nearest(target, width)
				}
			})
			b.Run("Naive", func(b *testing.B) {
				for i := 0; i < b.N; i++ {
					target := itTargets.Vec3At(i % nTargets)
					_, _ = ns.Nearest(target, width)
				}
			})
		})
	}
}
