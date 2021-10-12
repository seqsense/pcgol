package kdtree

import (
	"fmt"
	"math/rand"
	"reflect"
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

func assertKDTreesEqual(t *testing.T, it pc.Vec3Iterator, n1, n2 *node) bool {
	t.Helper()
	if n1 == nil && n2 == nil {
		return true
	}
	if n1 == nil && n2 != nil {
		return false
	}
	if n2 == nil && n1 != nil {
		return false
	}
	if n1.id != n2.id || !it.Vec3At(n1.id).Equal(it.Vec3At(n2.id)) {
		return false
	}
	return assertKDTreesEqual(t, it, n1.children[0], n2.children[0]) &&
		assertKDTreesEqual(t, it, n1.children[1], n2.children[1])
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
	if !reflect.DeepEqual(expectedTree.root, kdt.root) {
		t.Fatalf("Expected:\n%s\nGot:\n%s", expectedTree.root, kdt.root)
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
			t.Run(fmt.Sprintf("dim: %d", tt.dim), func(t *testing.T) {
				id, err := kdt.FindMinimum(tt.dim)
				if id != tt.nodeID {
					t.Errorf("Expected id: %d, got: %d", tt.nodeID, id)
				}
				if tt.dim > 2 && err == nil {
					t.Errorf("Expected an error when dim>2")
				}
			})
		}
	})

	t.Run("deleteNodeImpl", func(t *testing.T) {
		it := createTestPointCloud(t)
		var kdt *KDTree
		testCases := map[string][]struct {
			p            mat.Vec3
			expectedTree *KDTree
		}{
			"leaf then node with right sub tree": {
				{
					p: mat.Vec3{1, 0, 0},
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
					p: mat.Vec3{0, 1, 0},
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
			"root then node with left sub tree": {
				{
					p: mat.Vec3{3, 0, 0},
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
					p: mat.Vec3{6, 2, 1},
					expectedTree: &KDTree{
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
			"node both left and right sub trees": {
				{
					p: mat.Vec3{4, 1, 0},
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
		}
		for name, steps := range testCases {
			t.Run(name, func(t *testing.T) {
				kdt = New(it)
				for _, tt := range steps {
					kdt.deleteNodeImpl(kdt.root, tt.p, 0)
					if !assertKDTreesEqual(t, it, tt.expectedTree.root, kdt.root) {
						t.Fatalf("Expected:\n%v\nGot:\n%v", tt.expectedTree, kdt)
					}
				}
			})
		}
	})

	t.Run("DeleteNode", func(t *testing.T) {
		it := createTestPointCloud(t)
		testCases := []struct {
			p        mat.Vec3
			hasError bool
		}{
			{p: mat.Vec3{6, 2, 1}, hasError: false},
			{p: mat.Vec3{13, 0, 0}, hasError: true},
		}
		for _, tt := range testCases {
			tt := tt
			t.Run(fmt.Sprintf(
				"(%.1f,%.1f,%.1f)",
				tt.p[0], tt.p[1], tt.p[2],
			), func(t *testing.T) {
				kdt := New(it)
				err := kdt.DeleteNode(tt.p)
				if tt.hasError != (err != nil) {
					t.Errorf("Expected an error when trying to delete a point that is not in the tree")
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
	// 	                    -> {6.000, 2.000, 1.000}
	//           -> {4.000, 1.000, 0.000}
	//                     -> {5.000, 0.000, 0.000}
	// -> {3.000, 0.000, 0.000}
	//                     -> {2.000, 2.000, 1.000}
	//           -> {0.000, 1.000, 0.000}
	//                     -> {1.000, 0.000, 0.000}
}

func TestKDtree_randomCloud(t *testing.T) {
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
	ns := &naiveSearch{it}

	for i := 0; i < nPoints; i++ {
		p := randomPoint(width)
		maxRange := rand.Float32() * width

		idNaive, dsqNaive := ns.Nearest(p, maxRange)
		idKDTree, dsqKDTree := kdt.Nearest(p, maxRange)
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

type naiveSearch struct {
	ra pc.Vec3RandomAccessor
}

func (s *naiveSearch) Nearest(p mat.Vec3, maxRange float32) (int, float32) {
	dsq := maxRange * maxRange
	id := -1
	for i := 0; i < s.ra.Len(); i++ {
		dsq1 := s.ra.Vec3At(i).Sub(p).NormSq()
		if dsq1 < dsq {
			id, dsq = i, dsq1
		}
	}
	return id, dsq
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
			ns := &naiveSearch{it}

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
