package kdtree

import (
	"fmt"
	"sort"
	"strings"
	"sync"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
)

type KDTree struct {
	pc.Vec3RandomAccessor
	root *node

	poolNodeArray *sync.Pool
}

type node struct {
	children [2]*node
	id       int
	dim      int
}

func New(ra pc.Vec3RandomAccessor) *KDTree {
	ids := make([]int, ra.Len())
	for i := 0; i < ra.Len(); i++ {
		ids[i] = i
	}
	root := newNode(ra, ids, 0)
	maxDepth := root.maxDepth(0)
	return &KDTree{
		Vec3RandomAccessor: ra,
		root:               root,

		poolNodeArray: &sync.Pool{
			New: func() interface{} {
				return make([]*node, 0, maxDepth)
			},
		},
	}
}

func (k *KDTree) newNodeArray(n0 *node) []*node {
	nodesBuf := k.poolNodeArray.Get().([]*node)
	return append(nodesBuf[:0], n0)
}

func (k *KDTree) Nearest(p mat.Vec3, maxRange float32) (int, float32) {
	nodesOrig := k.newNodeArray(k.root)
	defer k.poolNodeArray.Put(nodesOrig)

	nodes := k.searchLeafNode(p, nodesOrig)
	return k.nearestImpl(p, nodes, maxRange*maxRange)
}

func (k *KDTree) nearestImpl(p mat.Vec3, nodes []*node, maxRangeSq float32) (int, float32) {
	i := len(nodes) - 1
	id := nodes[i].id
	dsq := (k.Vec3At(id).Sub(p)).NormSq()
	if dsq > maxRangeSq {
		id, dsq = -1, maxRangeSq
	}
	for j := i - 1; j >= 0; j-- {
		pivot := k.Vec3At(nodes[j].id)
		dim := nodes[j].dim
		fromPivot := p[dim] - pivot[dim]
		fromPivotSq := fromPivot * fromPivot
		if fromPivotSq > dsq {
			continue
		}
		dsqPivot := (pivot.Sub(p)).NormSq()
		if dsqPivot < dsq {
			id, dsq = nodes[j].id, dsqPivot
		}
		var nextNode *node
		if nodes[j].children[0] == nodes[j+1] {
			nextNode = nodes[j].children[1]
		} else {
			nextNode = nodes[j].children[0]
		}
		if nextNode == nil {
			continue
		}

		nodesOrig := k.newNodeArray(nextNode)
		ns := k.searchLeafNode(p, nodesOrig)
		id2, dsq2 := k.nearestImpl(p, ns, dsq)
		k.poolNodeArray.Put(nodesOrig)
		if id2 >= 0 {
			id, dsq = id2, dsq2
		}
	}
	return id, dsq
}

func (k *KDTree) searchLeafNode(p mat.Vec3, base []*node) []*node {
	i := len(base) - 1
	parent := base[i]
	pivotVal, val := k.Vec3At(parent.id)[parent.dim], p[parent.dim]

	child0, child1 := parent.children[0], parent.children[1]
	switch {
	case child0 == nil && child1 == nil:
		return base
	case child0 == nil:
		base = append(base, child1)
	case child1 == nil:
		base = append(base, child0)
	case pivotVal > val:
		base = append(base, child0)
	default:
		base = append(base, child1)
	}
	return k.searchLeafNode(p, base)
}

func newNode(ra pc.Vec3RandomAccessor, indice []int, depth int) *node {
	is := &indiceSorter{
		ra:     ra,
		indice: indice,
		dim:    depth % 3,
	}
	sort.Sort(is)
	mid := len(is.indice) / 2
	med := is.indice[mid]

	var left, right *node
	if mid > 0 {
		left = newNode(ra, indice[:mid], depth+1)
	}
	if mid+1 < len(indice) {
		right = newNode(ra, indice[mid+1:], depth+1)
	}
	return &node{
		children: [2]*node{left, right},
		id:       med,
		dim:      is.dim,
	}
}

func (n *node) String() string {
	return n.stringImpl(0)
}

func (n *node) stringImpl(depth int) string {
	if n == nil {
		return strings.Repeat(" ", depth) + "nil"
	}
	return strings.Repeat(" ", depth) + fmt.Sprintf("%d", n.id) + "\n" +
		n.children[0].stringImpl(depth+1) + "\n" +
		n.children[1].stringImpl(depth+1)
}

func (n *node) maxDepth(depth int) int {
	if n == nil {
		return depth
	}
	d0 := n.children[0].maxDepth(depth + 1)
	d1 := n.children[1].maxDepth(depth + 1)
	if d0 > d1 {
		return d1
	}
	return d0
}

type indiceSorter struct {
	ra     pc.Vec3RandomAccessor
	indice []int
	dim    int
}

func (s *indiceSorter) Len() int {
	return len(s.indice)
}

func (s *indiceSorter) Less(i, j int) bool {
	return s.ra.Vec3At(s.indice[i])[s.dim] < s.ra.Vec3At(s.indice[j])[s.dim]
}

func (s *indiceSorter) Swap(i, j int) {
	s.indice[i], s.indice[j] = s.indice[j], s.indice[i]
}