package kdtree

import (
	"fmt"
	"sort"
	"strings"
	"sync"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/storage"
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

func (k *KDTree) Nearest(p mat.Vec3, maxRange float32) storage.Neighbor {
	if k.root == nil {
		return storage.Neighbor{ID: -1, DistSq: maxRange * maxRange}
	}
	nodesOrig := k.newNodeArray(k.root)
	defer k.poolNodeArray.Put(nodesOrig)

	nodes := k.searchLeafNode(p, nodesOrig)
	return k.nearestImpl(p, nodes, maxRange*maxRange)
}

func (k *KDTree) nearestImpl(p mat.Vec3, nodes []*node, maxRangeSq float32) storage.Neighbor {
	i := len(nodes) - 1
	neighbor1 := storage.Neighbor{
		ID:     nodes[i].id,
		DistSq: (k.Vec3At(nodes[i].id).Sub(p)).NormSq(),
	}
	if neighbor1.DistSq > maxRangeSq {
		neighbor1.ID = -1
		neighbor1.DistSq = maxRangeSq
	}
	for j := i - 1; j >= 0; j-- {
		pivot := k.Vec3At(nodes[j].id)
		dim := nodes[j].dim
		fromPivot := p[dim] - pivot[dim]
		fromPivotSq := fromPivot * fromPivot
		if fromPivotSq > neighbor1.DistSq {
			continue
		}
		dsqPivot := (pivot.Sub(p)).NormSq()
		if dsqPivot < neighbor1.DistSq {
			neighbor1.ID = nodes[j].id
			neighbor1.DistSq = dsqPivot
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
		neighbor2 := k.nearestImpl(p, ns, neighbor1.DistSq)
		k.poolNodeArray.Put(nodesOrig)
		if neighbor2.ID >= 0 {
			neighbor1.ID = neighbor2.ID
			neighbor1.DistSq = neighbor2.DistSq
		}
	}
	return neighbor1
}

type neighborSorter struct {
	neighbors []storage.Neighbor
}

func (ns *neighborSorter) Len() int {
	return len(ns.neighbors)
}

func (ns *neighborSorter) Swap(i, j int) {
	ns.neighbors[i], ns.neighbors[j] = ns.neighbors[j], ns.neighbors[i]
}

func (ns *neighborSorter) Less(i, j int) bool {
	return ns.neighbors[i].DistSq < ns.neighbors[j].DistSq
}

func (k *KDTree) Range(p mat.Vec3, maxRange float32) []storage.Neighbor {
	neighbors := []storage.Neighbor{}
	if k.root == nil {
		return neighbors
	}
	nodesOrig := k.newNodeArray(k.root)
	defer k.poolNodeArray.Put(nodesOrig)

	nodes := k.searchLeafNode(p, nodesOrig)
	k.rangeImpl(p, nodes, maxRange*maxRange, &neighbors)

	sort.Sort(&neighborSorter{neighbors: neighbors})
	return neighbors
}

func (k *KDTree) rangeImpl(p mat.Vec3, nodes []*node, maxRangeSq float32, neighbors *[]storage.Neighbor) {
	i := len(nodes) - 1
	id := nodes[i].id
	dsq := (k.Vec3At(id).Sub(p)).NormSq()
	if dsq < maxRangeSq {
		*neighbors = append(*neighbors, storage.Neighbor{ID: id, DistSq: dsq})
	}
	for j := i - 1; j >= 0; j-- {
		pivot := k.Vec3At(nodes[j].id)
		dim := nodes[j].dim
		fromPivot := p[dim] - pivot[dim]
		fromPivotSq := fromPivot * fromPivot
		if fromPivotSq > maxRangeSq {
			continue
		}
		dsqPivot := (pivot.Sub(p)).NormSq()
		if dsqPivot < maxRangeSq {
			*neighbors = append(*neighbors, storage.Neighbor{ID: nodes[j].id, DistSq: dsqPivot})
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
		k.rangeImpl(p, ns, maxRangeSq, neighbors)
		k.poolNodeArray.Put(nodesOrig)
	}
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

func (k *KDTree) findMinimumImpl(n *node, dim int, depth int) (int, error) {
	if dim > 2 {
		return -1, fmt.Errorf("dim should be <3")
	}

	if n == nil {
		return -1, nil
	}

	minNode := func(d int, nID1, nID2, nID3 int) int {
		min := nID1
		if nID2 != -1 && k.Vec3At(nID2)[d] < k.Vec3At(min)[d] {
			min = nID2
		}
		if nID3 != -1 && k.Vec3At(nID3)[d] < k.Vec3At(min)[d] {
			min = nID3
		}
		return min
	}

	if n.dim == dim {
		if n.children[0] == nil {
			return n.id, nil
		}
		min, err := k.findMinimumImpl(n.children[0], dim, depth+1)
		if err != nil {
			return -1, err
		}
		return min, nil
	}

	min0, err := k.findMinimumImpl(n.children[0], dim, depth+1)
	if err != nil {
		return -1, err
	}
	min1, err := k.findMinimumImpl(n.children[1], dim, depth+1)
	if err != nil {
		return -1, err
	}
	return minNode(dim, n.id, min0, min1), nil
}

func (k *KDTree) stringImpl(n *node, depth int) string {
	if n != nil {
		s := k.stringImpl(n.children[1], depth+1)
		s += fmt.Sprintf(strings.Repeat(" ", 10*depth)+"-> (%d,%d) %v\n", n.id, n.dim, k.Vec3At(n.id))
		s += k.stringImpl(n.children[0], depth+1)
		return s
	}
	return ""
}

func (k *KDTree) String() string {
	return k.stringImpl(k.root, 0)
}

func (k *KDTree) deleteNodeImpl(n *node, pID int, depth int) (*node, error) {
	if n == nil {
		return nil, nil
	}

	if pID == n.id {
		if n.children[1] != nil {
			minNodeID, err := k.findMinimumImpl(n.children[1], n.dim, depth)
			if err != nil {
				return nil, err
			}
			child, err := k.deleteNodeImpl(n.children[1], minNodeID, depth+1)
			if err != nil {
				return nil, err
			}
			n.id = minNodeID
			n.children[1] = child
		} else if n.children[0] != nil {
			minNodeID, err := k.findMinimumImpl(n.children[0], n.dim, depth)
			if err != nil {
				return nil, err
			}
			child, err := k.deleteNodeImpl(n.children[0], minNodeID, depth+1)
			if err != nil {
				return nil, err
			}
			n.id = minNodeID
			n.children[0] = nil
			n.children[1] = child
		} else {
			return nil, nil
		}
		return n, nil
	}

	pointAtNode := k.Vec3At(n.id)
	p := k.Vec3At(pID)
	if p[n.dim] < pointAtNode[n.dim] {
		child, err := k.deleteNodeImpl(n.children[0], pID, depth+1)
		if err != nil {
			return nil, err
		}
		n.children[0] = child
	} else {
		child, err := k.deleteNodeImpl(n.children[1], pID, depth+1)
		if err != nil {
			return nil, err
		}
		n.children[1] = child
	}
	return n, nil
}

func (k *KDTree) DeletePoint(pID int) error {
	if pID < 0 || pID > k.Len()-1 {
		return fmt.Errorf("%d does not correspond to any point in the tree", pID)
	}
	n, err := k.deleteNodeImpl(k.root, pID, 0)
	if err != nil {
		return err
	}
	k.root = n
	return nil
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
