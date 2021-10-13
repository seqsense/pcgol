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

func (k *KDTree) findMinimumImpl(n *node, dim int, depth int) (*node, error) {
	if dim > 2 {
		return nil, fmt.Errorf("dim should be <3")
	}

	if n == nil {
		return nil, nil
	}

	minNode := func(d int, ns ...*node) *node {
		min := ns[0]
		for _, n := range ns {
			if n == nil {
				continue
			}
			if k.Vec3At(min.id)[d] > k.Vec3At(n.id)[d] {
				min = n
			}
		}
		return min
	}

	if n.dim == dim {
		if n.children[0] == nil {
			return n, nil
		}
		min, err := k.findMinimumImpl(n.children[0], dim, depth+1)
		if err != nil {
			return nil, err
		}
		return min, nil
	}

	min0, err := k.findMinimumImpl(n.children[0], dim, depth+1)
	if err != nil {
		return nil, err
	}
	min1, err := k.findMinimumImpl(n.children[1], dim, depth+1)
	if err != nil {
		return nil, err
	}
	return minNode(dim, n, min0, min1), nil
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
			minNode, err := k.findMinimumImpl(n.children[1], n.dim, 0)
			if err != nil {
				return nil, err
			}
			n.id = minNode.id
			children1, err := k.deleteNodeImpl(n.children[1], minNode.id, depth+1)
			if err != nil {
				return nil, err
			}
			n.children[1] = children1
		} else if n.children[0] != nil {
			minNode, err := k.findMinimumImpl(n.children[0], n.dim, 0)
			if err != nil {
				return nil, err
			}
			n.id = minNode.id
			children0, err := k.deleteNodeImpl(n.children[0], minNode.id, depth+1)
			if err != nil {
				return nil, err
			}
			n.children[0] = children0
		} else {
			return nil, nil
		}
		return n, nil
	}

	pointAtNode := k.Vec3At(n.id)
	p := k.Vec3At(pID)
	if p[n.dim] < pointAtNode[n.dim] {
		children0, err := k.deleteNodeImpl(n.children[0], pID, depth+1)
		if err != nil {
			return nil, err
		}
		n.children[0] = children0
	} else {
		children1, err := k.deleteNodeImpl(n.children[1], pID, depth+1)
		if err != nil {
			return nil, err
		}
		n.children[1] = children1
	}
	return n, nil
}

func (k *KDTree) DeletePoint(pID int) error {
	if pID < 0 || pID > k.Len()-1 {
		return fmt.Errorf("%d does not correspond to any point in the tree", pID)
	}
	_, err := k.deleteNodeImpl(k.root, pID, 0)
	return err
}

func equal(n1, n2 *node, it1, it2 pc.Vec3RandomAccessor) bool {
	if n1 == nil && n2 == nil {
		return true
	}
	if n1 == nil && n2 != nil {
		return false
	}
	if n2 == nil && n1 != nil {
		return false
	}
	if n1.id != n2.id || n1.dim != n2.dim || !it1.Vec3At(n1.id).Equal(it2.Vec3At(n2.id)) {
		return false
	}
	return equal(n1.children[0], n2.children[0], it1, it2) &&
		equal(n1.children[1], n2.children[1], it1, it2)
}

func (k *KDTree) Equal(k2 *KDTree) bool {
	return equal(k.root, k2.root, k, k2)
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
