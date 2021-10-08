package mat

type Mat6 [36]float32

func (m Mat6) At(u, v int) float32 {
	return m[v*6+u]
}

func (m Mat6) SetAt(u, v int, val float32) {
	m[v*6+u] = val
}
