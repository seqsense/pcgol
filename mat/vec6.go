package mat

type Vec6 [6]float32

func (v Vec6) Equal(a Vec6) bool {
	return a[0] == v[0] && a[1] == v[1] && a[2] == v[2] &&
		a[3] == v[3] && a[4] == v[4] && a[5] == v[5]
}
