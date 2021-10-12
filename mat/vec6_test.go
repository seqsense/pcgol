package mat

import (
	"testing"
)

func TestVec6_Equal(t *testing.T) {
	t.Run("Equal", func(t *testing.T) {
		a := Vec6{1, 2, 3, 4, 5, 6}
		b := Vec6{1, 2, 3, 4, 5, 6}
		if !a.Equal(b) {
			t.Errorf("%v must be equal to %v", a, b)
		}
	})
	t.Run("NotEqual", func(t *testing.T) {
		a := Vec6{1, 2, 3, 4, 5, 6}
		for i := range a {
			b := Vec6{1, 2, 3, 4, 5, 6}
			b[i] += 0.1
			if a.Equal(b) {
				t.Errorf("%v must not be equal to %v", a, b)
			}
		}
	})
}
