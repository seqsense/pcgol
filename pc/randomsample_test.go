package pc

import (
	"testing"
)

func TestVec3RandomSampleIterator(t *testing.T) {
	testCases := map[string]struct {
		n         int
		ratio     float32
		expected  int
		tolerance int
	}{
		"10000x0.0": {
			n:         10000,
			ratio:     0.0,
			expected:  0,
			tolerance: 0,
		},
		"10000x0.1": {
			n:         10000,
			ratio:     0.1,
			expected:  1000,
			tolerance: 150,
		},
		"10000x0.5": {
			n:         10000,
			ratio:     0.5,
			expected:  5000,
			tolerance: 150,
		},
		"10000x0.75": {
			n:         10000,
			ratio:     0.75,
			expected:  7500,
			tolerance: 150,
		},
		"10000x0.9": {
			n:         10000,
			ratio:     0.9,
			expected:  9000,
			tolerance: 150,
		},
		"10000x1.0": {
			n:         10000,
			ratio:     1.0,
			expected:  10000,
			tolerance: 0,
		},
		"100000x0.01": {
			n:         100000,
			ratio:     0.01,
			expected:  1000,
			tolerance: 500,
		},
		"100000x0.1": {
			n:         100000,
			ratio:     0.1,
			expected:  10000,
			tolerance: 500,
		},
		"100000x0.5": {
			n:         100000,
			ratio:     0.5,
			expected:  50000,
			tolerance: 500,
		},
	}
	for name, tt := range testCases {
		tt := tt
		t.Run(name, func(t *testing.T) {
			orig := make(Vec3Slice, tt.n)
			for i := range orig {
				orig[i][0] = float32(i)
			}

			var sampled Vec3Slice
			it := NewVec3RandomSampleIterator(orig, tt.ratio)
			for ; it.IsValid(); it.Incr() {
				sampled = append(sampled, it.Vec3())
			}

			n := len(sampled)
			if n < tt.expected-tt.tolerance || tt.expected+tt.tolerance < n {
				t.Fatalf("Expected %d samples, got %d", tt.expected, n)
			}

			count := make(map[int]int, n)
			for _, v := range sampled {
				count[int(v[0])]++
				if v[0] < 0 || float32(tt.n) <= v[0] || v[1] != 0 || v[2] != 0 {
					t.Fatalf("Nonexistent point is sampled: %v", v)
				}
			}
			for _, c := range count {
				if c > 1 {
					t.Fatal("Duplicated sample")
				}
			}
		})
	}

}
