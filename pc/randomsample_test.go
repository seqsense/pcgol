package pc

import (
	"testing"
)

func TestVec3RandomSampleIterator(t *testing.T) {
	testCases := map[string]struct {
		n        int
		ratio    float32
		expected int
	}{
		"10000x0.1": {
			n:        10000,
			ratio:    0.1,
			expected: 1000,
		},
		"10000x0.5": {
			n:        10000,
			ratio:    0.5,
			expected: 5000,
		},
		"10000x0.75": {
			n:        10000,
			ratio:    0.75,
			expected: 7500,
		},
		"10000x0.9": {
			n:        10000,
			ratio:    0.9,
			expected: 9000,
		},
		"100000x0.01": {
			n:        100000,
			ratio:    0.01,
			expected: 1000,
		},
		"100000x0.1": {
			n:        100000,
			ratio:    0.1,
			expected: 10000,
		},
		"100000x0.5": {
			n:        100000,
			ratio:    0.5,
			expected: 50000,
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

			tolerance := tt.n / 100
			n := len(sampled)
			if n < tt.expected-tolerance || tt.expected+tolerance < n {
				t.Fatalf("Expected %d samples, got %d", tt.expected, n)
			}

			count := make(map[int]int, n)
			for _, v := range sampled {
				count[int(v[0])]++
			}
			for _, c := range count {
				if c > 1 {
					t.Fatal("Duplicated sample")
				}
			}
		})
	}
}
