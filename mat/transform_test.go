package mat

import (
	"math"
	"testing"
)

func TestTranslate(t *testing.T) {
	m := Translate(1, 2, 3)
	v := Vec3{4, 5, 6}
	out := m.Transform(v)
	expected := Vec3{5, 7, 9}

	if !out.Equal(expected) {
		t.Errorf("%s transformed by %s is expected to be %s, got %s",
			v, m, expected, out,
		)
	}
}

func TestRotate(t *testing.T) {
	testCases := map[string]struct {
		m        Mat4
		v        Vec3
		expected Vec3
	}{
		"zero,l=1": {
			m:        Rotate(0, 0, 1, 0),
			v:        Vec3{1, 0, 0},
			expected: Vec3{1, 0, 0},
		},
		"zero,l=2": {
			m:        Rotate(0, 0, 1, 0),
			v:        Vec3{2, 0, 0},
			expected: Vec3{2, 0, 0},
		},
		/*
		 * ^y
		 * |   ^yaw
		 * |   |
		 * +---->x
		 */
		"yaw=+45deg,l=2": {
			m:        Rotate(0, 0, 1, math.Pi/4),
			v:        Vec3{2, 0, 0},
			expected: Vec3{math.Sqrt2, math.Sqrt2, 0},
		},
		"yaw=+45deg,l=1,Crossing": {
			m:        Rotate(0, 0, 1, math.Pi/4),
			v:        Vec3{0, 0, 1},
			expected: Vec3{0, 0, 1},
		},
		"yaw=45+45deg,l=1": {
			m:        Rotate(0, 0, 1, math.Pi/4),
			v:        Vec3{math.Sqrt2 / 2, math.Sqrt2 / 2, 0},
			expected: Vec3{0, 1, 0},
		},
		"yaw=+45deg,l=1": {
			m:        Rotate(0, 0, 1, math.Pi/4),
			v:        Vec3{1, 0, 0},
			expected: Vec3{math.Sqrt2 / 2, math.Sqrt2 / 2, 0},
		},
		"yaw=-45deg,l=1": {
			m:        Rotate(0, 0, 1, -math.Pi/4),
			v:        Vec3{1, 0, 0},
			expected: Vec3{math.Sqrt2 / 2, -math.Sqrt2 / 2, 0},
		},
		"yaw=+135deg,l=1": {
			m:        Rotate(0, 0, 1, math.Pi*3/4),
			v:        Vec3{1, 0, 0},
			expected: Vec3{-math.Sqrt2 / 2, math.Sqrt2 / 2, 0},
		},
		"yaw=-135deg,l=1": {
			m:        Rotate(0, 0, 1, -math.Pi*3/4),
			v:        Vec3{1, 0, 0},
			expected: Vec3{-math.Sqrt2 / 2, -math.Sqrt2 / 2, 0},
		},
		/*
		 * ^z
		 * |   ^roll
		 * |   |
		 * +---->y
		 */
		"roll=+45deg,l=1": {
			m:        Rotate(1, 0, 0, math.Pi/4),
			v:        Vec3{0, 1, 0},
			expected: Vec3{0, math.Sqrt2 / 2, math.Sqrt2 / 2},
		},
		"roll=-45deg,l=1": {
			m:        Rotate(1, 0, 0, -math.Pi/4),
			v:        Vec3{0, 1, 0},
			expected: Vec3{0, math.Sqrt2 / 2, -math.Sqrt2 / 2},
		},
		"roll=+135deg,l=1": {
			m:        Rotate(1, 0, 0, math.Pi*3/4),
			v:        Vec3{0, 1, 0},
			expected: Vec3{0, -math.Sqrt2 / 2, math.Sqrt2 / 2},
		},
		"roll=-135deg,l=1": {
			m:        Rotate(1, 0, 0, -math.Pi*3/4),
			v:        Vec3{0, 1, 0},
			expected: Vec3{0, -math.Sqrt2 / 2, -math.Sqrt2 / 2},
		},
		/*
		 * ^x
		 * |   ^pitch
		 * |   |
		 * +---->z
		 */
		"pitch=+45deg,l=1": {
			m:        Rotate(0, 1, 0, math.Pi/4),
			v:        Vec3{0, 0, 1},
			expected: Vec3{math.Sqrt2 / 2, 0, math.Sqrt2 / 2},
		},
		"pitch=-45deg,l=1": {
			m:        Rotate(0, 1, 0, -math.Pi/4),
			v:        Vec3{0, 0, 1},
			expected: Vec3{-math.Sqrt2 / 2, 0, math.Sqrt2 / 2},
		},
		"pitch=+135deg,l=1": {
			m:        Rotate(0, 1, 0, math.Pi*3/4),
			v:        Vec3{0, 0, 1},
			expected: Vec3{math.Sqrt2 / 2, 0, -math.Sqrt2 / 2},
		},
		"pitch=-135deg,l=1": {
			m:        Rotate(0, 1, 0, -math.Pi*3/4),
			v:        Vec3{0, 0, 1},
			expected: Vec3{-math.Sqrt2 / 2, 0, -math.Sqrt2 / 2},
		},
	}

	const tolerance = 1e-6
	nearlyEqual := func(a, b float32) bool {
		return -tolerance < a-b && a-b < tolerance
	}
	if !nearlyEqual(1.0, 1.0000009) {
		t.Fatal("internal: tolerance check")
	}
	if nearlyEqual(1.0, 1.0000011) {
		t.Fatal("internal: tolerance check")
	}
	if !nearlyEqual(1.0, 0.9999991) {
		t.Fatal("internal: tolerance check")
	}
	if nearlyEqual(1.0, 0.999998) {
		t.Fatal("internal: tolerance check")
	}

	for name, tt := range testCases {
		tt := tt
		t.Run(name, func(t *testing.T) {
			out := tt.m.Transform(tt.v)

			if !nearlyEqual(out[0], tt.expected[0]) ||
				!nearlyEqual(out[1], tt.expected[1]) ||
				!nearlyEqual(out[2], tt.expected[2]) {
				t.Errorf("%s transformed by %s is expected to be %s, got %s",
					tt.v, tt.m, tt.expected, out,
				)
			}
		})
	}
}
