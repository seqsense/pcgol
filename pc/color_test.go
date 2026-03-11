package pc

import (
	"testing"
)

func TestColorPackUnpack(t *testing.T) {
	testCases := []Color{
		{R: 255, G: 0, B: 0, A: 0},
		{R: 0, G: 255, B: 0, A: 0},
		{R: 0, G: 0, B: 255, A: 0},
		{R: 128, G: 64, B: 32, A: 0},
		{R: 255, G: 255, B: 255, A: 255},
		{R: 0, G: 0, B: 0, A: 0},
		{R: 1, G: 2, B: 3, A: 4},
	}

	for _, c := range testCases {
		t.Run("Uint32", func(t *testing.T) {
			v := c.Uint32()
			got := ColorFromUint32(v)
			if got != c {
				t.Errorf("Uint32 round-trip: expected %v, got %v", c, got)
			}
		})
		t.Run("Float32", func(t *testing.T) {
			f := c.Float32()
			got := ColorFromFloat32(f)
			if got != c {
				t.Errorf("Float32 round-trip: expected %v, got %v", c, got)
			}
		})
	}
}

func TestColorUint32Layout(t *testing.T) {
	c := Color{R: 0x12, G: 0x34, B: 0x56, A: 0x78}
	got := c.Uint32()
	expected := uint32(0x78123456)
	if got != expected {
		t.Errorf("Expected 0x%08X, got 0x%08X", expected, got)
	}
}
