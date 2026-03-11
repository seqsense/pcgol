package pc

import (
	"math"
)

// Color represents an RGBA color with 8 bits per channel.
// The bit layout matches the PCL convention: 0xAARRGGBB.
type Color struct {
	R, G, B, A uint8
}

// ColorFromFloat32 unpacks a Color from a float32 using the PCL convention
// where the uint32 bit pattern is 0xAARRGGBB.
func ColorFromFloat32(f float32) Color {
	return ColorFromUint32(math.Float32bits(f))
}

// ColorFromUint32 unpacks a Color from a uint32 with bit pattern 0xAARRGGBB.
func ColorFromUint32(v uint32) Color {
	return Color{
		A: uint8((v >> 24) & 0xFF),
		R: uint8((v >> 16) & 0xFF),
		G: uint8((v >> 8) & 0xFF),
		B: uint8(v & 0xFF),
	}
}

// Float32 packs the Color into a float32 using the PCL convention.
func (c Color) Float32() float32 {
	return math.Float32frombits(c.Uint32())
}

// Uint32 packs the Color into a uint32 with bit pattern 0xAARRGGBB.
func (c Color) Uint32() uint32 {
	return (uint32(c.A) << 24) | (uint32(c.R) << 16) | (uint32(c.G) << 8) | uint32(c.B)
}
