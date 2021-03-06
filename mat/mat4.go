package mat

import (
	"fmt"
	"strings"
)

// Mat4 represents 4x4 matrix stored by (column * 4 + row) index.
// (Column is stored first to optimize memory access when transforming vector.)
type Mat4 [16]float32

func (m Mat4) Floats() [16]float32 {
	return m
}

func (m Mat4) Mul(a Mat4) Mat4 {
	var out Mat4
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			var sum float32
			for k := 0; k < 4; k++ {
				sum += m[4*k+i] * a[4*j+k]
			}
			out[4*j+i] = sum
		}
	}
	return out
}

func (m Mat4) Factor(f float32) Mat4 {
	var out Mat4
	for i := range m {
		out[i] = m[i] * f
	}
	return out
}

func (m Mat4) Add(a Mat4) Mat4 {
	var out Mat4
	for i := range m {
		out[i] = m[i] + a[i]
	}
	return out
}

func (m Mat4) MulAffine(a Mat4) Mat4 {
	var out Mat4

	out[4*0+0] = m[4*0+0]*a[4*0+0] +
		m[4*1+0]*a[4*0+1] +
		m[4*2+0]*a[4*0+2]
	out[4*1+0] = m[4*0+0]*a[4*1+0] +
		m[4*1+0]*a[4*1+1] +
		m[4*2+0]*a[4*1+2]
	out[4*2+0] = m[4*0+0]*a[4*2+0] +
		m[4*1+0]*a[4*2+1] +
		m[4*2+0]*a[4*2+2]
	out[4*3+0] = m[4*0+0]*a[4*3+0] +
		m[4*1+0]*a[4*3+1] +
		m[4*2+0]*a[4*3+2] +
		m[4*3+0]

	out[4*0+1] = m[4*0+1]*a[4*0+0] +
		m[4*1+1]*a[4*0+1] +
		m[4*2+1]*a[4*0+2]
	out[4*1+1] = m[4*0+1]*a[4*1+0] +
		m[4*1+1]*a[4*1+1] +
		m[4*2+1]*a[4*1+2]
	out[4*2+1] = m[4*0+1]*a[4*2+0] +
		m[4*1+1]*a[4*2+1] +
		m[4*2+1]*a[4*2+2]
	out[4*3+1] = m[4*0+1]*a[4*3+0] +
		m[4*1+1]*a[4*3+1] +
		m[4*2+1]*a[4*3+2] +
		m[4*3+1]

	out[4*0+2] = m[4*0+2]*a[4*0+0] +
		m[4*1+2]*a[4*0+1] +
		m[4*2+2]*a[4*0+2]
	out[4*1+2] = m[4*0+2]*a[4*1+0] +
		m[4*1+2]*a[4*1+1] +
		m[4*2+2]*a[4*1+2]
	out[4*2+2] = m[4*0+2]*a[4*2+0] +
		m[4*1+2]*a[4*2+1] +
		m[4*2+2]*a[4*2+2]
	out[4*3+2] = m[4*0+2]*a[4*3+0] +
		m[4*1+2]*a[4*3+1] +
		m[4*2+2]*a[4*3+2] +
		m[4*3+2]

	out[4*3+3] = 1

	return out
}

func (m Mat4) InvAffine() Mat4 {
	var out Mat4
	normInv := 1 / (m[4*0+0]*m[4*1+1]*m[4*2+2] +
		m[4*0+1]*m[4*1+2]*m[4*2+0] +
		m[4*0+2]*m[4*1+0]*m[4*2+1] -
		m[4*0+2]*m[4*1+1]*m[4*2+0] -
		m[4*0+1]*m[4*1+0]*m[4*2+2] -
		m[4*0+0]*m[4*1+2]*m[4*2+1])

	out[4*0+0] = (m[4*1+1]*m[4*2+2] - m[4*1+2]*m[4*2+1]) * normInv
	out[4*0+1] = -(m[4*0+1]*m[4*2+2] - m[4*0+2]*m[4*2+1]) * normInv
	out[4*0+2] = (m[4*0+1]*m[4*1+2] - m[4*0+2]*m[4*1+1]) * normInv
	out[4*1+0] = -(m[4*1+0]*m[4*2+2] - m[4*1+2]*m[4*2+0]) * normInv
	out[4*1+1] = (m[4*0+0]*m[4*2+2] - m[4*0+2]*m[4*2+0]) * normInv
	out[4*1+2] = -(m[4*0+0]*m[4*1+2] - m[4*0+2]*m[4*1+0]) * normInv
	out[4*2+0] = (m[4*1+0]*m[4*2+1] - m[4*1+1]*m[4*2+0]) * normInv
	out[4*2+1] = -(m[4*0+0]*m[4*2+1] - m[4*0+1]*m[4*2+0]) * normInv
	out[4*2+2] = (m[4*0+0]*m[4*1+1] - m[4*0+1]*m[4*1+0]) * normInv
	out[4*3+3] = 1
	b2 := out.Transform(NewVec3(m[4*3+0], m[4*3+1], m[4*3+2]))
	out[4*3+0] = -b2[0]
	out[4*3+1] = -b2[1]
	out[4*3+2] = -b2[2]
	return out
}

func (m Mat4) TransformAffine(a Vec3) Vec3 {
	return Vec3{
		m[4*0+0]*a[0] + m[4*1+0]*a[1] + m[4*2+0]*a[2] + m[4*3+0],
		m[4*0+1]*a[0] + m[4*1+1]*a[1] + m[4*2+1]*a[2] + m[4*3+1],
		m[4*0+2]*a[0] + m[4*1+2]*a[1] + m[4*2+2]*a[2] + m[4*3+2],
	}
}

func (m Mat4) Transform(a Vec3) Vec3 {
	w := 1 / (m[4*0+3]*a[0] + m[4*1+3]*a[1] + m[4*2+3]*a[2] + m[4*3+3])
	return Vec3{
		(m[4*0+0]*a[0] + m[4*1+0]*a[1] + m[4*2+0]*a[2] + m[4*3+0]) * w,
		(m[4*0+1]*a[0] + m[4*1+1]*a[1] + m[4*2+1]*a[2] + m[4*3+1]) * w,
		(m[4*0+2]*a[0] + m[4*1+2]*a[1] + m[4*2+2]*a[2] + m[4*3+2]) * w,
	}
}

func (m Mat4) TransformAffineX(a Vec3) float32 {
	return m[4*0+0]*a[0] + m[4*1+0]*a[1] + m[4*2+0]*a[2] + m[4*3+0]
}

func (m Mat4) TransformAffineY(a Vec3) float32 {
	return m[4*0+1]*a[0] + m[4*1+1]*a[1] + m[4*2+1]*a[2] + m[4*3+1]
}

func (m Mat4) TransformAffineZ(a Vec3) float32 {
	return m[4*0+2]*a[0] + m[4*1+2]*a[1] + m[4*2+2]*a[2] + m[4*3+2]
}

func (m Mat4) Det() float32 {
	a11, a12, a13, a14 := m[4*0+0], m[4*0+1], m[4*0+2], m[4*0+3]
	a21, a22, a23, a24 := m[4*1+0], m[4*1+1], m[4*1+2], m[4*1+3]
	a31, a32, a33, a34 := m[4*2+0], m[4*2+1], m[4*2+2], m[4*2+3]
	a41, a42, a43, a44 := m[4*3+0], m[4*3+1], m[4*3+2], m[4*3+3]
	return a11*a22*a33*a44 + a11*a23*a34*a42 + a11*a24*a32*a43 -
		a11*a24*a33*a42 - a11*a23*a32*a44 - a11*a22*a34*a43 -
		a12*a21*a33*a44 - a13*a21*a34*a42 - a14*a21*a32*a43 +
		a14*a21*a33*a42 + a13*a21*a32*a44 + a12*a21*a34*a43 +
		a12*a23*a31*a44 + a13*a24*a31*a42 + a14*a22*a31*a43 -
		a14*a23*a31*a42 - a13*a22*a31*a44 - a12*a24*a31*a43 -
		a12*a23*a34*a41 - a13*a24*a32*a41 - a14*a22*a33*a41 +
		a14*a23*a32*a41 + a13*a22*a34*a41 + a12*a24*a33*a41
}

func (m Mat4) Inv() Mat4 {
	a11, a12, a13, a14 := m[4*0+0], m[4*0+1], m[4*0+2], m[4*0+3]
	a21, a22, a23, a24 := m[4*1+0], m[4*1+1], m[4*1+2], m[4*1+3]
	a31, a32, a33, a34 := m[4*2+0], m[4*2+1], m[4*2+2], m[4*2+3]
	a41, a42, a43, a44 := m[4*3+0], m[4*3+1], m[4*3+2], m[4*3+3]

	var out Mat4
	out[4*0+0] = a22*a33*a44 + a23*a34*a42 + a24*a32*a43 -
		a24*a33*a42 - a23*a32*a44 - a22*a34*a43
	out[4*0+1] = -a12*a33*a44 - a13*a34*a42 - a14*a32*a43 +
		a14*a33*a42 + a13*a32*a44 + a12*a34*a43
	out[4*0+2] = a12*a23*a44 + a13*a24*a42 + a14*a22*a43 -
		a14*a23*a42 - a13*a22*a44 - a12*a24*a43
	out[4*0+3] = -a12*a23*a34 - a13*a24*a32 - a14*a22*a33 +
		a14*a23*a32 + a13*a22*a34 + a12*a24*a33

	out[4*1+0] = -a21*a33*a44 - a23*a34*a41 - a24*a31*a43 +
		a24*a33*a41 + a23*a31*a44 + a21*a34*a43
	out[4*1+1] = a11*a33*a44 + a13*a34*a41 + a14*a31*a43 -
		a14*a33*a41 - a13*a31*a44 - a11*a34*a43
	out[4*1+2] = -a11*a23*a44 - a13*a24*a41 - a14*a21*a43 +
		a14*a23*a41 + a13*a21*a44 + a11*a24*a43
	out[4*1+3] = a11*a23*a34 + a13*a24*a31 + a14*a21*a33 -
		a14*a23*a31 - a13*a21*a34 - a11*a24*a33

	out[4*2+0] = a21*a32*a44 + a22*a34*a41 + a24*a31*a42 -
		a24*a32*a41 - a22*a31*a44 - a21*a34*a42
	out[4*2+1] = -a11*a32*a44 - a12*a34*a41 - a14*a31*a42 +
		a14*a32*a41 + a12*a31*a44 + a11*a34*a42
	out[4*2+2] = a11*a22*a44 + a12*a24*a41 + a14*a21*a42 -
		a14*a22*a41 - a12*a21*a44 - a11*a24*a42
	out[4*2+3] = -a11*a22*a34 - a12*a24*a31 - a14*a21*a32 +
		a14*a22*a31 + a12*a21*a34 + a11*a24*a32

	out[4*3+0] = -a21*a32*a43 - a22*a33*a41 - a23*a31*a42 +
		a23*a32*a41 + a22*a31*a43 + a21*a33*a42
	out[4*3+1] = a11*a32*a43 + a12*a33*a41 + a13*a31*a42 -
		a13*a32*a41 - a12*a31*a43 - a11*a33*a42
	out[4*3+2] = -a11*a22*a43 - a12*a23*a41 - a13*a21*a42 +
		a13*a22*a41 + a12*a21*a43 + a11*a23*a42
	out[4*3+3] = a11*a22*a33 + a12*a23*a31 + a13*a21*a32 -
		a13*a22*a31 - a12*a21*a33 - a11*a23*a32

	dinv := 1 / m.Det()
	for i := range out {
		out[i] *= dinv
	}
	return out
}

func (m Mat4) Transpose() Mat4 {
	return Mat4{
		m[4*0+0], m[4*1+0], m[4*2+0], m[4*3+0],
		m[4*0+1], m[4*1+1], m[4*2+1], m[4*3+1],
		m[4*0+2], m[4*1+2], m[4*2+2], m[4*3+2],
		m[4*0+3], m[4*1+3], m[4*2+3], m[4*3+3],
	}
}

func (m Mat4) String() string {
	out := make([]string, 4)
	for j := 0; j < 4; j++ {
		out[j] = fmt.Sprintf("[%0.3f %0.3f %0.3f %0.3f]",
			m[j*4+0], m[j*4+1], m[j*4+2], m[j*4+3],
		)
	}
	return "[" + strings.Join(out, " ") + "]"
}
