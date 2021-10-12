package mat

import (
	"fmt"
	"reflect"
	"testing"
)

func TestMat4_Floats(t *testing.T) {
	m := Mat4{
		1, 2, 3, 4,
		5, 6, 7, 8,
		9, 10, 11, 12,
		13, 14, 15, 16,
	}
	f := m.Floats()
	expected := [16]float32{
		1, 2, 3, 4,
		5, 6, 7, 8,
		9, 10, 11, 12,
		13, 14, 15, 16,
	}
	if !reflect.DeepEqual(expected, f) {
		t.Errorf("Expected %v, got %v", expected, f)
	}
}

func TestMat4_Mul(t *testing.T) {
	m0 := Translate(0.1, 0.2, 0.3)
	m1 := Scale(1.1, 1.2, 1.3)
	m2 := Rotate(1, 0, 0, 0.1)
	m3 := Rotate(0, 1, 0, 0.1)
	m4 := Rotate(0, 0, 1, 0.1)

	r := m0.MulAffine(m1).MulAffine(m2).MulAffine(m3).MulAffine(m4)
	rNaive := m0.Mul(m1).Mul(m2).Mul(m3).Mul(m4)

	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			a := j*4 + i
			diff := r[a] - rNaive[a]
			if diff < -0.01 || 0.01 < diff {
				t.Errorf("m(%d, %d) expected to be %0.3f, got %0.3f",
					i, j, rNaive[a], r[a],
				)
			}
		}
	}
}

func TestMat4_Factor(t *testing.T) {
	m := Mat4{
		0, 1, 2, 3,
		4, 5, 6, 7,
		8, 9, 10, 11,
		12, 13, 14, 15,
	}
	out := m.Factor(2)
	expected := Mat4{
		0, 2, 4, 6,
		8, 10, 12, 14,
		16, 18, 20, 22,
		24, 26, 28, 30,
	}
	if !reflect.DeepEqual(expected, out) {
		t.Errorf("Expected %v, got %v", expected, out)
	}
}

func TestMat4_Add(t *testing.T) {
	m0 := Mat4{
		0, 1, 2, 3,
		4, 5, 6, 7,
		8, 9, 10, 11,
		12, 13, 14, 15,
	}
	m1 := Mat4{
		2, 4, 6, 8,
		10, 12, 14, 16,
		18, 20, 22, 24,
		26, 28, 30, 32,
	}
	out := m0.Add(m1)
	expected := Mat4{
		2, 5, 8, 11,
		14, 17, 20, 23,
		26, 29, 32, 35,
		38, 41, 44, 47,
	}
	if !reflect.DeepEqual(expected, out) {
		t.Errorf("Expected %v, got %v", expected, out)
	}
}

func TestMat4_InvAffine(t *testing.T) {
	m0 := Translate(0.1, 0.2, 0.3)
	m1 := Scale(1.1, 1.2, 1.3)
	m2 := Rotate(1, 0, 0, 0.5)

	m := m0.MulAffine(m1).MulAffine(m2)
	mi := m.InvAffine()

	diag := m.Mul(mi)
	for i := 0; i < 4; i++ {
		t.Logf("%+0.1f %+0.1f %+0.1f %+0.1f", diag[4*i+0], diag[4*i+1], diag[4*i+2], diag[4*i+3])
		for j := 0; j < 3; j++ {
			if i == j {
				if diag[4*i+j] < 0.99 || 1.01 < diag[4*i+j] {
					t.Errorf("m(%d, %d): %0.3f", i, j, diag[4*i+j])
				}
			} else {
				if diag[4*i+j] < -0.01 || 0.01 < diag[4*i+j] {
					t.Errorf("m(%d, %d): %0.3f", i, j, diag[4*i+j])
				}
			}
		}
	}
}

func transformNaive(m Mat4, a Vec3) Vec3 {
	var out Vec3
	in := [4]float32{a[0], a[1], a[2], 1}
	for i := 0; i < 3; i++ {
		var sum float32
		for k := 0; k < 4; k++ {
			sum += m[4*k+i] * in[k]
		}
		out[i] = sum
	}
	return out
}

func TestMat4_Transform(t *testing.T) {
	m0 := Translate(0.1, 0.2, 0.3)
	m1 := Scale(1.1, 1.2, 1.3)
	m2 := Rotate(1, 0, 0, 0.1)
	m3 := Rotate(0, 1, 0, 0.1)
	m4 := Rotate(0, 0, 1, 0.1)

	m := m0.Mul(m1).Mul(m2).Mul(m3).Mul(m4)

	in := NewVec3(1, 2, 3)
	v := m.Transform(in)
	vNaive := transformNaive(m, in)

	for i := 0; i < 3; i++ {
		diff := v[i] - vNaive[i]
		if diff < -0.01 || 0.01 < diff {
			t.Errorf("v(%d) expected to be %0.3f, got %0.3f",
				i, vNaive[i], v[i],
			)
		}
	}
}

func TestMat4_Det(t *testing.T) {
	m := Mat4{
		1, 1, 1, -1,
		1, 1, -1, 1,
		1, -1, 1, 1,
		-1, 1, 1, 1,
	}
	if d := m.Det(); d != -16.0 {
		t.Errorf("Determinant is wrong %0.3f != %0.3f", d, -16.0)
	}
}

func TestMat4_Inv(t *testing.T) {
	m0 := Translate(0.1, 0.2, 0.3)
	m1 := Scale(1.1, 1.2, 1.3)
	m2 := Rotate(1, 0, 0, 0.5)

	m := m0.MulAffine(m1).MulAffine(m2)
	mia := m.InvAffine()
	mi := m.Inv()
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			a := j*4 + i
			diff := mia[a] - mi[a]
			if diff < -0.01 || 0.01 < diff {
				t.Errorf("m(%d, %d) expected to be %0.3f, got %0.3f",
					i, j, mia[a], mi[a],
				)
			}
		}
	}
}

func TestMat4_Transpose(t *testing.T) {
	m := Mat4{
		1, 2, 3, 4,
		5, 6, 7, 8,
		9, 10, 11, 12,
		13, 14, 15, 16,
	}
	expected := Mat4{
		1, 5, 9, 13,
		2, 6, 10, 14,
		3, 7, 11, 15,
		4, 8, 12, 16,
	}
	trans := m.Transpose()
	if !reflect.DeepEqual(expected, trans) {
		t.Errorf("Expected %v, got %v", expected, trans)
	}
}

func ExampleMat4_String() {
	m := Mat4{
		1, 2, 3, 4,
		5, 6, 7, 8,
		9, 10, 11, 12,
		13, 14, 15, 16,
	}
	fmt.Println(m)
	// Output:
	// [[1.000 2.000 3.000 4.000] [5.000 6.000 7.000 8.000] [9.000 10.000 11.000 12.000] [13.000 14.000 15.000 16.000]]
}
