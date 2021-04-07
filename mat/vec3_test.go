package mat

import (
	"reflect"
	"testing"
)

func TestVec3_Floats(t *testing.T) {
	v := Vec3{1, 2, 3}
	f := v.Floats()
	expected := [3]float32{1, 2, 3}
	if !reflect.DeepEqual(expected, f) {
		t.Errorf("Expected %v, got %v", expected, f)
	}
}

func TestVec3_Cross(t *testing.T) {
	x := Vec3{1, 0, 0}
	y := Vec3{0, 1, 0}

	c := x.Cross(y)
	if c[0] < -0.01 || 0.01 < c[0] {
		t.Error("Cross()[0] is wrong")
	}
	if c[1] < -0.01 || 0.01 < c[1] {
		t.Error("Cross()[1] is wrong")
	}
	if c[2] < 0.99 || 1.01 < c[2] {
		t.Error("Cross()[2] is wrong")
	}

	cn := x.CrossNormSq(y)
	if cn < 0.99 || 1.01 < cn {
		t.Error("CrossNormSq is wrong")
	}
}
