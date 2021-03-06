package icp

import (
	"testing"

	"github.com/seqsense/pcgol/mat"
	"github.com/seqsense/pcgol/pc"
	"github.com/seqsense/pcgol/pc/storage/kdtree"
)

func TestPointToPointEvaluator(t *testing.T) {
	base := pc.Vec3Slice{
		mat.Vec3{0, 0, 0},
		mat.Vec3{1, 1, 0},
		mat.Vec3{2, 2, 0},
		mat.Vec3{3, 1, 1},
		mat.Vec3{4, 0, 0},
	}
	delta := mat.Vec3{0.25, 0.125, -0.125}
	target := pc.Vec3Slice{
		base[2].Add(delta),
		base[3].Add(delta),
		base[4].Add(delta),
	}
	expected := Evaluated{
		Value: delta.NormSq(),
	}

	kdt := kdtree.New(base)

	e := &PointToPointEvaluator{
		Corresponder: &NearestPointCorresponder{MaxDist: 2},
		MinPairs:     3,
	}
	ev, err := e.Evaluate(kdt, target)
	if err != nil {
		t.Fatal(err)
	}

	if ev.Value != expected.Value {
		t.Errorf("Expected evaluated value: %f, got: %f", expected.Value, ev.Value)
	}
	const factor = -0.1
	dR := rodriguesToRotation(mat.Vec3{
		ev.Gradient[dWx] * factor,
		ev.Gradient[dWy] * factor,
		ev.Gradient[dWz] * factor,
	})
	dT := mat.Vec3{
		ev.Gradient[dX] * factor,
		ev.Gradient[dY] * factor,
		ev.Gradient[dZ] * factor,
	}
	updatedTargetRot := make(pc.Vec3Slice, len(target))
	for i, p := range target {
		updatedTargetRot[i] = dR.Transform(p)
	}
	ev2, err := e.Evaluate(kdt, updatedTargetRot)
	if err != nil {
		t.Fatal(err)
	}
	if ev2.Value >= ev.Value {
		t.Fatalf("Evaluated value is not decreased by rotation: %f !< %f", ev2.Value, ev.Value)
	}

	updatedTargetTrans := make(pc.Vec3Slice, len(target))
	for i, p := range target {
		updatedTargetTrans[i] = p.Add(dT)
	}
	ev3, err := e.Evaluate(kdt, updatedTargetTrans)
	if err != nil {
		t.Fatal(err)
	}
	if ev3.Value >= ev.Value {
		t.Fatalf("Evaluated value is not decreased by translation: %f !< %f", ev3.Value, ev.Value)
	}
}
