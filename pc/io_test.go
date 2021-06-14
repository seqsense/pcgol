package pc

import (
	"fmt"
	"io/ioutil"
	"os"
	"testing"
)

func TestUnmarshalAscii(t *testing.T) {

	points := []float32{
		0.352, -0.151, -0.106, 0,
		-0.473, 0.292, -0.731, 0,
		0.441, -0.734, 0.854, 2,
		-0.46, -0.277, -0.916, 1,
		0.968, 0.512, -0.998, 1,
	}

	header := "# .PCD v0.7 - Point Cloud Data file format\n"
	header += "VERSION 0.7\n"
	header += "FIELDS x y z xyz label\n"
	header += "SIZE 4 4 4 4 4\n"
	header += "TYPE F F F F U\n"
	header += "COUNT 1 1 1 3 1\n"
	header += "WIDTH 5\n"
	header += "HEIGHT 1\n"
	header += "VIEWPOINT 0 0 0 1 0 0 0\n"
	header += "POINTS 5\n"
	header += "DATA ascii\n"

	body := ""
	for i := 0; i < len(points); i += 4 {
		body += fmt.Sprintf("%f %f %f %f %f %f %d\n", points[i], points[i+1], points[i+2], points[i], points[i+1], points[i+2], uint32(points[i+3]))
	}

	fileToWrite, _ := ioutil.TempFile(".", "ascii.*.pcd")
	defer os.Remove(fileToWrite.Name())
	fileToWrite.Write([]byte(header))
	fileToWrite.Write([]byte(body))
	fileToWrite.Close()

	fileToRead, _ := os.Open(fileToWrite.Name())

	pp, err := Unmarshal(fileToRead)
	if err != nil {
		t.Fatal(err)
	}

	vt, err := pp.Vec3Iterator()
	if err != nil {
		t.Fatal(err)
	}

	lt, err := pp.Uint32Iterator("label")
	if err != nil {
		t.Fatal(err)
	}

	for i := 0; i < len(points); i += 4 {
		p := vt.Vec3()
		for j := range p {
			if p[j] != points[i+j] {
				t.Errorf("Point %v, expected coordinate %v: %v, got %v", i, j, points[i+j], p[j])
			}
		}
		l := lt.Uint32()
		if l != uint32(points[i+3]) {
			t.Errorf("Point %v, expected label: %v, got: %v", i, uint32(points[i+3]), l)
		}
		vt.Incr()
		lt.Incr()
	}
}
