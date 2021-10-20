package pc

import (
	"encoding/binary"
	"math"

	"github.com/seqsense/pcgol/mat"
)

type binaryIterator struct {
	data   []byte
	pos    int
	stride int
}

func (i *binaryIterator) Incr() {
	i.pos += i.stride
}

func (i *binaryIterator) Len() int {
	return len(i.data) / i.stride
}

type Float32Iterator interface {
	Incr()
	IsValid() bool
	Float32() float32
	SetFloat32(float32)
	Float32At(int) float32
	Len() int
}

type Vec3Iterator interface {
	Vec3RandomAccessor
	Vec3ForwardIterator
}

type Vec3ForwardIterator interface {
	Vec3ConstForwardIterator
	SetVec3(mat.Vec3)
}

type Vec3ConstForwardIterator interface {
	Incr()
	IsValid() bool
	Vec3() mat.Vec3
}

type binaryFloat32Iterator struct {
	binaryIterator
}

func (i *binaryFloat32Iterator) Float32() float32 {
	return math.Float32frombits(
		binary.LittleEndian.Uint32(i.binaryIterator.data[i.binaryIterator.pos : i.binaryIterator.pos+4]),
	)
}

func (i *binaryFloat32Iterator) Float32At(j int) float32 {
	pos := i.binaryIterator.pos + i.stride*j
	return math.Float32frombits(
		binary.LittleEndian.Uint32(i.binaryIterator.data[pos : pos+4]),
	)
}

func (i *binaryFloat32Iterator) SetFloat32(v float32) {
	b := math.Float32bits(v)
	binary.LittleEndian.PutUint32(
		i.binaryIterator.data[i.binaryIterator.pos:i.binaryIterator.pos+4], b,
	)
}

func (i *binaryFloat32Iterator) IsValid() bool {
	return i.pos+4 <= len(i.data)
}

type float32Iterator struct {
	data   []float32
	pos    int
	stride int
}

func (i *float32Iterator) Incr() {
	i.pos += i.stride
}

func (i *float32Iterator) IsValid() bool {
	return i.pos+1 <= len(i.data)
}

func (i *float32Iterator) Len() int {
	return len(i.data) / i.stride
}

func (i *float32Iterator) Float32() float32 {
	return i.data[i.pos]
}

func (i *float32Iterator) Float32At(j int) float32 {
	return i.data[i.pos+i.stride*j]
}

func (i *float32Iterator) SetFloat32(v float32) {
	i.data[i.pos] = v
}

func (i *float32Iterator) Vec3() mat.Vec3 {
	var ret mat.Vec3
	copy(ret[:], i.data[i.pos:i.pos+3])
	return ret
}

func (i *float32Iterator) Vec3At(j int) mat.Vec3 {
	pos := i.pos + i.stride*j
	var ret mat.Vec3
	copy(ret[:], i.data[pos:pos+3])
	return ret
}

func (i *float32Iterator) SetVec3(v mat.Vec3) {
	copy(i.data[i.pos:i.pos+3], v[:])
}

type naiveVec3Iterator [3]Float32Iterator

func (i naiveVec3Iterator) IsValid() bool {
	return i[0].IsValid()
}

func (i naiveVec3Iterator) Len() int {
	return i[0].Len()
}

func (i naiveVec3Iterator) Incr() {
	i[0].Incr()
	i[1].Incr()
	i[2].Incr()
}

func (i naiveVec3Iterator) Vec3() mat.Vec3 {
	return mat.Vec3{i[0].Float32(), i[1].Float32(), i[2].Float32()}
}

func (i naiveVec3Iterator) Vec3At(j int) mat.Vec3 {
	return mat.Vec3{i[0].Float32At(j), i[1].Float32At(j), i[2].Float32At(j)}
}

func (i naiveVec3Iterator) SetVec3(v mat.Vec3) {
	i[0].SetFloat32(v[0])
	i[1].SetFloat32(v[1])
	i[2].SetFloat32(v[2])
}

type Uint32Iterator interface {
	Uint32RandomAccessor
	Incr()
	IsValid() bool
	Uint32() uint32
	SetUint32(uint32)
}

type binaryUint32Iterator struct {
	binaryIterator
}

func (i *binaryUint32Iterator) Uint32() uint32 {
	return binary.LittleEndian.Uint32(i.binaryIterator.data[i.binaryIterator.pos : i.binaryIterator.pos+4])
}

func (i *binaryUint32Iterator) Uint32At(j int) uint32 {
	pos := i.binaryIterator.pos + i.binaryIterator.stride*j
	return binary.LittleEndian.Uint32(i.binaryIterator.data[pos : pos+4])
}

func (i *binaryUint32Iterator) SetUint32(v uint32) {
	binary.LittleEndian.PutUint32(
		i.binaryIterator.data[i.binaryIterator.pos:i.binaryIterator.pos+4], v,
	)
}

func (i *binaryUint32Iterator) IsValid() bool {
	return i.pos+4 <= len(i.data)
}
