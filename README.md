## Point cloud library for Go (independent of PCL)

[![ci](https://github.com/seqsense/pcgol/actions/workflows/ci.yml/badge.svg)](https://github.com/seqsense/pcgol/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/seqsense/pcgol/branch/master/graph/badge.svg?token=tBu1O2VcOR)](https://codecov.io/gh/seqsense/pcgol)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview

This package implements algorithms to process point cloud data on Go.
It's not a clone or porting of [PCL](https://pointclouds.org/).

## Package structure

<dl>
  <dt>mat</dt><dd>Linear algebra library for point cloud processing</dd>
  <dt>pc</dt><dd>Binary point cloud data marshaller/unmarshaller and iterators
    <dl>
      <dt>pc/filter</dt><dd>Point cloud filters like VoxelGrid downsampling filter</dd>
      <dt>pc/storage</dt><dd>Storage to handle spacial structure of point cloud data</dd>
      <dt>pc/segmentation</dt><dd>Point cloud segmentation algorithms</dd>
      <dt>pc/sac</dt><dd>Sample consensus based model parameter estimators</dd>
    </dl>
  <dd>
</dl>

## License

This package is licensed under [Apache License Version 2.0](./LICENSE).
