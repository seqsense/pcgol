## Point cloud library for Go (independent of PCL)

## Overview

This package implements algorithms to process point cloud data on Go.
It's not a clone or porting of [PCL](https://pointclouds.org/).

- ./mat
    > Linear algebra library for point cloud processing
- ./pc
    > Binary point cloud data marshaller/unmarshaller and iterators
  - ./pc/filter
      > Point cloud filters like VoxelGrid downsampling filter
  - ./pc/storage
      > Storage to handle spacial structure of point cloud data. Mainly used to implement other subpackages
  - ./pc/segmentation
      > Point cloud segmentation algorithms
  - ./pc/sac
      > Sample consensus based model parameter estimators

## License

This package is licensed under [Apache License Version 2.0](./LICENSE).
