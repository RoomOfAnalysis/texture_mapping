#

## Texture Mapping

Map image texture onto mesh

### Reference

- [PCL](https://github.com/PointCloudLibrary/pcl/blob/master/gpu/kinfu_large_scale/tools/standalone_texture_mapping.cpp)
- [iory's repo](https://github.com/iory/texture-mapping)

### Camera Matrix

camera pose with intrinsic parameters

```sh
translation_x translation_y translation_z
rotation_00   rotation_01   rotation_02
rotation_10   rotation_11   rotation_12
rotation_20   rotation_21   rotation_22
fx
fy
cx
cy
image_height
image_width
```

### Dependency

- [PCL 1.12.0](https://github.com/PointCloudLibrary/pcl/releases)

### Usage

```sh
// argv[1]: input mesh file
// argv[2]: output obj file
// argv[3]: input texture.png / camera.txt folder 
.\texture_mapping.exe xxx\input.obj xxx\output.obj xxx\camera_matrix_and_texture_folder 
```
