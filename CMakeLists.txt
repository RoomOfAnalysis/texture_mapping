cmake_minimum_required(VERSION 3.6 FATAL_ERROR)
project(texture_mapping)

if (POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

find_package(PCL REQUIRED COMPONENTS
  common
  features
  io
  kdtree
  search

  visualization
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#add_compile_options(${PCL_COMPILE_OPTIONS})
add_compile_options(/arch:AVX)  # issue: https://github.com/PointCloudLibrary/pcl/issues/2147

add_executable(texture_mapping texture_mapping.cpp)

target_link_libraries(texture_mapping
  ${PCL_LIBRARIES}

  # ${Boost_FILESYSTEM_LIBRARY}
  # ${Boost_SYSTEM_LIBRARY}
  # ${VTK_LIBRARIES}
  # FLANN::FLANN
  # pcl_common
  # pcl_features
  # pcl_io
  # pcl_kdtree
  pcl_surface
  # pcl_visualization
)

add_executable(without_vtk without_vtk.cpp)

target_link_libraries(without_vtk
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  FLANN::FLANN
  pcl_common
  pcl_features
  pcl_io
  pcl_kdtree
  pcl_search
  pcl_surface
)

add_executable(with_vtk with_vtk.cpp)

target_link_libraries(with_vtk
  ${PCL_LIBRARIES}
  pcl_surface
)
