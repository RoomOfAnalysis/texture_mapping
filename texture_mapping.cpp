/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Raphael Favier, Technical University Eindhoven, (r.mysurname <aT>
 * tue.nl)
 */

#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/common/io.h> // for concatenateFields

#include "read_cam_file.h"
#include "stopwatch.h"

//#define DEBUG_PRINT
//#define DEBUG_PLOT

#ifdef DEBUG_PLOT
#include <pcl/visualization/pcl_visualizer.h>
#endif

// argv[1]: input mesh file
// argv[2]: output obj file
// argv[3]: input texture.png / camera.txt folder 
int main(int argc, char **argv)
{
  pcl::PolygonMesh triangles;
  {
    TIME_BLOCK("- load mesh");
    pcl::io::loadPolygonFile(argv[1], triangles);
  }

#ifdef DEBUG_PLOT
  // viewer
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  viewer.setBackgroundColor(0, 0, 0);
  viewer.addPolygonMesh(triangles, "triangles", 0);
  viewer.addCoordinateSystem(1.0);
  viewer.initCameraParameters();
  viewer.spin();
#endif

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(triangles.cloud, *cloud);

  // Create the texturemesh object that will contain our UV-mapped mesh
  pcl::TextureMesh mesh;
  mesh.cloud = triangles.cloud;
  std::vector<pcl::Vertices> polygon_1;

  // push faces into the texturemesh object
  {
    TIME_BLOCK("- push faces into the texturemesh object");
    polygon_1.resize(triangles.polygons.size());
    for (size_t i = 0; i < triangles.polygons.size(); ++i)
    {
      polygon_1[i] = triangles.polygons[i];
    }
    mesh.tex_polygons.push_back(polygon_1);
  }

  // Load textures and cameras poses and intrinsics
  pcl::texture_mapping::CameraVector my_cams;
  {
    TIME_BLOCK("- load textures and cameras poses and intrinsics");
    const boost::filesystem::path base_dir(argv[3]);
    std::string extension(".txt");
    std::vector<boost::filesystem::path> filenames;
    try
    {
      for (boost::filesystem::directory_iterator it(base_dir);
          it != boost::filesystem::directory_iterator(); ++it)
      {
        if (boost::filesystem::is_regular_file(it->status()) &&
            boost::filesystem::extension(it->path()) == extension)
        {
          filenames.push_back(it->path());
        }
      }
    }
    catch (const boost::filesystem::filesystem_error &e)
    {
      std::cerr << e.what() << std::endl;
    }
    std::sort(filenames.begin(), filenames.end());

    for (int i = 0; i < filenames.size(); ++i)
    {
      std::cout << filenames[i].string() << std::endl;
      pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
      read_cam_pose_file(filenames[i].string(), cam);
      auto texture_path = filenames[i];
      texture_path.replace_extension(".png");
      if (boost::filesystem::exists(texture_path))
          cam.texture_file = texture_path.string();
      else
      {
          texture_path.replace_extension(".jpg");
          if (boost::filesystem::exists(texture_path))
              cam.texture_file = texture_path.string();
          else
              std::cerr << "no corresponding texture img for camera " << texture_path << std::endl;
      }
      my_cams.push_back(cam);
    }
  }

  // Create materials for each texture (and one extra for occluded faces)
  {
    TIME_BLOCK("- create materials for each texture (and one extra for occluded faces)");
    mesh.tex_materials.resize(my_cams.size() + 1);
    for (int i = 0; i <= my_cams.size(); ++i)
    {
      pcl::TexMaterial mesh_material;
      mesh_material.tex_Ka.r = 0.2f;
      mesh_material.tex_Ka.g = 0.2f;
      mesh_material.tex_Ka.b = 0.2f;

      mesh_material.tex_Kd.r = 0.8f;
      mesh_material.tex_Kd.g = 0.8f;
      mesh_material.tex_Kd.b = 0.8f;

      mesh_material.tex_Ks.r = 1.0f;
      mesh_material.tex_Ks.g = 1.0f;
      mesh_material.tex_Ks.b = 1.0f;

      mesh_material.tex_d = 1.0f;
      mesh_material.tex_Ns = 75.0f;
      mesh_material.tex_illum = 2;

      std::stringstream tex_name;
      tex_name << "material_" << i;
      tex_name >> mesh_material.tex_name;

      if (i < my_cams.size())
        mesh_material.tex_file = my_cams[i].texture_file;
      else
        mesh_material.tex_file = "occluded.jpg";

      mesh.tex_materials[i] = mesh_material;
    }
  }
  std::cout << "polygon mesh size: " << mesh.tex_polygons.size() << ", material size: " << mesh.tex_materials.size() << ", texture coordinate size: " << mesh.tex_coordinates.size() << std::endl;

  // FIXME: texture mapping works under release mode but not under debug mode
  // Sort faces
  {
    TIME_BLOCK("- sort faces");
    pcl::TextureMapping<pcl::PointXYZ> tm; // TextureMapping object that will perform the sort
    tm.textureMeshwithMultipleCameras(mesh, my_cams);
  }
  for (std::size_t i = 0; i <= my_cams.size(); ++i)
  {
      PCL_INFO("\tSub mesh %zu contains %zu faces and %zu UV coordinates.\n", i, mesh.tex_polygons[i].size(), mesh.tex_coordinates[i].size());
  }

  // compute normals for the mesh
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  {
    TIME_BLOCK("- compute normals");
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
  }

  // Concatenate XYZ and normal fields
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  {
    TIME_BLOCK("- concatenate xyz and normal fields");
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    pcl::toPCLPointCloud2(*cloud_with_normals, mesh.cloud);
  }

  //FIXME: error here: [PCLVisualizer::addTextureMesh] GPU texture units 0 < mesh textures 25!
  // https://github.com/PointCloudLibrary/pcl/issues/2252
#ifdef DEBUG_PLOT
  // viewer
  viewer.removeAllShapes(0);
  viewer.addTextureMesh(mesh, "mesh");
  viewer.spin();
#endif

  {
    TIME_BLOCK("- save mesh");
    pcl::io::saveOBJFile(argv[2], mesh, 5);
  }

  std::cout << "Done" << std::endl;

  return 0;
}
