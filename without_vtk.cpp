//
// Created by Harold on 2021/11/17.
//

#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/common/io.h> // for concatenateFields

#include "read_cam_file.h"
#include "stopwatch.h"

//#define DEBUG_PRINT

// argv[1]: input mesh file
// argv[2]: output obj file
// argv[3]: input texture.png / camera.txt folder 
int main(int argc, char **argv)
{
  pcl::PolygonMesh triangles;
  {
    TIME_BLOCK("- load mesh");
    auto mesh_ext = boost::filesystem::extension(argv[1]);
    if (mesh_ext == ".ply")
        pcl::io::loadPLYFile(argv[1], triangles);
    else if (mesh_ext == ".obj")
        pcl::io::loadOBJFile(argv[1], triangles);
    else
    {
        std::cerr << "only support .ply or .obj mesh file" << std::endl;
        exit(0);
    }
  }

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

  {
    TIME_BLOCK("- save mesh");
    pcl::io::saveOBJFile(argv[2], mesh, 5);
  }

  std::cout << "Done" << std::endl;

  return 0;
}
