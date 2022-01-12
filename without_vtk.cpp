//
// Created by Harold on 2021/11/17.
//

#pragma warning(disable : 4996)
#pragma warning(disable : 4819)

#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/common/io.h> // for concatenateFields

#include "stopwatch.h"

//#define DEBUG_PRINT
#include "read_cam_file_hololens2.h"

#define SAVE_CAM
#ifdef SAVE_CAM
// save camera pose lines for future plotting
void SaveCameraPoints(pcl::texture_mapping::Camera const& cam);
#endif

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

  const boost::filesystem::path base_dir(argv[3]);
  // Load textures and cameras poses and intrinsics
  pcl::texture_mapping::CameraVector my_cams;
  {
    TIME_BLOCK("- load textures and cameras poses and intrinsics");
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
      {
        mesh_material.tex_file = my_cams[i].texture_file;
#ifdef SAVE_CAM
        SaveCameraPoints(my_cams[i]);
#endif
      }
      else
        mesh_material.tex_file = (base_dir / boost::filesystem::path("occluded.jpg")).string();

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

#ifdef SAVE_CAM
void SaveCameraPoints(pcl::texture_mapping::Camera const& cam)
{
  double focal_x = cam.focal_length_w;
  double focal_y = cam.focal_length_h;
  double height = cam.height;
  double width = cam.width;

  // create a 5-point visual for each camera
  pcl::PointXYZ p1, p2, p3, p4, p5;
  p1.x = 0;
  p1.y = 0;
  p1.z = 0;
  double angleX = RAD2DEG(2.0 * atan(width / (2.0 * focal_x)));
  double angleY = RAD2DEG(2.0 * atan(height / (2.0 * focal_y)));
  double dist = 0.75;
  double minX, minY, maxX, maxY;
  maxX = dist * tan(atan(width / (2.0 * focal_x)));
  minX = -maxX;
  maxY = dist * tan(atan(height / (2.0 * focal_y)));
  minY = -maxY;
  p2.x = minX;
  p2.y = minY;
  p2.z = dist;
  p3.x = maxX;
  p3.y = minY;
  p3.z = dist;
  p4.x = maxX;
  p4.y = maxY;
  p4.z = dist;
  p5.x = minX;
  p5.y = maxY;
  p5.z = dist;
  p1 = pcl::transformPoint(p1, cam.pose);
  p2 = pcl::transformPoint(p2, cam.pose);
  p3 = pcl::transformPoint(p3, cam.pose);
  p4 = pcl::transformPoint(p4, cam.pose);
  p5 = pcl::transformPoint(p5, cam.pose);

  auto camera_file = boost::filesystem::path(cam.texture_file).replace_extension(".cam").string();
  std::ofstream ss(camera_file);
  ss << p1 << p2 << '\n'
     << p1 << p3 << '\n'
     << p1 << p4 << '\n'
     << p1 << p5 << '\n'
     << p2 << p5 << '\n'
     << p5 << p4 << '\n'
     << p4 << p3 << '\n'
     << p3 << p2;
}
#endif
