//
// Created by Harold on 2021/11/17.
//

#pragma once

#include <fstream>
#include <iostream>
#include <sstream>

#include <pcl/surface/texture_mapping.h>

std::ifstream& go_to_line(std::ifstream& file, unsigned int num)
{
    file.seekg(std::ios::beg);
    for (int i = 0; i < num - 1; ++i)
    {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return (file);
}

bool read_cam_pose_file(std::string filename, pcl::TextureMapping<pcl::PointXYZ>::Camera& cam)
{
    std::ifstream myReadFile;
    myReadFile.open(filename.c_str(), std::ios::in);
    if (!myReadFile.is_open())
    {
        PCL_ERROR("Error opening file %d\n", filename.c_str());
        return false;
    }
    myReadFile.seekg(std::ios::beg);

#ifdef DEBUG_PRINT
    std::cout << "camera: " << filename << '\n';
#endif

    const double width = 2272;
    const double height = 1278;
    const double half_width = width / 2.;
    const double half_height = height / 2.;
    double val, a, b, c, d;

    // projection matrix
    go_to_line(myReadFile, 3);
    myReadFile >> a;
#ifdef DEBUG_PRINT
    std::cout << "a: " << a << ", ";
#endif
    myReadFile >> val;
    myReadFile >> b;
#ifdef DEBUG_PRINT
    std::cout << "b: " << b << ", ";
#endif
    go_to_line(myReadFile, 4);
    myReadFile >> val;
    myReadFile >> c;
#ifdef DEBUG_PRINT
    std::cout << "c: " << c << ", ";
#endif
    myReadFile >> d;
#ifdef DEBUG_PRINT
    std::cout << "d: " << d << '\n';
#endif
    // fx
    cam.focal_length_w = a * half_width;
    // fy
    cam.focal_length_h = c * half_height;
    // cx
    cam.center_w = half_width * (1 + b);
    // cy
    cam.center_h = half_height * (1 + d);
    // h
    cam.height = height;
    // w
    cam.width = width;

    // camera_to_world matrix
    go_to_line(myReadFile, 9);
    myReadFile >> val;
    cam.pose(0, 0) = val;
    myReadFile >> val;
    cam.pose(0, 1) = val;
    myReadFile >> val;
    cam.pose(0, 2) = val;
    myReadFile >> val;
    cam.pose(0, 3) = val;

    go_to_line(myReadFile, 10);
    myReadFile >> val;
    cam.pose(1, 0) = val;
    myReadFile >> val;
    cam.pose(1, 1) = val;
    myReadFile >> val;
    cam.pose(1, 2) = val;
    myReadFile >> val;
    cam.pose(1, 3) = val;

    go_to_line(myReadFile, 11);
    myReadFile >> val;
    cam.pose(2, 0) = val;
    myReadFile >> val;
    cam.pose(2, 1) = val;
    myReadFile >> val;
    cam.pose(2, 2) = val;
    myReadFile >> val;
    cam.pose(2, 3) = val;

    go_to_line(myReadFile, 12);
    myReadFile >> val;
    cam.pose(3, 0) = val;
    myReadFile >> val;
    cam.pose(3, 1) = val;
    myReadFile >> val;
    cam.pose(3, 2) = val;
    myReadFile >> val;
    cam.pose(3, 3) = val;

    // camera_to_world => world_to_camera
    //cam.pose.linear() = cam.pose.linear().inverse();
    //cam.pose.linear()(0, 2) *= -1;
    //cam.pose.linear().row(1) *= -1;
    //cam.pose.linear()(2, 0) *= -1;

    cam.pose.translation().z() *= -1;

    char c_;
    Eigen::Vector3f euler;
    // world euler
    go_to_line(myReadFile, 17);
    myReadFile >> c_;
    myReadFile >> val;
    euler(0) = DEG2RAD(val);
    myReadFile >> c_;
    myReadFile >> val;
    euler(1) = DEG2RAD(val);
    myReadFile >> c_;
    myReadFile >> val;
    euler(2) = DEG2RAD(val);

    Eigen::Matrix3f m;
    auto x = Eigen::AngleAxisf(-euler(0), Eigen::Vector3f::UnitX());
    auto y = Eigen::AngleAxisf(-euler(1), Eigen::Vector3f::UnitY());
    auto z = Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitZ());

    // rotate 180 along x-axis
    const Eigen::Matrix3f b_rot = (Eigen::Matrix3f() << 1, 0, 0, 0, -1, 0, 0, 0, -1).finished();
    cam.pose.linear() = z * y * x * b_rot;

#ifdef DEBUG_PRINT
    std::cout << cam.pose.matrix() << std::endl;
#endif

    // close file
    myReadFile.close();
    return true;
}