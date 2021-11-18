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

    double val;

    go_to_line(myReadFile, 1);
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << ("t ");
    std::cout << val;
#endif
    cam.pose(0, 3) = val; // TX
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << (" ");
    std::cout << val;
#endif
    cam.pose(1, 3) = val; // TY
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << (" ");
    std::cout << val;
    std::cout << ("\n");
#endif
    cam.pose(2, 3) = val; // TZ

    // r1
    go_to_line(myReadFile, 2);
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << ("r1 ");
    std::cout << val;
#endif
    cam.pose(0, 0) = val;
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << (" ");
    std::cout << val;
#endif
    cam.pose(0, 1) = val;
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << (" ");
    std::cout << val;
    std::cout << ("\n");
#endif
    cam.pose(0, 2) = val;

    // r2
    go_to_line(myReadFile, 3);
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << ("r2 ");
    std::cout << val;
#endif
    cam.pose(1, 0) = val;
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << (" ");
    std::cout << val;
#endif
    cam.pose(1, 1) = val;
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << (" ");
    std::cout << val;
    std::cout << ("\n");
#endif
    cam.pose(1, 2) = val;

    // r3
    go_to_line(myReadFile, 4);
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << ("r3");
    std::cout << val;
#endif
    cam.pose(2, 0) = val;
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << (" ");
    std::cout << val;
#endif
    cam.pose(2, 1) = val;
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << (" ");
    std::cout << val;
    std::cout << ("\n");
#endif
    cam.pose(2, 2) = val;

    cam.pose(3, 0) = 0.0;
    cam.pose(3, 1) = 0.0;
    cam.pose(3, 2) = 0.0;
    cam.pose(3, 3) = 1.0; // Scale

    // fx
    go_to_line(myReadFile, 5);
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << ("fx ");
    std::cout << val;
    std::cout << ("\n");
#endif
    cam.focal_length_w = val;

    // fy
    go_to_line(myReadFile, 6);
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << ("fy ");
    std::cout << val;
    std::cout << ("\n");
#endif
    cam.focal_length_h = val;

    // cx
    go_to_line(myReadFile, 7);
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << ("cx ");
    std::cout << val;
    std::cout << ("\n");
#endif
    cam.center_w = val;

    // cy
    go_to_line(myReadFile, 8);
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << ("cy ");
    std::cout << val;
    std::cout << ("\n");
#endif
    cam.center_h = val;

    // h
    go_to_line(myReadFile, 9);
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << ("h ");
    std::cout << val;
    std::cout << ("\n");
#endif
    cam.height = val;

    // w
    go_to_line(myReadFile, 10);
    myReadFile >> val;
#ifdef DEBUG_PRINT
    std::cout << ("w ");
    std::cout << val;
    std::cout << ("\n");
#endif
    cam.width = val;

    // close file
    myReadFile.close();
    return true;
}