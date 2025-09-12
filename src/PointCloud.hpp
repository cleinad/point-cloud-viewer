#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

#include <cstdint> // for the 8 bit typedef
#include <vector>
#include <string>

// struct to represent a single point with XYZ coordinates, intensity, and RGB colour
struct Point {
    float x, y, z; // coordinates
    float intensity; // intensity (shocker!)
    uint8_t r, g, b; // uint8_t is from 0 to 255
    // r = 255 indicates full red
    // uint8_t = unsigned char but clearer
}

class PointCloud {
    public:
    // a constructor
    PointCloud()
}