#ifndef POINTCLOUD_HPP // this is a header guard, only needed in header files
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
};

class PointCloud {
    public:
    // constructor
    PointCloud();

    // Method to load data from a file
    // return boolean indicating success or failure
    bool loadFromFile(const std::string& filename);
        // const promises the function won't change the file
        // & is faster since no memory duplication (pass by reference)
    
    // Method to load data from a file with intensity
    // return boolean indicating success or failure
    bool loadFromFileIntensity(const std::string& filename);

        // Method to get a reference to the points vector
    // returns a constant reference to a vector of Points
    const std::vector<Point>& getPoints() const; 
    // 2nd const says it will not modify the object it's called on
    // so you can call it on a const object. Otherwise you can't

    void modulateColours();

    private:
        std::vector<Point> points_; // Stores the collection of points.
};

#endif // POINTCLOUD_HPP