#include "PointCloud.hpp"
#include <fstream>  // for reading and writing to files;
                    // std::ifstream -> for reading from files
                    // std::ofstream -> for writing to files
                    // std:: fstream -> for both reading and writing
#include <sstream>  // treat strings like streams, good for parsing or formatting text
                    // std::stringstream -> both input and output
                    // std::istringstream -> input only
                    // std::ostringstream -> output only
#include <iostream>

// No classes in source files

// Default Constructor
PointCloud::PointCloud() {
    //
}

bool PointCloud::loadFromFile(const std::string& filename) { // ensure that it is a member function
    std::ifstream file(filename); // read from file
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false; // indicates error
    }

    std::string line;
    int num_points_read = 0; // 0 points read so far
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        Point p;

        // Parse the values from the line based on your dataset format:
        // x y z intensity r g b
        ss >> p.x >> p.y >> p.z >> p.intensity >> p.r >> p.g >> p.b;

        // Implement downsampling: load every 100th point.
        // crucial for load handling and not overloading the CPU/GPU
        if (num_points_read % 100 == 0) {
            points_.push_back(p); // add it to the end of the private vector
        }
        num_points_read++;
    }

    std::cout << "Successfully loaded " << points_.size() << " points" << std::endl;
    return true; // indicates success
}