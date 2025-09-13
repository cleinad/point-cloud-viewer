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

const std::vector<Point>& PointCloud::getPoints() const {
    return points_;
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
        int r_int, g_int, b_int;

        // Parse the values from the line based on your dataset format:
        // x y z intensity r g b
        ss >> p.x >> p.y >> p.z >> p.intensity >> r_int >> g_int >> b_int;
        p.r = r_int;
        p.g = g_int;
        p.b = b_int;

        // Implement downsampling: load every 100th point.
        // crucial for load handling and not overloading the CPU/GPU
        if (num_points_read % 100 == 0) {
            points_.push_back(p); // add it to the end of the private vector
            // std::cout << "point added" << std::endl; // debug statement
        }
        num_points_read++;
    }

    std::cout << "Successfully loaded " << points_.size() << " points" << std::endl;
    return true; // indicates success
}

void PointCloud::modulateColours() {
    // Find min/max values for each colour channel
    uint8_t min_r = 255, max_r = 0;
    uint8_t min_g = 255, max_g = 0;
    uint8_t min_b = 255, max_b = 0;

    for (const auto& p : points_) {
        min_r = std::min(min_r, p.r);
        max_r = std::max(max_r, p.r);

        min_g = std::min(min_g, p.g);
        max_g = std::max(max_g, p.g);

        min_b = std::min(min_b, p.b);
        max_b = std::max(max_b, p.b);
    }
    
    // increase intensity by stretching colours
    for (auto& p : points_) {
        if (max_r > min_r) p.r = static_cast<uint8_t>(255.0 * (p.r - min_r) / (max_r - min_r));
        if (max_g > min_g) p.g = static_cast<uint8_t>(255.0 * (p.g - min_g) / (max_g - min_g));
        if (max_b > min_b) p.b = static_cast<uint8_t>(255.0 * (p.b - min_b) / (max_b - min_b));
    }
}