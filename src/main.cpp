#include <iostream>
#include "PointCloud.hpp"

int main(int argc, char** argv) {
    // This is the entry point of your program.
    // The linker requires it to exist.
    std::cout << "Hello, World! " << "I love Jesus.\n\n" << std::endl;
    std::cout << "Starting Point Cloud Viewer... " << std::endl;

    PointCloud cloud; // create an instance

    if (cloud.loadFromFile("../data/bildstein_station1_sample1.txt")) {
        std::cout << "Successfully loaded point cloud from file." << std::endl;
    } else {
        std::cerr << "Failed to load point cloud." << std::endl;
        return 1;
    }

    // visualization to follow

    return 0; // indicates success
}