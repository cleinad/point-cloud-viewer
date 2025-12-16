#include <iostream>
#include <filesystem>
#include <string>
#include <fstream>
#include <sstream>
#include <cctype>
#include "PointCloud.hpp"
namespace fs = std::filesystem;

// Include PCL headers for point cloud data structure and visualizer
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

// Helper function to detect if a file has intensity values
// Returns true if file format is: x y z intensity r g b (7 values)
// Returns false if file format is: x y z r g b (6 values)
bool hasIntensity(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false; // Default to no intensity if we can't read the file
    }
    
    std::string line;
    if (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        int count = 0;
        
        // Count the number of space-separated values in the first line
        while (ss >> token) {
            count++;
        }
        
        // 7 values means: x y z intensity r g b
        // 6 values means: x y z r g b
        return (count == 7);
    }
    
    return false; // Default to no intensity if file is empty
}

int main(int argc, char** argv) {
    // This is the entry point of your program.
    // The linker requires it to exist.
    std::cout << "Welcome to the program! \n" << std::endl;
    
    std::string path = "../data/"; // path containing all the data

    // for if the data directory doesn't exist
    if (!fs::exists(path)) {
        std::cout << "The data directory does not currently exist. \nPlease create a new folder at the root level named 'data'" << std::endl;
        return 1;
    } else {
        // std::cout << "Data directory exists!\n";
    }

    bool found = false; // keeps track of if there are any files in the directory

    std::vector<std::string> files; // a vector for all the files;

    // add file names to the vector
    for (const auto& entry : fs::directory_iterator(path)) {
        if (fs::is_regular_file(entry)) {
            files.push_back(entry.path().filename().string());
            found = true;
        }
    }

    int choice;
    if (found == false) {
        std::cout << "There are no current datasets to read from. \nPlease upload dataset to get started. \n" << std::endl;
        return 1;
    } else { // lists all the existing files
        for (size_t i = 0; i < files.size(); i++) {
            std::cout << i << ": " << files[i] << std::endl;
        }
        std::cout << "\nChoose a file to view by number: ";
        std::cin >> choice;

        if (choice >= 0 && choice < files.size()) {
            std::cout << files[choice] << " selected." << std::endl;
        }
    }

    // Prompt user for background color preference
    std::string bgChoice;
    std::cout << "\nChoose background color (black/white) [default: black]: ";
    std::cin >> bgChoice;
    
    std::cout << "\nStarting Point Cloud Viewer... " << std::endl;


    PointCloud cloud; // create an instance

    std::string filepath = "../data/" + files[choice];
    bool loadSuccess = false;
    
    // Detect file format and load accordingly
    if (hasIntensity(filepath)) {
        std::cout << "Detected file format: x y z intensity r g b" << std::endl;
        loadSuccess = cloud.loadFromFileIntensity(filepath);
    } else {
        std::cout << "Detected file format: x y z r g b" << std::endl;
        loadSuccess = cloud.loadFromFile(filepath);
    }
    
    if (loadSuccess) {
        std::cout << "\nSuccessfully loaded point cloud from file" << std::endl;
    } else {
        std::cerr << "Failed to load point cloud data" << std::endl;
        return 1;
    }

    // stretch the colours to be more vivid
    cloud.modulateColours(); // change the intensity
    // std::cout << "Colours modified" << std::endl;

    // Create a PCL point cloud object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Convert custom point cloud to PCL point cloud
    for (const auto& p : cloud.getPoints()) {
        // creates a pcl pointer object
        pcl::PointXYZRGB pcl_point;
        // copy over coordinate and colour data
        pcl_point.x = p.x;
        pcl_point.y = p.y;
        pcl_point.z = p.z;
        pcl_point.r = p.r;
        pcl_point.g = p.g;
        pcl_point.b = p.b;
        // add point to the cloud
        pcl_cloud->points.push_back(pcl_point);
    }
    pcl_cloud->width = pcl_cloud->points.size();
    pcl_cloud->height = 1;
    pcl_cloud->is_dense = false; // there may be invalid points

    
    
    // Convert to lowercase for case-insensitive comparison
    for (char& c : bgChoice) {
        c = std::tolower(c);
    }
    
    // Determine background color (default to black if invalid input)
    double bg_r = 0.0, bg_g = 0.0, bg_b = 0.0; // black default
    if (bgChoice == "white" || bgChoice == "w") {
        bg_r = 1.0;
        bg_g = 1.0;
        bg_b = 1.0;
        std::cout << "Using white background" << std::endl;
    } else {
        std::cout << "Using black background" << std::endl;
    }

    std::cout << "Viewing!" << std::endl;
    // Create the PCL visualizer and add the point cloud
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(bg_r, bg_g, bg_b);
    viewer->addPointCloud<pcl::PointXYZRGB>(pcl_cloud, "sample cloud");
    viewer -> setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    // Main visualization loop
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100); // update the viewer every 100 milliseconds
    }
    
    return 0; // indicates success
}