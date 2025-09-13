#include <iostream>
#include "PointCloud.hpp"

// Include PCL headers for point cloud data structure and visualizer
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv) {
    // This is the entry point of your program.
    // The linker requires it to exist.
    std::cout << "Hello, World! " << "I love Jesus.\n\n" << std::endl;
    std::cout << "Starting Point Cloud Viewer... " << std::endl;

    PointCloud cloud; // create an instance

    if (cloud.loadFromFile("../data/bildstein_station1.txt")) {
        std::cout << "Successfully loaded point cloud from file" << std::endl;
    } else {
        std::cerr << "Failed to load point cloud data" << std::endl;
        return 1;
    }

    cloud.modulateColours(); // change the intensity
    std::cout << "Colours modified" << std::endl;

    // Create a PCL point cloud object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Convert custom point cloud to PCL point cloud
    for (const auto& p : cloud.getPoints()) {
        pcl::PointXYZRGB pcl_point;
        pcl_point.x = p.x;
        pcl_point.y = p.y;
        pcl_point.z = p.z;
        pcl_point.r = p.r;
        pcl_point.g = p.g;
        pcl_point.b = p.b;
        pcl_cloud->points.push_back(pcl_point);
    }
    pcl_cloud->width = pcl_cloud->points.size();
    pcl_cloud->height = 1;
    pcl_cloud->is_dense = false;

    std::cout << "Viewing!" << std::endl;
    // Create the PCL visualizer and add the point cloud
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0); // Black background
    viewer->addPointCloud<pcl::PointXYZRGB>(pcl_cloud, "sample cloud");
    viewer -> setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

    // Main visualization loop
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
    
    return 0; // indicates success
}