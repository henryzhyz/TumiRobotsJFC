#include <iostream>
#include <string>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

int main()
{
    string FilePath = "../../section1_rooftbolts.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::cout << "Beginning Loading Process of \"" << FilePath << '\"' << std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(FilePath, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file\n");
        return (-1);
    }
    std::cout << "Loaded " << cloud->width * cloud->height << " points" <<std::endl;

    
    //---------------------------------------------
    //-------------Display Cloud VTK---------------
    //---------------------------------------------
    std::cout << "Beginning Visualization" << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Test Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();

    // Clear the view
    viewer->removeAllShapes();
    viewer->removeAllPointClouds();

    // The point cloud
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    // viewer->resetCamera ();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return (0);
}