#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/comparator.h>

#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <valarray>
#include <unordered_set>

#include "KNNFilter.h"

using namespace std;
using namespace pcl;

int main()
{
	string FilePath = "../../PointcloudTests/bunny.pcd";
	// string FilePath;

	// std::cout << "Please input filepath to .PCD cloud to filter (Or drag and drop the file into the terminal)." << std::endl;
	// std::cin >> FilePath;

	// Create pointer to point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filt_cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filt_cloud_cc(new pcl::PointCloud<pcl::PointXYZ>);

	// Read point cloud data
    std::cout << "Beginning Loading Process of \"" << FilePath << "\"..." << std::endl << std::endl;
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(FilePath, *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	std::cout << "Loaded " << cloud->width * cloud->height << " points" << std::endl;

	// Lets introduce noise onto the point cloud
	srand((unsigned)time(NULL));
	int num_points = cloud->size();
	int i=0;
	for(i; i<num_points/10; i++)
	{
		int idx = (rand() % num_points) + 1;
		cloud->points[idx].x += 0.02 * (float)rand() / RAND_MAX;
		cloud->points[idx].y += 0.02 * (float)rand() / RAND_MAX;
		cloud->points[idx].z += 0.02 * (float)rand() / RAND_MAX;
	}
	
	std::cout << "Added noise to " << i << " points." << std::endl;

	// KNN filter
	std::cout << "Starting KNN filter..." << std::endl << std::endl;
	KNN_filter(cloud, filt_cloud_colored, filt_cloud);
	std::cout << "\nKNN filtering done!" << std::endl;

	// Connected component filter
	// Connected_component(filt_cloud, filt_cloud_cc);

	//---------------------------------------------
	//-------------Display Cloud VTK---------------
	//---------------------------------------------

	std::cout << "Beginning the visualization" << std::endl;

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Test Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(-0.1, -0.1, 0, 0, 0, 0, 0, 0, 1);
	viewer->setCameraFieldOfView(0.523599);
	viewer->setCameraClipDistances(0.00522511, 50);

	// Clear the view
	viewer->removeAllShapes();
	viewer->removeAllPointClouds();

	// Create a viewport 1
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Unfiltered PointCloud", 10, 10, "v1 text", v1);
	viewer->addPointCloud<pcl::PointXYZRGB>(filt_cloud_colored, "cloud1", v1);

	// Create a viewport 2
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.15, 0.15, 0.15, v2);
	viewer->addText("Filtered Point Cloud", 10, 10, "v2 text", v2);
	viewer->addPointCloud<pcl::PointXYZ>(filt_cloud, "cloud2", v2);

	// The point cloud
	viewer->addPointCloud<pcl::PointXYZ>(filt_cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");

	// Save the filtered cloud to a .pcd File
	pcl::io::savePCDFileASCII("test_pcd.pcd", *filt_cloud);

	std::cerr << "Saved " << filt_cloud->size () << " data points to test_pcd.pcd." << std::endl;
	// viewer->resetCamera ();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	return 0;
}