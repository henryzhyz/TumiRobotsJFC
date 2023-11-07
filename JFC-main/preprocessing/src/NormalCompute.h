#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <iostream>

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr NormalComputeOMP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int K_search, int ThreadC)
{
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> neomp;
    neomp.setNumberOfThreads(ThreadC);
    neomp.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    neomp.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    // Copy XYZRGB data from the input cloud to the output cloud
    copyPointCloud(*cloud, *cloud_normals);

    // Use all KNN search
    neomp.setKSearch(K_search);
        
    // Compute the features
    neomp.compute(*cloud_normals);

    return(cloud_normals);
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr NormalCompute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int K_search)
{
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    // Copy XYZRGB data from the input cloud to the output cloud
    copyPointCloud(*cloud, *cloud_normals);

    // Use all KNN search
    ne.setKSearch(K_search);
        
    // Compute the features
    ne.compute(*cloud_normals);

    return(cloud_normals);
}


pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mls(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double Search_Radius, int Polynomial){
    // Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

	// Output has the PointXYZRGBNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;

	mls.setComputeNormals(true);

	// Set parameters
	mls.setInputCloud(cloud);
	mls.setPolynomialOrder(Polynomial);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(Search_Radius);

	// // Reconstruct
	mls.process(*mls_points);

    return(mls_points);
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mlsOMP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double Search_Radius, int Polynomial, int ThreadC){
    // Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

	// Output has the PointXYZRGBNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;

	mls.setComputeNormals(true);

	// Set parameters
	mls.setInputCloud(cloud);
	mls.setPolynomialOrder(Polynomial);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(Search_Radius);
    mls.setNumberOfThreads(ThreadC);
    
	// // Reconstruct
	mls.process(*mls_points);

    return(mls_points);
}