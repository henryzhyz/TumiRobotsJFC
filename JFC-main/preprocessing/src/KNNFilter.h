#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <valarray>
#include <unordered_set>
#include <iomanip>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>

void remove(std::vector<int>& v)
{
	std::vector<int>::iterator itr = v.begin();
	std::unordered_set<int> s;

	for (auto curr = v.begin(); curr != v.end(); ++curr)
	{
		if (s.insert(*curr).second)
		{
			*itr++ = *curr;
		}
	}
	v.erase(itr, v.end());
}

void KNN_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr filt_cloud_colored, pcl::PointCloud<pcl::PointXYZRGB>::Ptr filt_cloud, int NUM_NEIGHBORS)
{
	// Create kdtree object
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud);
	pcl::PointXYZRGB searchPoint;

	// K nearest neighbor search
	int K = NUM_NEIGHBORS + 1; // number of neighboors

	// kdtree searching vectors
	std::vector<int> pointIdxKNNSearch(K);
	std::vector<float> pointKNNSquaredDistance(K);

	// KNN points
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr KNNpoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	KNNpoints->resize(K);
	std::vector<float> pointKNNDistance_vec;

	// Fitting plane vectors
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZRGB> seg; // segmentation object
	std::vector<int> inliers_idx;

	// start timer
	auto start = std::chrono::steady_clock::now();

	// Search KNN points
	int cont_failed = 0;
	int cont = 0;
	for (const auto& point : *cloud)
	{
		if (kdtree.nearestKSearch(point, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
		{
			// Find distances to the KNN points
			std::valarray<float> pointKNNDistance(pointKNNSquaredDistance.data(), K);
			pointKNNDistance = sqrt(pointKNNDistance);
			pointKNNDistance_vec.assign(std::begin(pointKNNDistance), std::end(pointKNNDistance));
			// for (int i = 1; i < pointIdxKNNSearch.size(); i++) std::cout << "Distance: " << pointKNNDistance_vec[i] << std::endl;


			// Compute mean and standard deviation
			double sum = std::accumulate(pointKNNDistance_vec.begin(), pointKNNDistance_vec.end(), 0.0);
			double mean = sum / (pointKNNDistance_vec.size() - 1);
			double sq_sum = std::inner_product(pointKNNDistance_vec.begin(), pointKNNDistance_vec.end(), pointKNNDistance_vec.begin(), 0.0);
			double stdev = std::sqrt(sq_sum / (pointKNNDistance_vec.size() - 1) - mean * mean);
			/*std::cout << "mean: " << mean << ", 2*std: " << 2 * stdev << std::endl;*/

			// Get the KNN points
			for (int i = 0; i < pointIdxKNNSearch.size(); i++)
				KNNpoints->points[i] = cloud->points[pointIdxKNNSearch[i]];

			///// Fit a plane to the neighborhood
			seg.setOptimizeCoefficients(false);// Optional

			// Mandatory
			seg.setModelType(pcl::SACMODEL_PLANE); // fit to a plane
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(2 * stdev);  // minimum distance of 2*stdev
			seg.setInputCloud(KNNpoints);		  // Neighborhood as the point cloud
			seg.segment(*inliers, *coefficients); // get inliers and plane coefficients

			for (const auto& idx : inliers->indices)
			{
				// std::cout << "idx: " << idx << std::endl;
				// std::cout << "Storing index: " << pointIdxKNNSearch[idx] << std::endl;
				inliers_idx.push_back(pointIdxKNNSearch[idx]);
			}

			cont++;

			if (inliers->indices.size() == 0)
			{
				cont_failed++;
				// std::cout << "Fail!" << std::endl;
				continue;
			}
		}
	}

	// Remove duplicates from the inliers_idx vector
	remove(inliers_idx);
	std::cout << "Number of inliers: " << inliers_idx.size() << std::endl;
	std::cout << "Number of outliers: " << cloud->points.size() - inliers_idx.size() << std::endl;

	// stop timer
	auto end = std::chrono::steady_clock::now();

	// Number of points that failed to fit a plane
	std::cout << "Number of points that failed to fit a plane: " << cont_failed << std::endl;
	std::cout << "Failed Percentage: " << (float)cont_failed * 100.0 / (cloud->points.size()) << "%" << std::endl;

	// Display measured time
	cout << "Filtering elapsed time in milliseconds: "
		<< std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
		<< " ms" << endl;

	// Copy the original to the filtered cloud
	pcl::copyPointCloud(*cloud, *filt_cloud_colored);
	filt_cloud->resize(inliers_idx.size());

	// Colour outliers with red
	for (int i = 0; i < filt_cloud_colored->points.size(); i++)
	{
		filt_cloud_colored->points[i].r = 255;
		filt_cloud_colored->points[i].g = 0;
		filt_cloud_colored->points[i].b = 0;
	}

	// Colour inliers with white
	for (int i = 0; i < inliers_idx.size(); i++)
	{
		//// add the color to filt_cloud_colored
		filt_cloud_colored->points[inliers_idx[i]].r = 255;
		filt_cloud_colored->points[inliers_idx[i]].g = 255;
		filt_cloud_colored->points[inliers_idx[i]].b = 255;

		// keep filtered points in filt_cloud
		filt_cloud->points[i] = cloud->points[inliers_idx[i]];
	}
}