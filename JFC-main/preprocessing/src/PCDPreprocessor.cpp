#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <valarray>
#include <unordered_set>
#include <iomanip>
#include <getopt.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "KNNFilter.h"
#include "NormalCompute.h"

#define DEFAULT_NEIGHBOURS 6
#define DEFAULT_RADIUS 0.03
#define DEFAULT_POLY 3

using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

void usage(void);

				
int main(int argc, char* argv[] )
{
	int opt, do_filt = false, do_mls = false, do_norm = false, do_OMP = false, cmd_filedef = false, cmd_outdef = false, cmd_dbgdef = false; 
	int K_filt, K_norm, threadC, Poly;
	float Rad_mls;
	string Filepath, mlsstr;
    vector<string> v;
 
    string Outname = "Output";
    while(1){
		static struct option long_options[]={
			{"input", 	required_argument, 	0, 'i'},
			{"output", 	required_argument, 	0, 'O'},
			{"filter", 	required_argument, 	0, 'f'},
			{"normal", 	required_argument,  0, 'n'},
			{"openmp", 	required_argument, 	0, 'o'}, 
			{"mls", 	required_argument, 	0, 'm'},
			{"help", 	no_argument, 		0, 'h'},
			{"debug", 	no_argument, 		0, 'd'},
			{"version", no_argument, 		0, 'v'}
		};

		int option_index = 0;

		opt = getopt_long(argc, argv, ":i:O:f:n:o:m:hdv", long_options, &option_index);

		// Detect the end of options
		if(opt==-1) {
			break;
		}

		// Switch Case statement on opt
		switch(opt){
			case 'i':// --input X
				// set flag and filepath variable accordinly
				cmd_filedef = true;
				Filepath = optarg;
				break;
			case 'O':// --output X
				cmd_outdef = true;
				Outname = optarg;
				break;
			case 'f':// --filter X
				do_filt = true;	
				K_filt = stoi(optarg);
				break;
			case 'n':// --normal
				do_norm = true;
				K_norm = stoi(optarg);
				break;
			case 'o':
				do_OMP = true;
				threadC = stoi(optarg);
				break;
			case 'm':
				do_mls = true;
				mlsstr = optarg;
				break;
			case 'h':
				usage();
				return 0;
			case 'd':
				cmd_dbgdef = true;
				break;
			case 'v':
				std::cout << "PCD Preprocessor V1.0"<<endl;
				return 0;
			case '?':
				break;
			case ':':// No Argument cases argument cases
				switch(optopt){
					case 'i':
						std::cerr<<"ERROR - Input filename called but not given." <<endl;
						abort();
						break;
					case 'O':
						cmd_outdef = false;
						std::cerr<<"WARNING - Output filename option called without name specified: Ignoring..."<<endl;
						break;
					case 'f':
						do_filt = true;
						K_filt = DEFAULT_NEIGHBOURS;
						std::cerr<<"[Filter] Using default values | K = " << DEFAULT_NEIGHBOURS <<endl;
						break;
					case 'n':
						do_norm = true;
						K_norm = DEFAULT_NEIGHBOURS;
						std::cerr<<"[Normalization] Using default values | K = " << DEFAULT_NEIGHBOURS <<endl;
						break;
					case 'o':
						do_OMP = false;
						std::cerr<<"WARNING: [OPENMP] No thread count specified: Ignoring..."<<endl;
						break;
					case 'm':
						do_mls = true;
						Rad_mls = DEFAULT_NEIGHBOURS;
						Poly = DEFAULT_POLY;
						std::cerr<<"[MLS Resample] Using default values | Poly = " << DEFAULT_POLY << " | R = " << DEFAULT_RADIUS <<endl;
						break;
				}
				break;
			default:
				std::cerr<< "ERROR - Command line input unrecognized"<<endl;
				abort();
		}
	}

	// Create pointer to point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filt_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filt_cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	// Read point cloud data
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(Filepath, *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file\n");
		return (-1);
	}
	std::cout << "Loaded " << cloud->width * cloud->height << " points from " << Filepath << std::endl;

	// Filtering section
	if (do_filt){
		// Start a timer (Used for debugging with -d)
		auto t1 = high_resolution_clock::now();

		// KNN filter
		std::cout << "Starting KNN filter..." << std::endl << std::endl;
		KNN_filter(cloud, filt_cloud_colored, filt_cloud, K_filt);
		std::cout << "KNN filtering complete" << std::endl;

		// End the timer
		auto t2 = high_resolution_clock::now();
		duration<double, std::milli> ms_double = t2 - t1;
		if (cmd_dbgdef){ //If debug is on print the time taken
			std::cout << "\t[DEBUG] - Filtered cloud in " << ms_double.count() << " milliseconds"<<std::endl;
		}
	}
	
	// Normal Section: Only completed if turned on and mls is turned off as the mls process computes normals internally.
	if (do_norm && !do_mls){
		// Start a timer
		auto t1 = high_resolution_clock::now();

		if(do_OMP){		// If OMP acceleration is enabled run it.
			if(do_filt){
				cloud_normals = NormalComputeOMP(filt_cloud, K_norm, threadC); // If filtered use filt_cloud
			}
			else{
				cloud_normals = NormalComputeOMP(cloud, K_norm, threadC); // If non filtered use cloud
			}			
		}else{			// If OMP Dissabled
			if(do_filt){
				cloud_normals = NormalCompute(filt_cloud, K_norm); // If filtered use filt_cloud
			}
			else{
				cloud_normals = NormalCompute(cloud, K_norm); // If non filtered use cloud
			}		
		}

		auto t2 = high_resolution_clock::now();
		duration<double, std::milli> ms_double = t2 - t1;
		if (cmd_dbgdef){ //If debug is on print the time taken
			std::cout << "\t[DEBUG] - Normalized cloud in " << ms_double.count() << " milliseconds"<<std::endl;
		}
	}

	// MLS Resampling section
	if (do_mls){
		// Create stringstream object and assign it the input command argument
		stringstream ss(mlsstr);
		// Until no more delimiters seperate string and push to a vector
		while (ss.good()) {
			string substr;
			getline(ss, substr, ',');
			v.push_back(substr);
		}
		// Extract the data from the vector converting string to correct type.
		Poly = stoi(v[0]);
		Rad_mls = stof(v[1]);

		auto t1 = high_resolution_clock::now();
		if(do_OMP){			// If OMP acceleration is enabled run it.
			if(do_filt){
				cloud_normals = mlsOMP(filt_cloud, Rad_mls, Poly, threadC); // If filtered use filt_cloud
			}else{
				cloud_normals = mlsOMP(cloud, Rad_mls, Poly, threadC);		// If non filtered use cloud
			}
		}else{
			if(do_filt){
				cloud_normals = mls(filt_cloud, Rad_mls, Poly); // If filtered use filt_cloud
			}else{
				cloud_normals = mls(cloud, Rad_mls, Poly); 		// If non filtered use cloud
			}
		}
		auto t2 = high_resolution_clock::now();
		duration<double, std::milli> ms_double = t2 - t1;
		if (cmd_dbgdef){ //If debug is on print the time taken
			std::cout << "\t[DEBUG] - Resampled cloud in " << ms_double.count() << " milliseconds"<<std::endl;
		}
	}
	
	// Output Section
	if (do_filt && !do_mls && !do_norm){		//If only filtered
		// Save output using the filt_cloud
		pcl::io::savePCDFile(Outname.append(".pcd"), *filt_cloud);
	}else if (do_mls || do_norm){				// If running mls or normals. output the respective cloud.
		pcl::io::savePCDFile(Outname.append(".pcd"), *cloud_normals);
	}

	std::cout << "Output processed pointcloud to " << Outname << "." << std::endl;





	// //---------------------------------------------
	// //-------------Display Cloud VTK---------------
	// //---------------------------------------------
	// pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Test Viewer"));
	// viewer->setBackgroundColor(0, 0, 0);
	// viewer->initCameraParameters();
	// viewer->setCameraPosition(-0.1, -0.1, 0, 0, 0, 0, 0, 0, 1);
	// viewer->setCameraFieldOfView(0.523599);
	// viewer->setCameraClipDistances(0.00522511, 50);

	// // Clear the view
	// viewer->removeAllShapes();
	// viewer->removeAllPointClouds();

	// // Create a viewport 1
	// int v1(0);
	// viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	// viewer->setBackgroundColor(0, 0, 0, v1);
	// viewer->addText("No processing", 10, 10, "v1 text", v1);
	// viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud1", v1);

	// // Create a viewport 2
	// int v2(0);
	// viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	// viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	// viewer->addText("Processing", 10, 10, "v2 text", v2);
	// viewer->addPointCloud<pcl::PointNormal>(mls_points, "cloud2", v2);

	// // Rendering Properties
	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");
	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2");

	// // viewer->resetCamera ();
	// while (!viewer->wasStopped())
	// {
	// 	viewer->spinOnce(100);
	// }

	return 0;
}


void usage(void){
	std::cout << 
		"\e[1;34m//~~~~Option Definitions~~~~//\e[0m" << endl
		<< "\e[1;33m[-i] [--input]\e[0m\t- Fileinput requires the following to be passed as an argument;"<<endl
				<< "\t\e[0;34mchar* \e[0;36mFilePath\e[0m;\tPath to input file" << endl<<endl
		<< "\e[1;33m[-O] [--output]\e[0m\t- Output Filename requires the following to be passed as an argument;" <<endl
				<< "\t\e[0;34mchar* \e[0;36mOutName\e[0m;\tOutput filename excluding extension" << endl<<endl
		<<"\e[1;33m[-f] [--filter]\e[0m\t- Program uses statistical filter, requires the following to b e passed as an argument;" <<endl
				<< "\t\e[0;34mint \e[0;36mK_filt\e[0m;\tKnn search number of neighbours"<< endl<<endl
		<<"\e[1;33m[-n] [--normal]\e[0m\t- Program will normalize points. requires the following to be passed as an argument;" << endl
				<<"\t\e[0;34mint \e[0;36mK_norm\e[0m;\tNeighbours to search for normals (m)"<<endl<<endl
		<<"\e[1;33m[-o] [--openmp]\e[0m\t- Enable multithreading for supported functions. requires the following;"<<endl
				<<"\t\e[0;34mint \e[0;36mthreadC\e[0m;\tNumber of threads to execute on." <<endl<<endl
		<<"\e[1;33m[-m]\t" "[--mls]\e[0m\t- Program will resample using MLS. requires the following to be passed as an argument in a comma seperated form;"<<endl
				<<"\te.g. \"-m 3,0.03\""<<endl
				<<"\t\e[0;34mint \e[0;36mPoly\e[0m;\tOrder of polynomial to to find"<<endl
				<<"\t\e[0;34mfloat \e[0;36mRad_mls\e[0m;\tRadius to search for mls (m)."<<endl<<endl
		<<"\e[1;33m[-h] [--help]\e[0m\t- Print this options menu"<<endl<<endl
		<<"\e[1;33m[-d] [--debug]\e[0m\t- Prints helpful items to the terminal including time estimates, and execution times. Must have a small .pcd file called test.pcd in root folder"<<endl<<endl
		<<"\e[1;33m[-v] [--version]\e[0m- Print version identifier"<<endl;							
}