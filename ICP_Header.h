#pragma once
#ifndef ICP_HEADER
#define ICP_HEADER

//general headers
#include <iostream>
#include <string>
#include <iomanip>
#include <thread>

//pcl headers
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/file_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>


using namespace std;
using namespace pcl;

typedef PointXYZRGBA PointT;
typedef PointCloud<PointT> PointCloudT;


class ICP {
private:
	
	console::TicToc time;

	PointCloudT::Ptr cloud_1_orig;// (new PointCloudT);
	PointCloudT::Ptr cloud_2_orig;// (new PointCloudT);
	PointCloudT::Ptr cloud_1_filt;// (new PointCloudT);
	PointCloudT::Ptr cloud_2_filt;// (new PointCloudT);

public:
	ICP()
	{


	}
	~ICP()
	{

	}

	void load_pc_1(string cloudname)
	{
		time.tic();
		PointCloudT::Ptr temp_cloud(new PointCloudT);

		if (io::loadPCDFile<PointT>(cloudname, *temp_cloud) == -1)
		{
			PCL_ERROR("Couldnt read file.\n");
		}
		
		cloud_1_orig = temp_cloud;

		//delete &temp_cloud;

		cout << "Loaded 1st Cloud in " << time.toc() << "ms." << endl;
	}

	void load_pc_2(string cloudname)
	{
		time.tic();
		PointCloudT::Ptr temp_cloud(new PointCloudT);

		if (io::loadPCDFile<PointT>(cloudname, *temp_cloud) == -1)
		{
			PCL_ERROR("Couldnt read file.\n");
		}

		cloud_2_orig = temp_cloud;

		//delete &temp_cloud;

		cout << "Loaded 2nd Cloud in " << time.toc() << "ms." << endl;
	}

	void filter_clouds(float uppr_lim=6.0, float lower_lim=-6.0)
	{
		cout << "Filtering Clouds." << endl;
		time.tic();
		PointCloudT::Ptr temp_cloud(new PointCloudT);

		PassThrough<PointT> pass;
		pass.setInputCloud(cloud_1_orig);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(lower_lim, uppr_lim);
		pass.filter(*temp_cloud);
		cloud_1_filt = temp_cloud;

		cout << "Filtered 1st Cloud in " << time.toc() << "ms." << endl;
		time.tic();

		PassThrough<PointT> pass2;
		pass2.setInputCloud(cloud_2_orig);
		pass2.setFilterFieldName("z");
		pass2.setFilterLimits(lower_lim, uppr_lim);
		pass2.filter(*temp_cloud);
		cloud_2_filt = temp_cloud;

		cout << "Filtered 2nd Cloud in " << time.toc() << "ms." << endl;

	}

	void basic_icp_process()
	{
		cout << "Starting ICP Process." << endl;
		time.tic();
		short max_iterations = 100;

		pcl::IterativeClosestPoint<PointT, PointT> icp;
		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance(0.10);
		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations(max_iterations);
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon(1e-8);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp.setEuclideanFitnessEpsilon(1);

		icp.setInputTarget(cloud_2_filt);
		PointCloudT::Ptr Final(new PointCloudT);
		*Final = *cloud_1_filt;

		for (short i = 1; i <= max_iterations; i++)
		{
			PCL_INFO("Iteration No. %d.\n", i);
			*cloud_1_filt = *Final;
			icp.setInputSource(cloud_1_filt);
			icp.align(*Final);
			cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
			
		}

		cout << icp.getFinalTransformation() << endl;
		cout << "ICP Process Finished in " << time.toc() << "ms." << endl;
		
		cout << "Enter name for cloud to be saved under: ";
		string s;
		cin >> s;
		stringstream ss;
		ss << s << ".pcd"; //

		cout << "Saving ICP Cloud." << endl;
		time.tic();
		io::savePCDFileBinary<PointT>(ss.str(), *Final);
		cout << "Cloud Saved as Binary in " << time.toc() << endl;
	}


};

#endif // !ICP_HEADER
