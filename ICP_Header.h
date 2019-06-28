#pragma once
#ifndef ICP_HEADER
#define ICP_HEADER

//general headers
#include <iostream>
#include <string>
#include <iomanip>
#include <thread>
#include <math.h>

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
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/console/parse.h>


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

	void filter_clouds(float uppr_lim=2.0, float lower_lim=-2.0)
	{
		cout << "Filtering Clouds." << endl;
		time.tic();
		PointCloudT::Ptr temp_cloud(new PointCloudT);
		PointCloudT::Ptr temp_cloud2(new PointCloudT);

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
		pass2.filter(*temp_cloud2);
		cloud_2_filt = temp_cloud2;

		cout << "Filtered 2nd Cloud in " << time.toc() << "ms." << endl;

	}

	void basic_icp_process()
	{
		cout << "Starting ICP Process." << endl;
		time.tic();
		short max_iterations = 120;
		
		pcl::IterativeClosestPoint<PointT, PointT> icp;
		// set the input and target
		icp.setInputSource(cloud_1_filt);
		icp.setInputTarget(cloud_2_filt);
		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance(5);
		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations(max_iterations);
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon(1e-9);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp.setEuclideanFitnessEpsilon(1);

		icp.setRANSACOutlierRejectionThreshold(0.05);
		icp.setRANSACIterations(max_iterations);


		PointCloudT::Ptr output_cloud(new PointCloudT);
				
		double fit_score;
		
		
		icp.align(*output_cloud);
		fit_score = icp.getFitnessScore();

		cout << icp.getFinalTransformation() << endl;
		cout << "ICP Process Finished in " << time.toc() << "ms.";
		cout << ", with a fitness score of " << fit_score << "." << endl;

		PointCloudT::Ptr combined_cloud(new PointCloudT);

		//transformPointCloud(*output_cloud, *output_cloud, icp.getFinalTransformation());
		
		*combined_cloud = *output_cloud;
		*combined_cloud += *cloud_2_filt;
		//concatenatePointCloud(output_cloud, cloud_2_filt, combined_cloud);
		
		cout << "Enter name for cloud to be saved under: ";
		string s;
		cin >> s;
		stringstream ss;
		ss << s << ".pcd"; //
		
		cout << "Saving ICP Cloud." << endl;
		time.tic();
		io::savePCDFileBinary<PointT>(ss.str(), *combined_cloud);
		cout << "Cloud Saved as Binary in " << time.toc() << endl;
	}


};

class CSTM_ICP_PIPE
{
private:
	

public:

	/*PointCloud<Normal>::Ptr get_normals(PointCloudT::Ptr cloud, double s_rad = 0.03)
	{
		// Create the normal estimation class, and pass the input dataset to it
		NormalEstimation<PointT, Normal> ne;
		ne.setInputCloud(cloud);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		search::KdTree<PointT>::Ptr tree(new search::KdTree<PointT>());
		ne.setSearchMethod(tree);

		// Output datasets
		PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>);

		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch(s_rad);

		// Compute the features
		ne.compute(*cloud_normals);

		return cloud_normals;
	}*/
	
	PointCloud<int> extract_keypoints(PointCloudT::Ptr cloud)
	{

		//Create RangeImage from the PointCloud
		float angularResolution = (float)(1.0f * (M_PI / 180.0f));  //   1.0 degree in radians
		float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 360.0 degree in radians
		float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
		Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
		RangeImage::CoordinateFrame coordinate_frame = RangeImage::CAMERA_FRAME;
		float noiseLevel = 0.00;
		float minRange = 0.0f;
		int borderSize = 1;

		PointCloud<PointXYZ>::Ptr xyz_cloud( new PointCloud<PointXYZ>);
		copyPointCloud(*cloud, *xyz_cloud);

		RangeImage rangeImage;
		rangeImage.createFromPointCloud(*xyz_cloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
		
		// Extract NARF keypoints
		RangeImageBorderExtractor range_image_border_extractor;
		NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
		narf_keypoint_detector.setRangeImage(&rangeImage);
		narf_keypoint_detector.getParameters().support_size = 0.1f;
		//narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
		//narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;

		PointCloud<int> keypoint_indices;
		narf_keypoint_detector.compute(keypoint_indices);
		cout << "Found " << keypoint_indices.points.size() << " key points.\n";

		return keypoint_indices;
	}

};


#endif // !ICP_HEADER
