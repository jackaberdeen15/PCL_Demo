#pragma once

#ifndef BASICGRAB
#define BASICGRAB

//general headers
#include <thread>

//pcl headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/common/time.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/point_types_conversion.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/crop_box.h>

//custom headers
#include <C:\Users\Jack\source\repos\pcl_visualizer\build\Useful_Functions.h>
//#include <C:\Users\Jack\source\repos\pcl_visualizer\build\Sphere_Detector_components.h>

using namespace pcl;
using namespace std;
using namespace chrono_literals;
//using namespace io;

typedef PointXYZRGBA T;
typedef PointCloud<T> PointCloudT;
typedef PointXYZHSV HSV;
typedef PointCloud<HSV> PointCloudHSV;
typedef PointXYZRGB PointRGB;
typedef PointCloud<PointRGB> PointCloudRGB;

//simply displays distance and fps to cmd window
class BasicGrabber
{
private:


public:
	PointCloud<T>::ConstPtr Grab_Cloud()
	{
		PointCloud<PointXYZRGBA>::Ptr sourceCloud(new PointCloud<PointXYZRGBA>);

		boost::function<void(const PointCloud<PointXYZRGBA>::ConstPtr&)> function = [&sourceCloud](const PointCloud<PointXYZRGBA>::ConstPtr &cloud)
		{
			copyPointCloud(*cloud, *sourceCloud);
		};

		// Create Kinect2Grabber
		Grabber* grabber = new io::OpenNI2Grabber();
		// Regist Callback Function
		grabber->registerCallback(function);
		// Start Retrieve Data
		grabber->start();
		boost::this_thread::sleep(boost::posix_time::seconds(1));
		// Stop Retrieve Data
		grabber->stop();
		cout << "The Cloud size: " << sourceCloud->size() << " points ..." << endl;

		return sourceCloud;
	}
	
};

//displays cloud to viewer, also distance and fps to cmd window
class BasicOpenNI2Processor
{
private:
	int num_s1 = 0;

	console::TicToc timer;

	void cloud_cb_(const PointCloud<T>::ConstPtr &cloud)
	{
		cout << "Entered callback 'View' " << std::endl;

		timer.tic();

		if (!viewer.wasStopped())
			viewer.showCloud(cloud);


		/*
		short save = 0;
		cout << "Save Cloud?\n1: Yes.\n2: No." << endl;
		cin >> save;

		if (save == 1)
		{
			string cloudname;
			cout << "Enter file name. (format= XXXXX.pcd). ";
			cin >> cloudname;
			io::savePCDFileASCII(cloudname, *cloud);
			cout << "Saved " << cloud->points.size() << " data points to " << cloudname << "." << endl;
		}
		*/

		cout << "cloud_cb completed in " << timer.toc() << "ms." << endl;
	}

	void save_cloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
		
		/*cout << "Removing statistical outliers from cloud." << endl;
		timer.tic();
		PointCloud<T>::Ptr filt_cloud(new PointCloud<T>);
		pcl::StatisticalOutlierRemoval<T> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(50);
		sor.setStddevMulThresh(1.0);
		sor.filter(*filt_cloud);		
		cout << "Outliers removed in " << timer.toc() << "ms." << endl;*/
		
		timer.tic();
		stringstream s;
		s << "cld_" << num_s1 << ".pcd"; //
		cout << "Entered callback 'Save' " << std::endl;
		io::savePCDFileBinary(s.str(), *cloud);
		cout << "file " << s.str() << " saved after " << timer.toc() << "ms." << endl;

		cout << "Converting cloud into Voxel Grid." << endl;
		timer.tic();
		PointCloudT::Ptr cloud_filtered(new PointCloudT);
		PointCloudT::Ptr cloud_filtered2(new PointCloudT);

		cout << "Point Cloud before filtering: " << cloud->width * cloud->height << " data points (" << getFieldsList(*cloud) << ")." << endl;

		PassThrough<PointT> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 2.0);
		pass.filter(*cloud_filtered2);


		VoxelGrid<PointT> vxl;
		vxl.setInputCloud(cloud);
		//vxl.setFilterFieldName("z");
		//vxl.setFilterLimits(0.0, 2.1);
		vxl.setLeafSize(0.01f, 0.01f, 0.01f);
		vxl.filter(*cloud_filtered);

		cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
			<< " data points (" << getFieldsList(*cloud_filtered) << ")." << endl;

		cout << "Cloud converted in " << timer.toc() << "ms." << endl;
		timer.tic();
		stringstream ss;
		ss << "vxl_" << num_s1++ << ".pcd"; //
		io::savePCDFileBinary(ss.str(), *cloud_filtered);
		cout << "file " << ss.str() << " saved after " << timer.toc() << "ms." << endl;

		cout << "Press enter to continue." << endl;
		ReadLastCharOfLine();
	}

public:
	BasicOpenNI2Processor() : viewer("PCL OpenNI Viewer") {}

	int run()
	{
		cout << "Entered run()." << endl;

		//create new grabber
		Grabber* interface = new io::OpenNI2Grabber();

		cout << "setting up callbacks." << endl;
		//make callback function from member function
		boost::function<void(const PointCloud<T>::ConstPtr&) > f = boost::bind(&BasicOpenNI2Processor::cloud_cb_, this, _1);

		boost::function<void(const PointCloud<T>::ConstPtr&) > f_save = boost::bind(&BasicOpenNI2Processor::save_cloud, this, _1);
		
		cout << "registering callbacks." << endl;
		//connect callback function for desired signal
		boost::signals2::connection c = interface->registerCallback(f);

		boost::signals2::connection c_save = interface->registerCallback(f_save);

		//boost::signals2::connection c_save2 = interface->
		cout << "starting interface" << endl;
		//start recieving point clouds
		interface->start();

		//wait until user quits with ctrl
		while (!viewer.wasStopped()) { boost::this_thread::sleep(boost::posix_time::seconds(1)); }

		//stop the grabber
		interface->stop();

		return num_s1;
	}

	void keyboard_event(const visualization::KeyboardEvent &event, void* viewer_void)
	{
		visualization::CloudViewer *viewer = static_cast<visualization::CloudViewer *>(viewer_void);
		if (event.getKeySym() == "s" && event.keyDown())
		{
			cout << "Would have saved." << endl;
		}
	}
	
	visualization::CloudViewer viewer;

};

//displays cloud to viewer, also distance and fps to cmd window
class Colour_Filter_Sphere
{
private:
	int num_s1 = 0;
	bool object_located = false;
	PointXYZ previous_centroid;
	float search_size = 0.25;
	int lost_counter = 0;

	console::TicToc timer;
	
	void cloud_cb_(const PointCloud<T>::ConstPtr &cloud)
	{
		cout << "\n\nEntered callback 'View' " << endl;

		PointCloudT::Ptr cloud_filtered(new PointCloudT);
		PointCloudHSV::Ptr cloud_hsv(new PointCloudHSV);

		if (object_located == false)
		{
			cout << "First loop. Filtering loud by distance (0-2m)." << endl;
			timer.tic();
			PassThrough<T> pass;
			pass.setInputCloud(cloud);
			pass.setFilterFieldName("z");
			pass.setFilterLimits(0.0, 2.0);
			pass.filter(*cloud_filtered);

		}
		else
		{
			cout << "Not first loop. Filtering around previous centroid " << previous_centroid << "." << endl;
			timer.tic();

			CropBox<T> boxFilter;
			Eigen::Vector4f min(previous_centroid.x - search_size, previous_centroid.y - search_size, previous_centroid.z - search_size, 1.0);
			Eigen::Vector4f max(previous_centroid.x + search_size, previous_centroid.y + search_size, previous_centroid.z + search_size, 1.0);

			boxFilter.setMin(min);
			boxFilter.setMax(max);
			boxFilter.setInputCloud(cloud);
			boxFilter.filter(*cloud_filtered);
		}
		cout << "Cloud now consists of " << cloud_filtered->size() << " Points .Cloud filtered in " << timer.toc() << "ms." << endl;

		cout << "Downsampling cloud using voxel grid." << endl;
		timer.tic();
		VoxelGrid<PointT> vxl;
		float leaf_size = 0.01f;
		vxl.setInputCloud(cloud_filtered);
		vxl.setLeafSize(leaf_size, leaf_size, leaf_size);
		vxl.filter(*cloud_filtered);
		cout << "Cloud now consists of " << cloud_filtered->size() << " Points .Cloud filtered in " << timer.toc() << "ms." << endl;


		cout << "Converting from RGBA to HSV." << endl;
		timer.tic();

		//copyPointCloud(*cloud_filtered, *cloud_hsv);
		PointCloudXYZRGBAtoXYZHSV(*cloud_filtered, *cloud_hsv);

		cout << "Time taken to convert was " << timer.toc() << "ms." << endl;

		cout << "Starting Colour Filtering.\nSize of unfiltered cloud is " << cloud_hsv->size() << endl;
		timer.tic();

		PointCloudHSV::Ptr cloud_hsv_filtered(new PointCloudHSV);
		PointCloudHSV::Ptr blue_cloud(new PointCloudHSV);
		/*PointCloudHSV::Ptr green_cloud(new PointCloudHSV);
		PointCloudHSV::Ptr yellow_cloud(new PointCloudHSV);
		PointCloudHSV::Ptr pink_cloud(new PointCloudHSV);*/


		short blue_h_upr_lim = 299;
		short blue_h_lwr_lim = 239;
		/*short green_h_upr_lim = 180;
		short green_h_lwr_lim = 120;
		short yellow_h_upr_lim = 119;
		short yellow_h_lwr_lim = 60;
		short pink_h_upr_lim = 359;
		short pink_h_lwr_lim = 300;*/

		double saturation_min = 0.4;
		double value_min = 0.4;

		//tracking::HSVColorCoherence colcoh;
		for (int i = 0; i < cloud_hsv->size(); ++i)
		{
			if (cloud_hsv->points[i].s >= saturation_min && cloud_hsv->points[i].v >= value_min)
			{
				if (cloud_hsv->points[i].h >= blue_h_lwr_lim && cloud_hsv->points[i].h < blue_h_upr_lim)
				{
					blue_cloud->points.push_back(cloud_hsv->points[i]);
				}
				/*else if (cloud_hsv->points[i].h >= green_h_lwr_lim && cloud_hsv->points[i].h < green_h_upr_lim)
				{
					green_cloud->points.push_back(cloud_hsv->points[i]);
				}
				else if (cloud_hsv->points[i].h >= yellow_h_lwr_lim && cloud_hsv->points[i].h < yellow_h_upr_lim)
				{
					yellow_cloud->points.push_back(cloud_hsv->points[i]);
				}
				else if (cloud_hsv->points[i].h >= pink_h_lwr_lim && cloud_hsv->points[i].h < pink_h_upr_lim)
				{
					pink_cloud->points.push_back(cloud_hsv->points[i]);
				}*/
			}
		}
		//cout << "Blue cloud has " << blue_cloud->size() << " points." << endl;
		//cout << "Green cloud has " << green_cloud->size() << " points." << endl;
		//cout << "Yellow cloud has " << yellow_cloud->size() << " points." << endl;
		//cout << "Pink cloud has " << pink_cloud->size() << " points." << endl;

		int blue = blue_cloud->size();
		/*int green = green_cloud->size();
		int yellow = yellow_cloud->size();
		int pink = pink_cloud->size();*/

		*cloud_hsv_filtered = *blue_cloud;

		/*if (blue > green&&blue > yellow&&blue > pink)
		{
			*cloud_hsv_filtered = *blue_cloud;
			cout << "Blue cloud is largest with " << blue_cloud->size() << " points." << endl;
		}
		else if (green > yellow && green > pink)
		{
			*cloud_hsv_filtered = *green_cloud;
			cout << "Green cloud is largest with " << green_cloud->size() << " points." << endl;
		}
		else if (yellow > pink)
		{
			*cloud_hsv_filtered = *yellow_cloud;
			cout << "Yellow cloud is largest with " << yellow_cloud->size() << " points." << endl;
		}
		else
		{
			*cloud_hsv_filtered = *pink_cloud;
			cout << "Pink cloud is largest with " << pink_cloud->size() << " points." << endl;
		}*/



		cloud_hsv_filtered->width = cloud_hsv_filtered->size();
		cloud_hsv_filtered->height = 1;
		cloud_hsv_filtered->resize(cloud_hsv_filtered->width * cloud_hsv_filtered->height);
		cout << "Colour filtering completed in " << timer.toc() << "ms.\nSize of filtered Cloud is " << cloud_hsv_filtered->size() << endl;

		PointCloudRGB::Ptr cloud_init_RGB(new PointCloudRGB);


		cout << "Converting back to RGBA." << endl;
		timer.tic();

		for (int i = 0; i < cloud_hsv_filtered->size(); ++i)
		{
			PointXYZRGB temp_point;

			PointXYZHSVtoXYZRGB(cloud_hsv_filtered->points[i], temp_point);
			//cout << "Temp point : " << temp_point << endl;
			cloud_init_RGB->points.push_back(temp_point);
		}

		cloud_init_RGB->width = cloud_init_RGB->size();
		cloud_init_RGB->height = 1;
		cloud_init_RGB->resize(cloud_init_RGB->width * cloud_init_RGB->height);

		cout << "Cloud converted in " << timer.toc() << "ms.\nSize of final cloud is " << cloud_init_RGB->size() << endl;

		cout << "Detecting all circles in filtered image." << endl;
		cout << "Calculating surface normals" << endl;
		timer.tic();

		// Create the normal estimation class, and pass the input dataset to it
		NormalEstimationOMP<PointRGB, Normal> ne;
		ne.setInputCloud(cloud_init_RGB);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		search::KdTree<PointRGB>::Ptr tree(new search::KdTree<PointRGB>());
		ne.setSearchMethod(tree);

		// Output datasets
		PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>);

		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch(0.02);

		// Compute the features
		ne.compute(*cloud_normals);

		cout << "Normals computed in " << timer.toc() << "ms." << endl;

		cout << "Performing RANSAC to find spheres." << endl;
		timer.tic();
		PointIndices circle_indices;
		ModelCoefficients::Ptr coefficients(new ModelCoefficients);
		SACSegmentationFromNormals<PointRGB, Normal> segmentation;

		segmentation.setInputCloud(cloud_init_RGB);
		segmentation.setInputNormals(cloud_normals);
		segmentation.setModelType(SACMODEL_SPHERE);
		//segmentation.setModelType(SACMODEL_CIRCLE3D);
		segmentation.setMethodType(SAC_RANSAC);
		segmentation.setDistanceThreshold(0.01);
		segmentation.setOptimizeCoefficients(true);
		segmentation.setRadiusLimits(0.01, 0.04);
		segmentation.setEpsAngle(15 / (180 / 3.141592654));
		segmentation.setMaxIterations(40);

		segmentation.segment(circle_indices, *coefficients);

		CentroidPoint<PointRGB> centroid;


		if (circle_indices.indices.size() == 0)
		{
			cout << "No Sphere was deteted with RANSAC. " << timer.toc() << "ms elapsed." << endl;
			lost_counter++;
			if (lost_counter > 10)
			{
				object_located = false;
			}
		}
		else
		{
			cout << "RANSAC detected a circles with " << circle_indices.indices.size() << " points. " << timer.toc() << "ms elapsed." << endl;
			for (int i = 0; i < circle_indices.indices.size(); ++i)
			{
				cloud_init_RGB->points[circle_indices.indices[i]].r = 0;
				cloud_init_RGB->points[circle_indices.indices[i]].g = 255;
				cloud_init_RGB->points[circle_indices.indices[i]].b = 0;

				centroid.add(cloud_init_RGB->points[circle_indices.indices[i]]);
			}
			PointXYZ sphere_centroid;
			centroid.get(sphere_centroid);

			cout << "Centroid of the sphere is " << sphere_centroid << "." << endl;
			previous_centroid = sphere_centroid;

			object_located = true;;
		}

		int index;
		PointCloudRGB border;

		for (double x = previous_centroid.x - search_size; x <= previous_centroid.x + search_size; x += 0.01)
		{
			for (double y = previous_centroid.y - search_size; y <= previous_centroid.y + search_size; y += 0.01)
			{
				if ((x <= previous_centroid.x - search_size + 0.01) || (x >= previous_centroid.x + search_size - 0.01) || (y <= previous_centroid.y - search_size + 0.01) || (y >= previous_centroid.y + search_size - 0.01))
				{
					PointRGB point;
					point.x = x;
					point.y = y;
					point.z = previous_centroid.z;
					point.r = 255;
					point.g = 0;
					point.b = 0;
					border.points.push_back(point);
				}
			}
		}

		border.width = border.size();
		border.height = 1;
		border.resize(border.width * border.height);

		*cloud_init_RGB += border;

		if (!viewer.wasStopped())
		{
			viewer.showCloud(cloud_init_RGB);
		}


	}

public:
	Colour_Filter_Sphere() : viewer("PCL OpenNI Viewer") {}
	
	int run()
	{
		cout << "Entered run()." << endl;

		//create new grabber
		Grabber* interface = new io::OpenNI2Grabber();

		cout << "setting up callbacks." << endl;
		//make callback function from member function
		boost::function<void(const PointCloud<T>::ConstPtr&) > f = boost::bind(&Colour_Filter_Sphere::cloud_cb_, this, _1);

		//boost::function<void(const PointCloud<T>::ConstPtr&) > f_save = boost::bind(&BasicOpenNI2Processor::save_cloud, this, _1);

		cout << "registering callbacks." << endl;
		//connect callback function for desired signal
		boost::signals2::connection c = interface->registerCallback(f);

		//boost::signals2::connection c_save = interface->registerCallback(f_save);

		//boost::signals2::connection c_save2 = interface->
		cout << "starting interface" << endl;
		//start recieving point clouds
		interface->start();

		//wait until user quits with ctrl
		while (!viewer.wasStopped()) { boost::this_thread::sleep(boost::posix_time::seconds(1)); }

		//stop the grabber
		interface->stop();

		return num_s1;
	}

	visualization::CloudViewer viewer;

	//visualization::PCLVisualizer viewer;
	

};


class Colour_Filter_Circles
{
private:
	int num_s1 = 0;
	bool object_located = false;
	PointXYZ previous_centroid;
	float search_size = 0.25;
	int lost_counter = 0;

	console::TicToc timer;

	void cloud_cb_(const PointCloud<T>::ConstPtr &cloud)
	{
		cout << "\n\nEntered callback 'View' " << endl;

		PointCloudT::Ptr cloud_filtered(new PointCloudT);
		PointCloudHSV::Ptr cloud_hsv(new PointCloudHSV);

		if (object_located == false)
		{
			cout << "First loop. Filtering loud by distance (0-2m)." << endl;
			timer.tic();
			PassThrough<T> pass;
			pass.setInputCloud(cloud);
			pass.setFilterFieldName("z");
			pass.setFilterLimits(0.0, 2.0);
			pass.filter(*cloud_filtered);

		}
		else
		{
			cout << "Not first loop. Filtering around previous centroid " << previous_centroid << "." << endl;
			timer.tic();

			CropBox<T> boxFilter;
			Eigen::Vector4f min(previous_centroid.x - search_size, previous_centroid.y - search_size, previous_centroid.z - search_size, 1.0);
			Eigen::Vector4f max(previous_centroid.x + search_size, previous_centroid.y + search_size, previous_centroid.z + search_size, 1.0);

			boxFilter.setMin(min);
			boxFilter.setMax(max);
			boxFilter.setInputCloud(cloud);
			boxFilter.filter(*cloud_filtered);
		}
		cout << "Cloud now consists of " << cloud_filtered->size() << " Points .Cloud filtered in " << timer.toc() << "ms." << endl;

		cout << "Downsampling cloud using voxel grid." << endl;
		timer.tic();
		VoxelGrid<PointT> vxl;
		float leaf_size = 0.01f;
		vxl.setInputCloud(cloud_filtered);
		vxl.setLeafSize(leaf_size, leaf_size, leaf_size);
		vxl.filter(*cloud_filtered);
		cout << "Cloud now consists of " << cloud_filtered->size() << " Points .Cloud filtered in " << timer.toc() << "ms." << endl;


		cout << "Converting from RGBA to HSV." << endl;
		timer.tic();

		//copyPointCloud(*cloud_filtered, *cloud_hsv);
		PointCloudXYZRGBAtoXYZHSV(*cloud_filtered, *cloud_hsv);

		cout << "Time taken to convert was " << timer.toc() << "ms." << endl;

		cout << "Starting Colour Filtering.\nSize of unfiltered cloud is " << cloud_hsv->size() << endl;
		timer.tic();

		PointCloudHSV::Ptr cloud_hsv_filtered(new PointCloudHSV);
		PointCloudHSV::Ptr blue_cloud(new PointCloudHSV);
		PointCloudHSV::Ptr green_cloud(new PointCloudHSV);
		PointCloudHSV::Ptr yellow_cloud(new PointCloudHSV);
		PointCloudHSV::Ptr pink_cloud(new PointCloudHSV);


		short blue_h_upr_lim = 299;
		short blue_h_lwr_lim = 239;
		short green_h_upr_lim = 180;
		short green_h_lwr_lim = 120;
		short yellow_h_upr_lim = 119;
		short yellow_h_lwr_lim = 60;
		short pink_h_upr_lim = 359;
		short pink_h_lwr_lim = 300;

		double saturation_min = 0.4;
		double value_min = 0.4;
		
		//tracking::HSVColorCoherence colcoh;
		for (int i = 0; i < cloud_hsv->size(); ++i)
		{
			if (cloud_hsv->points[i].s >= saturation_min && cloud_hsv->points[i].v >= value_min)
			{
				if (cloud_hsv->points[i].h >= blue_h_lwr_lim && cloud_hsv->points[i].h < blue_h_upr_lim)
				{
					blue_cloud->points.push_back(cloud_hsv->points[i]);
				}
				else if (cloud_hsv->points[i].h >= green_h_lwr_lim && cloud_hsv->points[i].h < green_h_upr_lim)
				{
					green_cloud->points.push_back(cloud_hsv->points[i]);
				}
				else if (cloud_hsv->points[i].h >= yellow_h_lwr_lim && cloud_hsv->points[i].h < yellow_h_upr_lim)
				{
					yellow_cloud->points.push_back(cloud_hsv->points[i]);
				}
				else if (cloud_hsv->points[i].h >= pink_h_lwr_lim && cloud_hsv->points[i].h < pink_h_upr_lim)
				{
					pink_cloud->points.push_back(cloud_hsv->points[i]);
				}
			}
		}
		//cout << "Blue cloud has " << blue_cloud->size() << " points." << endl;
		//cout << "Green cloud has " << green_cloud->size() << " points." << endl;
		//cout << "Yellow cloud has " << yellow_cloud->size() << " points." << endl;
		//cout << "Pink cloud has " << pink_cloud->size() << " points." << endl;

		int blue = blue_cloud->size();
		int green = green_cloud->size();
		int yellow = yellow_cloud->size();
		int pink = pink_cloud->size();

		if (blue > green&&blue > yellow&&blue > pink)
		{
			*cloud_hsv_filtered = *blue_cloud;
			cout << "Blue cloud is largest with " << blue_cloud->size() << " points." << endl;
		}
		else if (green > yellow && green > pink)
		{
			*cloud_hsv_filtered = *green_cloud;
			cout << "Green cloud is largest with " << green_cloud->size() << " points." << endl;
		}
		else if (yellow > pink)
		{
			*cloud_hsv_filtered = *yellow_cloud;
			cout << "Yellow cloud is largest with " << yellow_cloud->size() << " points." << endl;
		}
		else
		{
			*cloud_hsv_filtered = *pink_cloud;
			cout << "Pink cloud is largest with " << pink_cloud->size() << " points." << endl;
		}

		

		cloud_hsv_filtered->width = cloud_hsv_filtered->size();
		cloud_hsv_filtered->height = 1;
		cloud_hsv_filtered->resize(cloud_hsv_filtered->width * cloud_hsv_filtered->height);
		cout << "Colour filtering completed in " << timer.toc() << "ms.\nSize of filtered Cloud is " << cloud_hsv_filtered->size() << endl;

		PointCloudRGB::Ptr cloud_init_RGB(new PointCloudRGB);


		cout << "Converting back to RGBA." << endl;
		timer.tic();

		for (int i = 0; i < cloud_hsv_filtered->size(); ++i)
		{
			PointXYZRGB temp_point;

			PointXYZHSVtoXYZRGB(cloud_hsv_filtered->points[i], temp_point);
			//cout << "Temp point : " << temp_point << endl;
			cloud_init_RGB->points.push_back(temp_point);
		}

		cloud_init_RGB->width = cloud_init_RGB->size();
		cloud_init_RGB->height = 1;
		cloud_init_RGB->resize(cloud_init_RGB->width * cloud_init_RGB->height);

		cout << "Cloud converted in " << timer.toc() << "ms.\nSize of final cloud is " << cloud_init_RGB->size() << endl;

		cout << "Detecting all circles in filtered image." << endl;
		cout << "Calculating surface normals" << endl;
		timer.tic();

		// Create the normal estimation class, and pass the input dataset to it
		NormalEstimationOMP<PointRGB, Normal> ne;
		ne.setInputCloud(cloud_init_RGB);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		search::KdTree<PointRGB>::Ptr tree(new search::KdTree<PointRGB>());
		ne.setSearchMethod(tree);

		// Output datasets
		PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>);

		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch(0.02);

		// Compute the features
		ne.compute(*cloud_normals);

		cout << "Normals computed in " << timer.toc() << "ms." << endl;

		cout << "Performing RANSAC to find spheres." << endl;
		timer.tic();
		PointIndices circle_indices;
		ModelCoefficients::Ptr coefficients(new ModelCoefficients);
		SACSegmentationFromNormals<PointRGB, Normal> segmentation;

		segmentation.setInputCloud(cloud_init_RGB);
		segmentation.setInputNormals(cloud_normals);
		//segmentation.setModelType(SACMODEL_SPHERE);
		segmentation.setModelType(SACMODEL_CIRCLE3D);
		segmentation.setMethodType(SAC_RANSAC);
		segmentation.setDistanceThreshold(0.01);
		segmentation.setOptimizeCoefficients(true);
		segmentation.setRadiusLimits(0.01, 0.04);
		segmentation.setEpsAngle(15 / (180 / 3.141592654));
		segmentation.setMaxIterations(40);

		segmentation.segment(circle_indices, *coefficients);

		CentroidPoint<PointRGB> centroid;


		if (circle_indices.indices.size() == 0)
		{
			cout << "No Sphere was deteted with RANSAC. " << timer.toc() << "ms elapsed." << endl;
			lost_counter++;
			if (lost_counter > 10)
			{
				object_located = false;
			}
		}
		else
		{
			cout << "RANSAC detected a circles with " << circle_indices.indices.size() << " points. " << timer.toc() << "ms elapsed." << endl;
			for (int i = 0; i < circle_indices.indices.size(); ++i)
			{
				cloud_init_RGB->points[circle_indices.indices[i]].r = 0;
				cloud_init_RGB->points[circle_indices.indices[i]].g = 255;
				cloud_init_RGB->points[circle_indices.indices[i]].b = 0;

				centroid.add(cloud_init_RGB->points[circle_indices.indices[i]]);
			}
			PointXYZ circle_centroid;
			centroid.get(circle_centroid);

			cout << "Centroid of the sphere is " << circle_centroid << "." << endl;
			previous_centroid = circle_centroid;

			object_located = true;;
		}

		int index;
		PointCloudRGB border;

		for (double x = previous_centroid.x - search_size; x <= previous_centroid.x + search_size; x += 0.01)
		{
			for (double y = previous_centroid.y - search_size; y <= previous_centroid.y + search_size; y += 0.01)
			{
				if ((x <= previous_centroid.x - search_size + 0.01) || (x >= previous_centroid.x + search_size - 0.01) || (y <= previous_centroid.y - search_size + 0.01) || (y >= previous_centroid.y + search_size - 0.01))
				{
					PointRGB point;
					point.x = x;
					point.y = y;
					point.z = previous_centroid.z;
					point.r = 255;
					point.g = 0;
					point.b = 0;
					border.points.push_back(point);
				}
			}
		}

		border.width = border.size();
		border.height = 1;
		border.resize(border.width * border.height);

		*cloud_init_RGB += border;

		if (!viewer.wasStopped())
		{
			viewer.showCloud(cloud_init_RGB);
		}


	}

public:
	Colour_Filter_Circles() : viewer("PCL OpenNI Viewer") {}

	int run()
	{
		cout << "Entered run()." << endl;

		//create new grabber
		Grabber* interface = new io::OpenNI2Grabber();

		cout << "setting up callbacks." << endl;
		//make callback function from member function
		boost::function<void(const PointCloud<T>::ConstPtr&) > f = boost::bind(&Colour_Filter_Circles::cloud_cb_, this, _1);

		//boost::function<void(const PointCloud<T>::ConstPtr&) > f_save = boost::bind(&BasicOpenNI2Processor::save_cloud, this, _1);

		cout << "registering callbacks." << endl;
		//connect callback function for desired signal
		boost::signals2::connection c = interface->registerCallback(f);

		//boost::signals2::connection c_save = interface->registerCallback(f_save);

		//boost::signals2::connection c_save2 = interface->
		cout << "starting interface" << endl;
		//start recieving point clouds
		interface->start();

		//wait until user quits with ctrl
		while (!viewer.wasStopped()) { boost::this_thread::sleep(boost::posix_time::seconds(1)); }

		//stop the grabber
		interface->stop();

		return num_s1;
	}

	visualization::CloudViewer viewer;

	//visualization::PCLVisualizer viewer;


};

#endif // !BASICGRAB

