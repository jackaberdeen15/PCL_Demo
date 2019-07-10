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

//custom headers
#include <C:\Users\Jack\source\repos\pcl_visualizer\build\Useful_Functions.h>

using namespace pcl;
using namespace std;
using namespace chrono_literals;
//using namespace io;

typedef PointXYZRGBA T;

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

		cout << "Point Cloud before filtering: " << cloud->width * cloud->height << " data points (" << getFieldsList(*cloud) << ")." << endl;

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

#endif // !BASICGRAB

