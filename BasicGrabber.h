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

	void cloud_cb_(const PointCloud<T>::ConstPtr &cloud)
	{
		cout << "Entered callback 'View' " << std::endl;

		static unsigned count = 0;
		static double last = getTime();
		if (++count == 10)
		{
			double now = getTime();
			cout << "Distance of center pixel :" << cloud->points[(cloud->width >> 1) * (cloud->height + 1)].z << " m. Average framerate: " << double(count) / double(now - last) << " fps" << std::endl;
			count = 0;
			last = now;
		}

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
	}

	void save_cloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
		stringstream s;
		s << "cloud_1_" << num_s1++ << ".pcd"; //
		cout << "Entered callback 'Save' " << std::endl;
		io::savePCDFileASCII(s.str(), *cloud);
	}

public:
	BasicOpenNI2Processor() : viewer("PCL OpenNI Viewer") {}

	int run()
	{
		//create new grabber
		Grabber* interface = new io::OpenNI2Grabber();

		//make callback function from member function
		boost::function<void(const PointCloud<T>::ConstPtr&)>f = boost::bind(&BasicOpenNI2Processor::cloud_cb_, this, _1);

		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) > f_save = boost::bind(&BasicOpenNI2Processor::save_cloud, this, _1);
		
		//connect callback function for desired signal
		boost::signals2::connection c = interface->registerCallback(f);

		boost::signals2::connection c_save = interface->registerCallback(f_save);

		//boost::signals2::connection c_save2 = interface->

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

