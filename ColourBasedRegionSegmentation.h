#pragma once

#ifndef Col_Bas_Reg_Seg
#define Col_Bas_Reg_Seg

//General Headers
#include <iostream>
#include <string>
#include <iomanip>
#include <thread>
#include <math.h>
#include <vector>


//PCL Headers
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/file_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>

//Type Definitions
typedef PointXYZRGBA PointT;
typedef PointCloud<PointT> PointCloudT;

//names space declerations
using namespace std;
using namespace pcl;
using namespace chrono_literals;

class Col_Seg {

private:
	console::TicToc timer;

	PassThrough<PointT> pass;

	IndicesPtr indices;

	RegionGrowingRGB<PointT> reg;

	vector <PointIndices> clusters;
	
	PointCloudT::Ptr cloud;
	PointCloudT::Ptr coloured_cloud;

public:

	void input_cloud(PointCloudT::Ptr inp_cloud) { cloud = inp_cloud; }

	void set_segment_range(float upper_lim = 10.0, float lower_lim = 0.0, string axis="z")
	{
		cout << "Setting area to segment." << endl;
		timer.tic();
		IndicesPtr temp_indices(new vector <int>);
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 4.0);
		pass.filter(*temp_indices);
		cout << "Filtered cloud in " << timer.toc() << "ms." << endl;

		indices = temp_indices;
	}

	void segment(short min_clust_size = 40, float pcol_thresh = 4, float rcol_thresh = 4, float dist_thresh = 20)
	{
		cout << "Entered segment function." << endl;
		timer.tic();
		search::Search<PointT>::Ptr tree(new search::KdTree<PointT>);
		reg.setInputCloud(cloud);
		//reg.setIndices(indices);
		reg.setSearchMethod(tree);
		reg.setDistanceThreshold(dist_thresh);
		reg.setPointColorThreshold(pcol_thresh);
		reg.setRegionColorThreshold(rcol_thresh);
		reg.setMinClusterSize(min_clust_size);

		reg.extract(clusters);
		cout << "Extracted clusters in " << timer.toc() << "ms." << endl;

		coloured_cloud = reg.getColoredCloudRGBA();
	}

	void visualise()
	{
		visualization::CloudViewer viewer("Cluster viewer");
		viewer.showCloud(coloured_cloud);

		while (!viewer.wasStopped())
		{
			this_thread::sleep_for(100us);
		}
	}

	void save_cloud()
	{
		cout << "Enter name for cloud to be saved under: ";
		string s;
		cin >> s;
		stringstream ss;
		ss << s << ".pcd"; //
		timer.tic();
		cout << "Saving Cloud." << endl;
		io::savePCDFileBinary<PointT>(ss.str(), *coloured_cloud);
		cout << "Cloud Saved as Binary in " << timer.toc() << "ms." << endl;
	}

	vector<PointIndices> return_clusters() { return clusters; }

	void save_individual_clusters()
	{
		int j = 0;
		for (vector<PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)//iterates through all the clusters stored in vector
		{
			PointCloud<PointT>::Ptr cloud_cluster(new PointCloud<PointT>);
			for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) //iterates through all points in a cluster
				cloud_cluster->points.push_back(coloured_cloud->points[*pit]); //adds each point of the cluster at the end of the point cloud vector
			cloud_cluster->width = cloud_cluster->points.size(); 
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true; //no points are invalid

			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
			std::stringstream ss;
			ss << "cluster_" << j << ".pcd";
			io::savePCDFileBinary(ss.str(), *cloud_cluster); //saves the cloud of the current cluster
			j++;
		}
	}
};


#endif // !Col_Bas_Reg_Seg
