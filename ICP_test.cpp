//File Used to practice making iterative closest point to combine point clouds

//general headers
#include <iostream>
#include <string>
#include <iomanip>

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

//custom headers


//names space declerations
using namespace std;
using namespace pcl;

//global decs
typedef PointXYZRGBA PointT;
typedef PointCloud<PointT> PointCloudT;
bool next_iteration = false;

//Code begins

void print4x4matrix(const Eigen::Matrix4d & matrix)
{
	cout << "Rotation matrix:" << endl;
	printf("    | %6.3f %6.3f %6.3f |\n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f |\n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f |\n", matrix(2, 0), matrix(2, 1), matrix(2, 2));

	cout << "Translation vector:" << endl;
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void newkeyboardEventOccurred(const visualization::KeyboardEvent& event, void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}

int tmain(int argc, char* argv[])
{
	//point clouds used with ICP
	PointCloudT::Ptr cloud_in(new PointCloudT); //original point cloud
	PointCloudT::Ptr cloud_tr(new PointCloudT); //transformed point cloud
	PointCloudT::Ptr cloud_icp(new PointCloudT);//ICP output point cloud
	/*
	//checking program arguments
	if (argc < 2)
	{
		cout << "Usage: " << argv[0] << "file.pcd number_of_icp_iterations" << endl;
		PCL_ERROR("Provide only one PCD file.\n");
		
		return -1;
	}
	*/
	int iterations = 1; //1 is the default

	if (argc > 2)
	{
		//if the user passed the number of iteration as an argument
		iterations = atoi(argv[2]);
		if (iterations < 1)
		{
			PCL_ERROR("Number of inital iterations must be >=1.\n");
			return -1;
		}
	}
	
	console::TicToc time;
	time.tic();

	cout << "Please enter the name of the file you want to load. (format= XXXXX.pcd). ";
	string cloudname;
	cin >> cloudname;

	if (io::loadPCDFile<PointT>(cloudname,*cloud_in) == -1)
	{
		PCL_ERROR("Couldnt read file.\n");
		return -1;
	}

	cout << "\nLoaded file." << cloudname << " (" << cloud_in->size() << " points) in " << time.toc() << " ms.\n" << endl;

	//defining a rotation matrix and translation vector
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

	//A rotation matrix
	double theta = M_PI / 8;//the angle of rotation in radians
	transformation_matrix(0, 0) = cos(theta);
	transformation_matrix(0, 1) = -sin(theta);
	transformation_matrix(1, 0) = sin(theta);
	transformation_matrix(1, 1) = cos(theta);

	//a translation on z axis (0.4 meters)
	transformation_matrix(2, 3) = 0.4;

	//display in terminal the transformation matrix
	cout << "applying this rigid transformation to: cloud_in -> cloud_icp" << endl;
	print4x4matrix(transformation_matrix);

	//executing the tranformation
	transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);

	//the iterative closest point algorithm
	time.tic();
	IterativeClosestPoint<PointT, PointT>icp;
	icp.setMaximumIterations(iterations);
	icp.setInputSource(cloud_icp);
	icp.setInputTarget(cloud_in);
	icp.align(*cloud_icp);
	icp.setMaximumIterations(1); //set this to 1 for the next call of .align()

	if (icp.hasConverged())
	{
		cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
		cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << endl;
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		print4x4matrix(transformation_matrix);
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		return -1;
	}

	//visualisation
	visualization::PCLVisualizer viewer("ICP Demo");

	//create two vetically seperated vieports
	int v1(0), v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	//the colour we will be using

	float bckgr_gray_level = 0.0; //black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	//original point cloud is white
	//visualization::PointCloudColorHandler<PointT> rgba(cloud_in);
	visualization::PointCloudColorHandlerRGBAField<PointT> rgba1(cloud_in);

	viewer.addPointCloud(cloud_in, rgba1, "cloud_in_v1", v1);
	viewer.addPointCloud(cloud_in, rgba1, "cloud_in_v2", v2);

	visualization::PointCloudColorHandlerRGBAField<PointT> rgba2(cloud_tr);

	viewer.addPointCloud(cloud_tr, rgba2, "cloud_tr_v1", v1);

	visualization::PointCloudColorHandlerRGBAField<PointT> rgba3(cloud_icp);

	viewer.addPointCloud(cloud_icp, rgba3, "cloud_icp_v2", v2);

	stringstream ss;
	ss << iterations;

	string iterations_cnt = "ICP Iterations = " + ss.str();

	viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

	// Set camera position and orientation
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // Visualiser window size

	// Register keyboard callback :
	viewer.registerKeyboardCallback(&newkeyboardEventOccurred, (void*)NULL);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		//the user pressed space
		if (next_iteration)
		{
			time.tic();
			icp.align(*cloud_icp);
			cout << "Applied 1 ICP itertion in " << time.toc() << " ms" << endl;

			if (icp.hasConverged())
			{
				printf("\033[11A");  // Go up 11 lines in terminal output.
				printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
				transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
				print4x4matrix(transformation_matrix);  // Print the transformation between original pose and current pose

				ss.str("");
				ss << iterations;
				std::string iterations_cnt = "ICP iterations = " + ss.str();
				viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
				viewer.updatePointCloud(cloud_icp, rgba3, "cloud_icp_v2");
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
				return (-1);
			}
		}
		next_iteration = false;
	}

	return 0;
}


int bmain(int argc, char** argv[])
{
	PointCloudT::Ptr cloud_orig(new PointCloudT);
	PointCloudT::Ptr cloud_out(new PointCloudT);
	PointCloudT::Ptr cloud_in(new PointCloudT);
	
	if (io::loadPCDFile<PointT>("cloud_1_0.pcd", *cloud_orig) == -1)
	{
		PCL_ERROR("Couldnt read file.\n");
		return -1;
	}
	
	PassThrough<PointT> pass;
	pass.setInputCloud(cloud_orig);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-1000.0, 1000.0);
	pass.filter(*cloud_in);

	*cloud_out = *cloud_in;

	cout << "size:" << cloud_out->points.size() << endl;
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
		cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
	cout << "Transformed " << cloud_in->points.size() << endl;

	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_out);
	PointCloudT::Ptr Final(new PointCloudT);
	icp.align(*Final);
	cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
	cout << icp.getFinalTransformation() << endl;

	visualization::CloudViewer viewer("Simple Cloud Viewer");
	if (!viewer.wasStopped())
		viewer.showCloud(Final);
	while (!viewer.wasStopped())
	{
		//boost::this_thread::sleep(boost::posix_time::seconds(1));
	}

	return (0);
}