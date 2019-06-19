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
#include <pcl/filters/voxel_grid.h>

//custom headers
#include <C:\Users\Jack\source\repos\pcl_visualizer\build\ICP_Header.h>
#include <C:\Users\Jack\source\repos\pcl_visualizer\build\BasicGrabber.h>
#include <C:\Users\Jack\source\repos\pcl_visualizer\build\Useful_Functions.h>

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


int main(int argc, char** argv[])
{
	short choice = 1;
	
	while (choice != 0)
	{
		cout << "Select what you would like to do?" << endl;
		cout << "1: Perform ICP." << endl;
		cout << "2: View a Cloud." << endl;
		cout << "3: Get new Clouds." << endl;
		cout << "4: Convert existing cloud into a voxel grid(IN PROGRESS)." << endl;
		cout << "0: Exit Program." << endl;

		short choice;
		cin >> choice;

		switch (choice)
		{
		case 1:
		{
			ICP icp_proc;

			cout << "Enter name of first cloud to load: ";
			string s1;
			cin >> s1;
			stringstream ss1;
			ss1 << s1 << ".pcd"; //

			icp_proc.load_pc_1(ss1.str());

			cout << "Enter name of second cloud to load: ";
			string s2;
			cin >> s2;
			stringstream ss2;
			ss2 << s2 << ".pcd"; //

			icp_proc.load_pc_2(ss2.str());

			icp_proc.filter_clouds();
			icp_proc.basic_icp_process();

			break;
		}
		case 2:
		{
			PointCloudT::Ptr loaded_cloud(new PointCloudT);

			cout << "Enter name of the processed cloud to load: ";
			string sf;
			cin >> sf;
			stringstream ssf;
			ssf << sf << ".pcd";

			io::loadPCDFile(ssf.str(), *loaded_cloud);

			visualization::CloudViewer viewer("Simple Cloud Viewer");

			if (!viewer.wasStopped())
				viewer.showCloud(loaded_cloud);
			while (!viewer.wasStopped())
			{
				//boost::this_thread::sleep(boost::posix_time::seconds(1));
			}

			break;
		}
		case 3:
		{
			BasicOpenNI2Processor v;
			v.run();
		}
		case 4:
		{
			//cout << "Not yet implemented." << endl;
			
			PointCloudT::Ptr loaded_cloud(new PointCloudT);
			PointCloudT::Ptr cloud_filtered(new PointCloudT);

			cout << "Enter name of the cloud to load: ";
			string sf;
			cin >> sf;
			stringstream ssf;
			ssf << sf << ".pcd";

			io::loadPCDFile(ssf.str(), *loaded_cloud);
			
			cout << "Point Cloud before filtering: " << loaded_cloud->width * loaded_cloud->height << " data points (" << getFieldsList(*loaded_cloud) << ")." << endl;

			VoxelGrid<PointT> sor;
			sor.setInputCloud(loaded_cloud);
			sor.setLeafSize(0.02f, 0.02f, 0.02f);
			sor.filter(*cloud_filtered);

			cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
				<< " data points (" << getFieldsList(*cloud_filtered) << ")." << endl;

			cout << "Enter name of the cloud to save: ";
			string s1;
			cin >> s1;
			stringstream ss1;
			ss1 << s1 << ".pcd";

			io::savePCDFileBinary(ss1.str(), *cloud_filtered);
								
			break;
		}
		default:
			break;
		break;
		}
	}
	return (0);
}