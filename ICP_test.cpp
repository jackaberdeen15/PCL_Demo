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