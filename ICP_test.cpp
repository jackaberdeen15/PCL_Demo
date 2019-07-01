//File Used to practice making iterative closest point to combine point clouds

//general headers
#include <iostream>
#include <string>
#include <iomanip>
#include <vector>

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
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>

//custom headers
#include <C:\Users\Jack\source\repos\pcl_visualizer\build\ICP_Header.h>
#include <C:\Users\Jack\source\repos\pcl_visualizer\build\BasicGrabber.h>
#include <C:\Users\Jack\source\repos\pcl_visualizer\build\Useful_Functions.h>
#include <C:\Users\Jack\source\repos\pcl_visualizer\build\ColourBasedRegionSegmentation.h>

//names space declerations
using namespace std;
using namespace pcl;
using namespace chrono_literals;

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
		cout << "5: Region growing protoype." << endl;
		cout << "6: Custom ICP pipeline prototype." << endl;
		cout << "7: Normal distribution transform." << endl;
		cout << "8: Euclidean Cluster Extraction." << endl;
		cout << "0: Exit Program." << endl;

		short choice;
		cin >> choice;

		switch (choice)
		{
		case 0:
		{
			cout << "Ending program." << endl;
			return 0;
			break;
		}
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
		case 5:
		{
			//search::Search <PointT>::Ptr tree(new search::KdTree<PointT>);

			cout << "By colour or surface normals?" << endl;
			cout << "1: Colour." << endl;
			cout << "2: Surface Normals." << endl;
			short choice2 = 0;
			cin >> choice2;

			PointCloudT::Ptr cloud(new PointCloudT);
			cout << "Enter name of point cloud to load: ";
			string s1;
			cin >> s1;
			stringstream ss1;
			ss1 << s1 << ".pcd"; //

			io::loadPCDFile<PointT>(ss1.str(), *cloud);
			
			switch (choice2)
			{
				case 1:
				{
					Col_Seg colour_seg;
					colour_seg.input_cloud(cloud);
					colour_seg.set_segment_range();
					colour_seg.segment();
					
					//vector<PointIndices> clusters = colour_seg.return_clusters();
					
					colour_seg.visualise();
					colour_seg.save_cloud();

					colour_seg.save_individual_clusters();
					
					break;
				}

				case 2:
				{
					cout << "Entering Normal Based Region Segmentation." << endl;

					search::Search<PointT>::Ptr tree(new search::KdTree<PointT>);
					PointCloud <Normal>::Ptr normals(new pcl::PointCloud <Normal>);
					NormalEstimation<PointT, Normal> normal_estimator;
					normal_estimator.setSearchMethod(tree);
					normal_estimator.setInputCloud(cloud);
					normal_estimator.setKSearch(50);
					normal_estimator.compute(*normals);

					IndicesPtr indices(new vector <int>);
					PassThrough<PointT> pass;
					pass.setInputCloud(cloud);
					pass.setFilterFieldName("z");
					pass.setFilterLimits(0.0, 3.0);
					pass.filter(*indices);

					float curv_thresh = 0.0;
					cout << "Insert curvature threshold: ";
					cin >> curv_thresh;

					RegionGrowing<PointT, Normal> reg;
					reg.setMinClusterSize(40);
					reg.setMaxClusterSize(1000000);
					reg.setSearchMethod(tree);
					reg.setNumberOfNeighbours(30);
					reg.setInputCloud(cloud);
					//reg.setIndices (indices);
					reg.setInputNormals(normals);
					reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
					reg.setCurvatureThreshold(curv_thresh);

					std::vector <pcl::PointIndices> clusters;
					reg.extract(clusters);

					PointCloudT::Ptr coloured_cloud = reg.getColoredCloudRGBA();
					visualization::CloudViewer viewer("Cluster viewer");
					viewer.showCloud(coloured_cloud);
					while (!viewer.wasStopped())
					{
						std::this_thread::sleep_for(100us);
					}

					cout << "Number of clusters is equal to " << clusters.size() << endl;
					cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
					/*cout << "These are the indices of the points of the initial" <<
						endl << "cloud that belong to the first cluster:" << endl;
					int counter = 0;
					while (counter < clusters[0].indices.size())
					{
						cout << clusters[0].indices[counter] << ", ";
						counter++;
						if (counter % 10 == 0)
							cout << endl;
					}
					cout << endl;*/

					cout << "Enter name for cloud to be saved under: ";
					string s;
					cin >> s;
					stringstream ss;
					ss << s << ".pcd"; //

					cout << "Saving Cloud." << endl;
					io::savePCDFileBinary<PointT>(ss.str(), *coloured_cloud);
					cout << "Cloud Saved as Binary." << endl;

					break;
				}
			break;
			}
			
			break;
		}
		case 6:
		{
			/*PointCloudT::Ptr cloud(new PointCloudT);

			cout << "Enter name of the cloud to load: ";
			string sf;
			cin >> sf;
			stringstream ssf;
			ssf << sf << ".pcd";

			io::loadPCDFile(ssf.str(), *cloud);

			CSTM_ICP_PIPE pipeline;

			PointCloud<int> keypoint_indices = pipeline.extract_keypoints(cloud);

			cout << "Printing Keypoints:\n" << endl;

			for (int i = 0; i < keypoint_indices.size(); i++)
			{
				cout << "Keypoint " << i << " is " << keypoint_indices[i] << "." << endl;
			}*/
			

			break;
		}
		case 7:
		{
			PointCloudT::Ptr input_cloud(new PointCloudT);
			PointCloudT::Ptr output_cloud(new PointCloudT);
			PointCloudT::Ptr target_cloud(new PointCloudT);
			
			cout << "Enter name of target cloud to load: ";
			string s2;
			cin >> s2;
			stringstream ss2;
			ss2 << s2 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss2.str(), *target_cloud) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			PassThrough<PointT> pass;
			pass.setInputCloud(target_cloud);
			pass.setFilterFieldName("z");
			pass.setFilterLimits(0.0, 10.0);
			pass.filter(*target_cloud);

			cout << "Loaded " << target_cloud->size() << " data points from " << ss2.str() << "." << endl;

			cout << "Enter name of input cloud to load: ";
			string s1;
			cin >> s1;
			stringstream ss1;
			ss1 << s1 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss1.str(), *input_cloud) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			pass.setInputCloud(input_cloud);
			pass.setFilterFieldName("z");
			pass.setFilterLimits(0.0, 10.0);
			pass.filter(*input_cloud);
			
			cout << "Loaded " << input_cloud->size() << " data points from " << ss1.str() << "." << endl;

			// Filtering input scan to roughly 10% of original size to increase speed of registration.
			PointCloudT::Ptr filtered_cloud(new PointCloudT);
			ApproximateVoxelGrid<PointT> approximate_voxel_filter;
			approximate_voxel_filter.setLeafSize(0.01, 0.01, 0.01);
			approximate_voxel_filter.setInputCloud(input_cloud);
			approximate_voxel_filter.filter(*filtered_cloud);
			cout << "Filtered cloud contains " << filtered_cloud->size() << " data points from " << ss1.str() << "." << endl;

			// Initializing Normal Distributions Transform (NDT).
			pcl::NormalDistributionsTransform<PointT, PointT> ndt;

			// Setting scale dependent NDT parameters
			// Setting minimum transformation difference for termination condition.
			ndt.setTransformationEpsilon(0.01);
			// Setting maximum step size for More-Thuente line search.
			ndt.setStepSize(0.1);
			//Setting Resolution of NDT grid structure (VoxelGridCovariance).
			ndt.setResolution(1.0);

			// Setting max number of registration iterations.
			ndt.setMaximumIterations(50);

			// Setting point cloud to be aligned.
			ndt.setInputSource(filtered_cloud);
			// Setting point cloud to be aligned to.
			ndt.setInputTarget(target_cloud);

			// Set initial alignment estimate found using robot odometry.
			//Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
			//Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
			//Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

			// Calculating required rigid transform to align the input cloud to the target cloud.
			ndt.align(*output_cloud);// , init_guess);

			std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
				<< " score: " << ndt.getFitnessScore() << std::endl;

			// Transforming unfiltered, input cloud using found transform.
			transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());

			PointCloudT::Ptr combined_cloud(new PointCloudT);

			*combined_cloud = *output_cloud;
			*combined_cloud += *target_cloud;
			
			cout << "Enter name to save the combined cloud under: ";
			string s3;
			cin >> s3;
			stringstream ss3;
			ss3 << s3 << ".pcd"; //

			if (io::savePCDFileBinary<PointT>(ss3.str(), *target_cloud) == -1)
			{
				PCL_ERROR("Couldnt save file.\n");
			}

			cout << "saved " << combined_cloud->size() << " data points as " << ss2.str() << "." << endl;

			break;
		}
		case 8:
		{
			PointCloudT::Ptr cloud(new PointCloudT), cloud_f(new PointCloudT);
			PointCloudT::Ptr cloud_filtered(new PointCloudT);
			PointIndicesPtr ground(new PointIndices);

			cout << "Enter name of target cloud to load: ";
			string s2;
			cin >> s2;
			stringstream ss2;
			ss2 << s2 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss2.str(), *cloud) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			// Create the filtering object: downsample the dataset using a leaf size of 1cm
			VoxelGrid<PointT> vg;
			vg.setInputCloud(cloud);
			vg.setFilterFieldName("z");
			vg.setFilterLimits(0.0, 2.5);
			vg.setLeafSize(0.01f, 0.01f, 0.01f);
			vg.filter(*cloud_filtered);
			cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << endl; 

			// Create the segmentation object for the planar model and set all the parameters
			SACSegmentation<PointT> seg;
			PointIndices::Ptr inliers(new PointIndices);
			ModelCoefficients::Ptr coefficients(new ModelCoefficients);
			PointCloudT::Ptr cloud_plane(new PointCloudT());
			vector<PointIndices> cluster_indices;
			
			seg.setOptimizeCoefficients(true);
			seg.setModelType(SACMODEL_PLANE);
			seg.setMethodType(SAC_RANSAC);
			seg.setMaxIterations(100);
			seg.setDistanceThreshold(0.02);

			int i = 0, nr_points = (int)cloud_filtered->points.size();
			while (cloud_filtered->points.size() > 0.3 * nr_points)
			{
				cout << "Still in 'while' loop." << endl;
				// Segment the largest planar component from the remaining cloud
				seg.setInputCloud(cloud_filtered);
				seg.segment(*inliers, *coefficients);
				if (inliers->indices.size() == 0)
				{
					cout << "Could not estimate a planar model for the given dataset." << endl;
					break;
				}

				// Extract the planar inliers from the input cloud
				ExtractIndices<PointT> extract;
				extract.setInputCloud(cloud_filtered);
				extract.setIndices(inliers);
				extract.setNegative(false);

				// Get the points associated with the planar surface
				extract.filter(*cloud_plane);
				cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << endl;
				std::stringstream ss;
				ss << "plane_" << i << ".pcd";
				io::savePCDFileBinary(ss.str(), *cloud_plane); //saves the cloud of the current cluster
				i++;

				// Remove the planar inliers, extract the rest
				extract.setNegative(true);
				extract.filter(*cloud_f);
				*cloud_filtered = *cloud_f;

				

				cout << cloud_filtered->points.size() << " > " << 0.3 * nr_points << endl;
			}

			cout << "Exited 'while' loop." << endl;
			cout << "Extracting clusters." << endl;

			// Creating the KdTree object for the search method of the extraction
			search::KdTree<PointT>::Ptr tree(new search::KdTree<PointT>);
			tree->setInputCloud(cloud_filtered);


			EuclideanClusterExtraction<PointT> ec;
			ec.setClusterTolerance(0.02); // 2cm
			ec.setMinClusterSize(50);
			ec.setMaxClusterSize(25000);
			ec.setSearchMethod(tree);
			ec.setInputCloud(cloud_filtered);
			ec.extract(cluster_indices);

			cout << "Clusters extracted." << endl;
			cout << "Saving individual clusters." << endl;

			int j = 0;
			for (vector<PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)//iterates through all the clusters stored in vector
			{
				cout << "entered 'FOR' loop with iteration being " << j << "." << endl;
				PointCloud<PointT>::Ptr cloud_cluster(new PointCloud<PointT>);
				for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) //iterates through all points in a cluster
					cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //adds each point of the cluster at the end of the point cloud vector
				cloud_cluster->width = cloud_cluster->points.size();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true; //no points are invalid

				std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
				std::stringstream ss;
				ss << "cluster_" << j << ".pcd";
				io::savePCDFileBinary(ss.str(), *cloud_cluster); //saves the cloud of the current cluster
				j++;
			}

			break;
		}
		default:
			break;
		break;
		}
	}
	return (0);
}