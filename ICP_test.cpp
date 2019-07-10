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
#include <pcl/common/transforms.h>
#include <pcl/surface/mls.h>

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
		cout << "1:  Perform ICP." << endl;
		cout << "2:  View a Cloud." << endl;
		cout << "3:  Get new Clouds." << endl;
		cout << "4:  Convert existing cloud into a voxel grid(IN PROGRESS)." << endl;
		cout << "5:  Region growing protoype." << endl;
		cout << "6:  Custom ICP pipeline prototype." << endl;
		cout << "7:  Normal distribution transform." << endl;
		cout << "8:  Euclidean Cluster Extraction." << endl;
		cout << "9:  Transformation practice." << endl;
		cout << "10: Get the centroid of a cluster." << endl;
		cout << "11: imple matrix rotation." << endl;
		cout << "12: Insert cube into a cloud." << endl;
		cout << "13: Align 4 Clouds." << endl;
		cout << "14: Cloud combine test condition." << endl;
		cout << "15: Surface smoothing." << endl;
		cout << "0:  Exit Program." << endl;

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
			vg.setFilterLimits(0.0, 2.1);
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
		case 9:
		{
			PointCloudT::Ptr cloud(new PointCloudT);
			PointCloudT::Ptr cloud_transformed(new PointCloudT);

			cout << "Enter name of target cloud to load: ";
			string s2;
			cin >> s2;
			stringstream ss2;
			ss2 << s2 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss2.str(), *cloud) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			PointCloudT::Ptr cloud2(new PointCloudT);

			cout << "Enter name of target cloud to load: ";
			string s1;
			cin >> s1;
			stringstream ss1;
			ss1 << s1 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss1.str(), *cloud2) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			/*float distance = cloud->points[(cloud->width >> 1) * (cloud->height + 1)].z/1000;

			cout << "distance of center pixel :" << distance << " mm." << endl;

			Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();

			transform_1.translation() << 0, 0, -distance;

			transform_1.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()));

			cout << transform_1.matrix() << endl;

			transformPointCloud(*cloud, *cloud, transform_1);*/
			/* 
			Reminder: how transformation matrices work :
			
				   |-------> This column is the translation
			| 1 0 0 x |  \
			| 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
			| 0 0 1 z |  /
			| 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)
			
			*/
			float theta = - M_PI / 2; // The angle of rotation in radians

			Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
			
			// Define a translation of 0.0 meters on the x axis.
			transform_2.translation() << 0, 0, 0;

			// The same rotation matrix as before; theta radians around Z axis
			transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));

			// Print the transformation
			printf("\nMethod using an Affine3f\n");
			cout << transform_2.matrix() << endl;

			transformPointCloud(*cloud2, *cloud_transformed, transform_2);

			char ok='n';

			while (ok != 'y' && ok!='Y')
			{
				cout << "Input translation (x,y,z)." << endl;
				float x, y, z;
				x = y = z = 0;
				cin >> x;
				cin >> y;
				cin >> z;

				Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();

				// Define a translation of 0.0 meters on the x axis.
				transform_3.translation() << x, y, z;

				// The same rotation matrix as before; theta radians around Z axis
				transform_3.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()));

				printf("\nMethod using an Affine3f\n");
				cout << transform_2.matrix() << endl;

				transformPointCloud(*cloud_transformed, *cloud_transformed, transform_3);

				pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

				viewer.addPointCloud(cloud, "Cloud");// note that before it was showCloud
				viewer.addPointCloud(cloud_transformed, "Transformed Cloud");// note that before it was showCloud
				viewer.spin();

				cout << "Press Y to end alignment: ";
				cin >> ok;
			}

			PointCloudT::Ptr saved_cloud(new PointCloudT);

			*saved_cloud = *cloud;
			*saved_cloud += *cloud_transformed;

			cout << "Enter name for cloud to be saved under: ";
			string s3;
			cin >> s3;
			stringstream ss3;
			ss3 << s3 << ".pcd"; //

			if (io::savePCDFileBinary(ss3.str(), *saved_cloud) == -1)
			{
				PCL_ERROR("Couldnt save file.\n");
			}
			
			
			break;
		}
		case 10:
		{
			PointCloudT::Ptr cloud(new PointCloudT);
			//PointCloudT::Ptr cloud_transformed(new PointCloudT);

			cout << "Enter name of target cloud to load: ";
			string s2;
			cin >> s2;
			stringstream ss2;
			ss2 << s2 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss2.str(), *cloud) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}
			Eigen::Vector4f centroid;

			compute3DCentroid(*cloud, centroid);

			cout << "x: " << centroid[0] << endl;
			cout << "y: " << centroid[1] << endl;
			cout << "z: " << centroid[2] << endl;

			cout << "Press enter to continue." << endl;
			ReadLastCharOfLine();

			break;

		}
		case 11:
		{
			PointCloudT::Ptr cloud(new PointCloudT);
			PointCloudT::Ptr cloud_transformed(new PointCloudT);

			cout << "Enter name of target cloud to load: ";
			string s2;
			cin >> s2;
			stringstream ss2;
			ss2 << s2 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss2.str(), *cloud) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			float theta = M_PI / 2; // The angle of rotation in radians

			Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

			// Define a translation of 0.0 meters on the x axis.
			transform_2.translation() << 0, 0, 0;

			// The same rotation matrix as before; theta radians around Z axis
			transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));

			// Print the transformation
			printf("\nMethod using an Affine3f\n");
			cout << transform_2.matrix() << endl;

			transformPointCloud(*cloud, *cloud_transformed, transform_2);

			io::savePCDFileBinary("Cloud_rotated.pcd", *cloud_transformed);

			cout << "Press enter to continue." << endl;
			ReadLastCharOfLine();

			break;
		}
		case 12:
		{
			PointCloudT::Ptr cloud(new PointCloudT);
			PointCloudT::Ptr cloud_transformed(new PointCloudT);

			cout << "Enter name of target cloud to load: ";
			string s2;
			cin >> s2;
			stringstream ss2;
			ss2 << s2 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss2.str(), *cloud) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			PointCloudT::Ptr cloud_cube(new PointCloudT);

			cout << "Specify size of the cube in cm: ";
			short centim = 0;
			cin >> centim;

			cout << "Specify initial coordinates (x,y,z)." << endl;
			short x0, y0, z0;
			cin >> x0;
			cin >> y0;
			cin >> z0;

			short volume = centim * centim * centim;

			// Fill in the cloud data
			cloud_cube->width = volume;
			cloud_cube->height = 1;
			cloud_cube->is_dense = false;
			cloud_cube->points.resize(cloud_cube->width * cloud_cube->height);

			int i = 1;

			short x, y, z;
			x = y = z = 0;
			for (size_t i = 0; i < cloud_cube->points.size(); ++i)
			{
				cloud_cube->points[i].x = x0 + x/100;
				cloud_cube->points[i].y = y0 + y/100;
				cloud_cube->points[i].z = z0 + z/100;
				cloud_cube->points[i].a = 255;
				cloud_cube->points[i].r = 255;
				cloud_cube->points[i].b = 255;
				cloud_cube->points[i].g = 255;

				if (y == centim-1)
				{
					x++;
					z = y = 0;
				}
				else if (z == centim-1)
				{
					y++;
					z = 0;
				}
				if (x == centim-1) { x = 0; }
				z++;
			}
			

			visualization::PCLVisualizer viewer("Cloud Viewer");

			//viewer.addPointCloud(cloud, "Cloud");// note that before it was showCloud
			viewer.addPointCloud(cloud_cube, "Transformed Cloud");// note that before it was showCloud
			viewer.spin();

			break;
		}
		case 13:
		{
			PointCloudT::Ptr cloud1(new PointCloudT);

			cout << "Enter name of first cloud to load: ";
			string s1;
			cin >> s1;
			stringstream ss1;
			ss1 << s1 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss1.str(), *cloud1) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			PointCloudT::Ptr cloud2(new PointCloudT);

			cout << "Enter name of second cloud to load: ";
			string s2;
			cin >> s2;
			stringstream ss2;
			ss2 << s2 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss2.str(), *cloud2) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			PointCloudT::Ptr cloud3(new PointCloudT);

			cout << "Enter name of third cloud to load: ";
			string s3;
			cin >> s3;
			stringstream ss3;
			ss3 << s3 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss3.str(), *cloud3) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			PointCloudT::Ptr cloud4(new PointCloudT);

			cout << "Enter name of second cloud to load: ";
			string s4;
			cin >> s4;
			stringstream ss4;
			ss4 << s4 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss4.str(), *cloud4) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			cout << "Do you want to filter the clouds by distance?\n1: Yes.\n2: No." << endl;
			short filter = 0;
			cin >> filter;
			if (filter == 1)
			{
				// Create the filtering object
				PassThrough<PointT> pass1;
				pass1.setInputCloud(cloud1);
				pass1.setFilterFieldName("z");
				pass1.setFilterLimits(0.0, 2.5);
				//pass.setFilterLimitsNegative (true);
				pass1.filter(*cloud1);

				PassThrough<PointT> pass2;
				pass2.setInputCloud(cloud2);
				pass2.setFilterFieldName("z");
				pass2.setFilterLimits(0.0, 2.5);
				//pass.setFilterLimitsNegative (true);
				pass2.filter(*cloud2);

				PassThrough<PointT> pass3;
				pass3.setInputCloud(cloud3);
				pass3.setFilterFieldName("z");
				pass3.setFilterLimits(0.0, 2.5);
				//pass.setFilterLimitsNegative (true);
				pass3.filter(*cloud3);

				PassThrough<PointT> pass4;
				pass4.setInputCloud(cloud4);
				pass4.setFilterFieldName("z");
				pass4.setFilterLimits(0.0, 2.5);
				//pass.setFilterLimitsNegative (true);
				pass4.filter(*cloud4);
			}
			

			float theta = M_PI / 2; // The angle of rotation in radians

			Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
			Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
			Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();

			// Define a translation of x,y,z meters on the appropriate axis.
			transform_1.translation() << -0.9, 0, 0.9;
			transform_2.translation() << 0, 0, 1.8;
			transform_3.translation() << 0.9, 0, 0.9;

			// The same rotation matrix as before; theta radians around Z axis
			transform_1.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));
			transform_2.rotate(Eigen::AngleAxisf(2 * theta, Eigen::Vector3f::UnitY()));
			transform_3.rotate(Eigen::AngleAxisf(-theta, Eigen::Vector3f::UnitY()));

			// Print the transformation
			printf("\nMethod using an Affine3f\n");
			cout << "First Matrix:" << endl;
			cout << transform_1.matrix() << endl;
			cout << "\nSecond Matrix:" << endl;
			cout << transform_2.matrix() << endl;
			cout << "\nThird Matrix:" << endl;
			cout << transform_3.matrix() << endl;

			PointCloudT::Ptr cloud2_transformed(new PointCloudT);
			PointCloudT::Ptr cloud3_transformed(new PointCloudT);
			PointCloudT::Ptr cloud4_transformed(new PointCloudT);

			transformPointCloud(*cloud2, *cloud2_transformed, transform_1.matrix());
			transformPointCloud(*cloud3, *cloud3_transformed, transform_2.matrix());
			transformPointCloud(*cloud4, *cloud4_transformed, transform_3.matrix());

			PointCloudT::Ptr combined_cloud(new PointCloudT);

			*combined_cloud = *cloud1;
			*combined_cloud += *cloud2_transformed;
			*combined_cloud += *cloud3_transformed;
			*combined_cloud += *cloud4_transformed;

			visualization::PCLVisualizer viewer("Cloud Viewer");
			viewer.addPointCloud(combined_cloud, "Combined Cloud");
			viewer.spin();

			cout << "Enter name for cloud to be saved under: ";
			string ss;
			cin >> ss;
			stringstream sss;
			sss << ss << ".pcd"; //

			if (io::savePCDFileBinary(sss.str(), *combined_cloud) == -1)
			{
				PCL_ERROR("Couldnt save file.\n");
			}

			cout << "Combine with Master Cloud?" << endl;
			cout << "1: Yes." << endl;
			cout << "2: No." << endl;
			short combine = 0;
			cin >> combine;

			if (combine == 1)
			{
				PointCloudT::Ptr master_cloud(new PointCloudT);

				cout << "Enter name of master cloud to load: ";
				string sm;
				cin >> sm;
				stringstream ssm;
				ssm << sm << ".pcd"; //

				if (io::loadPCDFile<PointT>(ssm.str(), *master_cloud) == -1)
				{
					PCL_ERROR("Couldnt read file.\n");
				}

				PointCloudT::Ptr new_master_cloud(new PointCloudT);

				*new_master_cloud = *master_cloud;
				*new_master_cloud += *combined_cloud;

				visualization::PCLVisualizer viewer2("Cloud Viewer");
				viewer2.addPointCloud(new_master_cloud, "New Master Cloud");
				viewer2.spin();

				cout << "Do you want to update the master cloud?" << endl;
				cout << "1: Yes." << endl;
				cout << "2: No." << endl;
				short update = 0;
				cin >> update;

				if (update == 1)
				{
					if (io::savePCDFileBinary(ssm.str(), *new_master_cloud) == -1)
					{
						PCL_ERROR("Couldnt save file.\n");
					}
				}

			}


			break;
		}
		case 14:
		{
			PointCloudT::Ptr cloud1(new PointCloudT);
			PointCloudT::Ptr cloud2(new PointCloudT);

			// Fill in the cloud data
			cloud1->width = 8;
			cloud1->height = 1;
			cloud1->points.resize(cloud1->width * cloud1->height);

			for (size_t i = 0; i < cloud1->points.size(); ++i)
			{
				//srand(6254);
				cloud1->points[i].x = rand() % 3;
				cloud1->points[i].y = rand() % 3;
				cloud1->points[i].z = rand() % 3;
				cloud1->points[i].r = rand() % 256;
				cloud1->points[i].g = rand() % 256;
				cloud1->points[i].b = rand() % 256;
				cloud1->points[i].a = 255;
			}

			// Fill in the cloud data
			short width = 10;
			cloud2->width = width;
			cloud2->height = 1;
			cloud2->points.resize(cloud2->width * cloud2->height);

			for (size_t i = 0; i < cloud2->points.size()-1; ++i)
			{
				//srand(2465);
				cloud2->points[i].x = rand() % 3;
				cloud2->points[i].y = rand() % 3;
				cloud2->points[i].z = rand() % 3;
				cloud2->points[i].r = rand() % 256;
				cloud2->points[i].g = rand() % 256;
				cloud2->points[i].b = rand() % 256;
				cloud2->points[i].a = 255;
			}
			cloud2->points[width-1].x = 2;
			cloud2->points[width-1].y = 2;
			cloud2->points[width-1].z = 1;
			cloud2->points[width-1].r = 255;
			cloud2->points[width-1].g = 0;
			cloud2->points[width-1].b = 0;
			cloud2->points[width-1].a = 255;

			cout << "Cloud1: " << endl;
			for (size_t i = 0; i < cloud1->points.size(); ++i)
				cout << "    Point " << i << ", pos (" << cloud1->points[i].x << ","
				<< cloud1->points[i].y << ","
				<< cloud1->points[i].z << ") col ("
				<< (int)cloud1->points[i].r << ","
				<< (int)cloud1->points[i].g << ","
				<< (int)cloud1->points[i].b << ","
				<< (int)cloud1->points[i].a << ")." << endl;

			cout << "\n\nCloud2: " << endl;
			for (size_t i = 0; i < cloud2->points.size(); ++i)
				cout << "    Point " << i << ", pos (" << cloud2->points[i].x << ","
				<< cloud2->points[i].y << ","
				<< cloud2->points[i].z << ") col ("
				<< (int)cloud2->points[i].r << ","
				<< (int)cloud2->points[i].g << ","
				<< (int)cloud2->points[i].b << ","
				<< (int)cloud2->points[i].a << ")." << endl;

			PointCloudT::Ptr combined_cloud(new PointCloudT);

			*combined_cloud = *cloud1;
			*combined_cloud += *cloud2;

			cout << "\n\nThe two combined: " << endl;
			for (size_t i = 0; i < combined_cloud->points.size(); ++i)
				cout << "    Point " << i << ", pos (" << combined_cloud->points[i].x << ","
				<< combined_cloud->points[i].y << ","
				<< combined_cloud->points[i].z << ") col ("
				<< (int)combined_cloud->points[i].r << ","
				<< (int)combined_cloud->points[i].g << ","
				<< (int)combined_cloud->points[i].b << ","
				<< (int)combined_cloud->points[i].a << ")." << endl;


			visualization::PCLVisualizer viewer("Cloud Viewer");
			viewer.addPointCloud(combined_cloud, "Combined Cloud");
			viewer.spin();

		}
		case 15:
		{
			// Load input file into a PointCloud<T> with an appropriate type
			PointCloudT::Ptr cloud(new PointCloudT);

			cout << "Enter name of first cloud to load: ";
			string s1;
			cin >> s1;
			stringstream ss1;
			ss1 << s1 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss1.str(), *cloud) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			// Create a KD-Tree
			search::KdTree<PointT>::Ptr tree(new search::KdTree<PointT>);

			// Output has the PointNormal type in order to store the normals calculated by MLS
			PointCloud<PointNormal> mls_points;

			// Init object (second point type is for the normals, even if unused)
			MovingLeastSquares<PointT, PointNormal> mls;

			mls.setComputeNormals(true);

			// Set parameters
			mls.setInputCloud(cloud);
			mls.setPolynomialOrder(2);
			mls.setSearchMethod(tree);
			mls.setSearchRadius(0.05);

			// Reconstruct
			mls.process(mls_points);

			cout << "Enter name to save cloud under: ";
			string ss;
			cin >> ss;
			stringstream sss;
			sss << ss << ".pcd"; //

			if (io::savePCDFileBinary(sss.str(), mls_points) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
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