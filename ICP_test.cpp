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
		cout << "11: Simple matrix rotation." << endl;
		cout << "12: Insert cube into a cloud." << endl;
		cout << "13: Align 8 Clouds." << endl;
		cout << "14: Creating a square outline in a cloud." << endl;
		cout << "15: Surface smoothing." << endl;
		cout << "16: Voxelise a cloud." << endl;
		cout << "17: Add boudning boxes" << endl;
		cout << "18: Colour Filtering with Sphere." << endl;
		cout << "19: Colour Filtering Circles." << endl;
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
			ec.setClusterTolerance(0.10); // 2cm
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

			cout << "Enter name of first cloud to load (0*pi): ";
			string s1;
			cin >> s1;
			stringstream ss1;
			ss1 << s1 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss1.str(), *cloud1) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			PointCloudT::Ptr cloud2(new PointCloudT);

			cout << "Enter name of second cloud to load (pi/4): ";
			string s2;
			cin >> s2;
			stringstream ss2;
			ss2 << s2 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss2.str(), *cloud2) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			PointCloudT::Ptr cloud3(new PointCloudT);

			cout << "Enter name of third cloud to load (pi/2): ";
			string s3;
			cin >> s3;
			stringstream ss3;
			ss3 << s3 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss3.str(), *cloud3) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			PointCloudT::Ptr cloud4(new PointCloudT);

			cout << "Enter name of fourth cloud to load (3*pi/4): ";
			string s4;
			cin >> s4;
			stringstream ss4;
			ss4 << s4 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss4.str(), *cloud4) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			PointCloudT::Ptr cloud5(new PointCloudT);

			cout << "Enter name of fifth cloud to load (pi): ";
			string s5;
			cin >> s5;
			stringstream ss5;
			ss5 << s5 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss5.str(), *cloud5) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			PointCloudT::Ptr cloud6(new PointCloudT);

			cout << "Enter name of sixth cloud to load (5*pi/4): ";
			string s6;
			cin >> s6;
			stringstream ss6;
			ss6 << s6 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss6.str(), *cloud6) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			PointCloudT::Ptr cloud7(new PointCloudT);

			cout << "Enter name of seventh cloud to load (6*pi/2): ";
			string s7;
			cin >> s7;
			stringstream ss7;
			ss7 << s7 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss7.str(), *cloud7) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			PointCloudT::Ptr cloud8(new PointCloudT);

			cout << "Enter name of eighth cloud to load (7*pi/4): ";
			string s8;
			cin >> s8;
			stringstream ss8;
			ss8 << s8 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss8.str(), *cloud8) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			cout << "Do you want to filter the clouds by distance?\n1: Yes.\n2: No." << endl;
			short filter = 0;
			cin >> filter;
			if (filter == 1)
			{
				float filt_min = 0.0, filt_max = 1.8;
				// Create the filtering object
				PassThrough<PointT> pass1;
				pass1.setInputCloud(cloud1);
				pass1.setFilterFieldName("z");
				pass1.setFilterLimits(filt_min, filt_max);
				//pass.setFilterLimitsNegative (true);
				pass1.filter(*cloud1);

				PassThrough<PointT> pass2;
				pass2.setInputCloud(cloud2);
				pass2.setFilterFieldName("z");
				pass2.setFilterLimits(filt_min, filt_max);
				//pass.setFilterLimitsNegative (true);
				pass2.filter(*cloud2);

				PassThrough<PointT> pass3;
				pass3.setInputCloud(cloud3);
				pass3.setFilterFieldName("z");
				pass3.setFilterLimits(filt_min, filt_max);
				//pass.setFilterLimitsNegative (true);
				pass3.filter(*cloud3);

				PassThrough<PointT> pass4;
				pass4.setInputCloud(cloud4);
				pass4.setFilterFieldName("z");
				pass4.setFilterLimits(filt_min, filt_max);
				//pass.setFilterLimitsNegative (true);
				pass4.filter(*cloud4);

				PassThrough<PointT> pass5;
				pass5.setInputCloud(cloud5);
				pass5.setFilterFieldName("z");
				pass5.setFilterLimits(filt_min, filt_max);
				//pass.setFilterLimitsNegative (true);
				pass5.filter(*cloud5);

				PassThrough<PointT> pass6;
				pass6.setInputCloud(cloud6);
				pass6.setFilterFieldName("z");
				pass6.setFilterLimits(filt_min, filt_max);
				//pass.setFilterLimitsNegative (true);
				pass6.filter(*cloud6);

				PassThrough<PointT> pass7;
				pass7.setInputCloud(cloud7);
				pass7.setFilterFieldName("z");
				pass7.setFilterLimits(filt_min, filt_max);
				//pass.setFilterLimitsNegative (true);
				pass7.filter(*cloud7);

				PassThrough<PointT> pass8;
				pass8.setInputCloud(cloud8);
				pass8.setFilterFieldName("z");
				pass8.setFilterLimits(filt_min, filt_max);
				//pass.setFilterLimitsNegative (true);
				pass8.filter(*cloud8);
			}
			

			float theta = M_PI / 4; // The angle of rotation in radians (45 degrees)

			Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity(); //45 degrees
			Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity(); //90 degrees
			Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity(); //135 degrees
			Eigen::Affine3f transform_4 = Eigen::Affine3f::Identity(); //180 degrees
			Eigen::Affine3f transform_5 = Eigen::Affine3f::Identity(); //225 degrees
			Eigen::Affine3f transform_6 = Eigen::Affine3f::Identity(); //270 degrees
			Eigen::Affine3f transform_7 = Eigen::Affine3f::Identity(); //315 degrees

			// Define a translation of x,y,z meters on the appropriate axis
			transform_1.translation() << -0.636, 0, 0.9 - 0.636;	//45 degrees
			transform_2.translation() << -0.9, 0, 0.9;					//90 degrees
			transform_3.translation() << -0.636, 0, 0.9 + 0.636;	//135 degrees
			transform_4.translation() << 0, 0, 1.8;						//180 degrees
			transform_5.translation() << 0.636, 0, 0.9 + 0.636;	//225 degrees
			transform_6.translation() << 0.9, 0, 0.9;					//270 degrees
			transform_7.translation() << 0.636, 0, 0.9 - 0.636;	//315 degrees

			// The same rotation matrix as before; theta radians around Z axis
			transform_1.rotate(Eigen::AngleAxisf(1 * theta, Eigen::Vector3f::UnitY()));	//45 degrees
			transform_2.rotate(Eigen::AngleAxisf(2 * theta, Eigen::Vector3f::UnitY())); //90 degrees
			transform_3.rotate(Eigen::AngleAxisf(3 * theta, Eigen::Vector3f::UnitY())); //135 degrees
			transform_4.rotate(Eigen::AngleAxisf(4 * theta, Eigen::Vector3f::UnitY())); //180 degrees
			transform_5.rotate(Eigen::AngleAxisf(-3 * theta, Eigen::Vector3f::UnitY()));//225 degrees
			transform_6.rotate(Eigen::AngleAxisf(-2 * theta, Eigen::Vector3f::UnitY()));//270 degrees
			transform_7.rotate(Eigen::AngleAxisf(-1 * theta, Eigen::Vector3f::UnitY()));//315 degrees

			// Print the transformation
			printf("\nMethod using an Affine3f\n");
			cout << "First Matrix:" << endl;
			cout << transform_1.matrix() << endl;
			cout << "\nSecond Matrix:" << endl;
			cout << transform_2.matrix() << endl;
			cout << "\nThird Matrix:" << endl;
			cout << transform_3.matrix() << endl;
			cout << "\nFourth Matrix:" << endl;
			cout << transform_4.matrix() << endl;
			cout << "\nFifth Matrix:" << endl;
			cout << transform_5.matrix() << endl;
			cout << "\nSixth Matrix:" << endl;
			cout << transform_6.matrix() << endl;
			cout << "\nSeventh Matrix:" << endl;
			cout << transform_7.matrix() << endl;

			PointCloudT::Ptr cloud2_transformed(new PointCloudT);
			PointCloudT::Ptr cloud3_transformed(new PointCloudT);
			PointCloudT::Ptr cloud4_transformed(new PointCloudT);
			PointCloudT::Ptr cloud5_transformed(new PointCloudT);
			PointCloudT::Ptr cloud6_transformed(new PointCloudT);
			PointCloudT::Ptr cloud7_transformed(new PointCloudT);
			PointCloudT::Ptr cloud8_transformed(new PointCloudT);

			cout << "Use ICP to increase accuracy (increases compilation time)?\n1: Yes.\n2: No." << endl;
			short use_icp = 0;
			cin >> use_icp;
			if (use_icp == 1)
			{
				short max_iterations = 40;
				int corres_dist = 10; //cm
				double trans_epsilon = 1e-9;
				double fit_epsilon = 0.5;
				double reject_thresh = 0.05;

				//cloud2 alignment
				IterativeClosestPoint<PointT, PointT> icp1;
				// set the input and target
				icp1.setInputSource(cloud2);
				icp1.setInputTarget(cloud1);
				// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
				icp1.setMaxCorrespondenceDistance(corres_dist);
				// Set the maximum number of iterations (criterion 1)
				icp1.setMaximumIterations(max_iterations);
				// Set the transformation epsilon (criterion 2)
				icp1.setTransformationEpsilon(trans_epsilon);
				// Set the euclidean distance difference epsilon (criterion 3)
				icp1.setEuclideanFitnessEpsilon(fit_epsilon);

				icp1.setRANSACOutlierRejectionThreshold(reject_thresh);
				icp1.setRANSACIterations(max_iterations);

				icp1.align(*cloud2_transformed,transform_1.matrix());

				//cloud3 alignment
				IterativeClosestPoint<PointT, PointT> icp2;
				// set the input and target
				icp2.setInputSource(cloud3);
				icp2.setInputTarget(cloud1);
				// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
				icp2.setMaxCorrespondenceDistance(corres_dist);
				// Set the maximum number of iterations (criterion 1)
				icp2.setMaximumIterations(max_iterations);
				// Set the transformation epsilon (criterion 2)
				icp2.setTransformationEpsilon(trans_epsilon);
				// Set the euclidean distance difference epsilon (criterion 3)
				icp2.setEuclideanFitnessEpsilon(fit_epsilon);

				icp2.setRANSACOutlierRejectionThreshold(reject_thresh);
				icp2.setRANSACIterations(max_iterations);

				icp2.align(*cloud3_transformed, transform_2.matrix());

				//cloud4 alignment
				IterativeClosestPoint<PointT, PointT> icp3;
				// set the input and target
				icp3.setInputSource(cloud4);
				icp3.setInputTarget(cloud1);
				// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
				icp3.setMaxCorrespondenceDistance(corres_dist);
				// Set the maximum number of iterations (criterion 1)
				icp3.setMaximumIterations(max_iterations);
				// Set the transformation epsilon (criterion 2)
				icp3.setTransformationEpsilon(trans_epsilon);
				// Set the euclidean distance difference epsilon (criterion 3)
				icp3.setEuclideanFitnessEpsilon(fit_epsilon);

				icp3.setRANSACOutlierRejectionThreshold(reject_thresh);
				icp3.setRANSACIterations(max_iterations);

				icp3.align(*cloud4_transformed, transform_3.matrix());

				//cloud5 alignment
				IterativeClosestPoint<PointT, PointT> icp4;
				// set the input and target
				icp4.setInputSource(cloud5);
				icp4.setInputTarget(cloud1);
				// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
				icp4.setMaxCorrespondenceDistance(corres_dist);
				// Set the maximum number of iterations (criterion 1)
				icp4.setMaximumIterations(max_iterations);
				// Set the transformation epsilon (criterion 2)
				icp4.setTransformationEpsilon(trans_epsilon);
				// Set the euclidean distance difference epsilon (criterion 3)
				icp4.setEuclideanFitnessEpsilon(fit_epsilon);

				icp4.setRANSACOutlierRejectionThreshold(reject_thresh);
				icp4.setRANSACIterations(max_iterations);

				icp4.align(*cloud5_transformed, transform_4.matrix());

				//cloud4 alignment
				IterativeClosestPoint<PointT, PointT> icp5;
				// set the input and target
				icp5.setInputSource(cloud6);
				icp5.setInputTarget(cloud1);
				// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
				icp5.setMaxCorrespondenceDistance(corres_dist);
				// Set the maximum number of iterations (criterion 1)
				icp5.setMaximumIterations(max_iterations);
				// Set the transformation epsilon (criterion 2)
				icp5.setTransformationEpsilon(trans_epsilon);
				// Set the euclidean distance difference epsilon (criterion 3)
				icp5.setEuclideanFitnessEpsilon(fit_epsilon);

				icp5.setRANSACOutlierRejectionThreshold(reject_thresh);
				icp5.setRANSACIterations(max_iterations);

				icp5.align(*cloud6_transformed, transform_5.matrix());

				//cloud4 alignment
				IterativeClosestPoint<PointT, PointT> icp6;
				// set the input and target
				icp6.setInputSource(cloud7);
				icp6.setInputTarget(cloud1);
				// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
				icp6.setMaxCorrespondenceDistance(corres_dist);
				// Set the maximum number of iterations (criterion 1)
				icp6.setMaximumIterations(max_iterations);
				// Set the transformation epsilon (criterion 2)
				icp6.setTransformationEpsilon(trans_epsilon);
				// Set the euclidean distance difference epsilon (criterion 3)
				icp6.setEuclideanFitnessEpsilon(fit_epsilon);

				icp6.setRANSACOutlierRejectionThreshold(reject_thresh);
				icp6.setRANSACIterations(max_iterations);

				icp6.align(*cloud7_transformed, transform_6.matrix());

				//cloud4 alignment
				IterativeClosestPoint<PointT, PointT> icp7;
				// set the input and target
				icp7.setInputSource(cloud8);
				icp7.setInputTarget(cloud1);
				// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
				icp7.setMaxCorrespondenceDistance(corres_dist);
				// Set the maximum number of iterations (criterion 1)
				icp7.setMaximumIterations(max_iterations);
				// Set the transformation epsilon (criterion 2)
				icp7.setTransformationEpsilon(trans_epsilon);
				// Set the euclidean distance difference epsilon (criterion 3)
				icp7.setEuclideanFitnessEpsilon(fit_epsilon);

				icp7.setRANSACOutlierRejectionThreshold(reject_thresh);
				icp7.setRANSACIterations(max_iterations);

				icp7.align(*cloud8_transformed, transform_7.matrix());
			}
			else 
			{
				transformPointCloud(*cloud2, *cloud2_transformed, transform_1.matrix());
				transformPointCloud(*cloud3, *cloud3_transformed, transform_2.matrix());
				transformPointCloud(*cloud4, *cloud4_transformed, transform_3.matrix());
				transformPointCloud(*cloud5, *cloud5_transformed, transform_4.matrix());
				transformPointCloud(*cloud6, *cloud6_transformed, transform_5.matrix());
				transformPointCloud(*cloud7, *cloud7_transformed, transform_6.matrix());
				transformPointCloud(*cloud8, *cloud8_transformed, transform_7.matrix());
			}

			PointCloudT::Ptr combined_cloud(new PointCloudT);

			*combined_cloud = *cloud1;
			*combined_cloud += *cloud2_transformed;
			*combined_cloud += *cloud3_transformed;
			*combined_cloud += *cloud4_transformed;
			*combined_cloud += *cloud5_transformed;
			*combined_cloud += *cloud6_transformed;
			*combined_cloud += *cloud7_transformed;
			*combined_cloud += *cloud8_transformed;

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
			PointCloudT::Ptr cloud(new PointCloudT);

			cloud->width = 40000;
			cloud->height = 1;
			cloud->resize(cloud->height * cloud->width);
			float xmax = 210.0;
			float zmax = 210.0;
			int i = 0;

			for (float x = 0; x <= xmax; x+=1.0)
			{
				for (float z = 0; z <= zmax; z+=1)
				{
					cout << x << "," << z << endl;
					if ((x <= xmax && x >= (xmax - 10.0)) || (z <= zmax && z >= (zmax - 10.0)))
					{
						cout << "entered if condition." << endl;
						++i;
						cloud->points[i].x = x / 100.0;
						cloud->points[i].y = 0.0;
						cloud->points[i].z = z / 100.0;
						cloud->points[i].r = 127;
						cloud->points[i].g = 0;
						cloud->points[i].b = 255;
						cloud->points[i].a = 255;
						cout << i << endl;
						cout << cloud->points[i].x << endl;
						++i;
						cloud->points[i].x = -x / 100;
						cloud->points[i].y = 0;
						cloud->points[i].z = z / 100;
						cloud->points[i].r = 127;
						cloud->points[i].g = 0;
						cloud->points[i].b = 255;
						cloud->points[i].a = 255;
						cout << i << endl;
						cout << cloud->points[i].x << endl;
						++i;
						cloud->points[i].x = x / 100;
						cloud->points[i].y = 0;
						cloud->points[i].z = -z / 100;
						cloud->points[i].r = 127;
						cloud->points[i].g = 0;
						cloud->points[i].b = 255;
						cloud->points[i].a = 255;
						cout << cloud->points[i] << endl;
						++i;
						cloud->points[i].x = -x / 100;
						cloud->points[i].y = 0;
						cloud->points[i].z = -z / 100;
						cloud->points[i].r = 127;
						cloud->points[i].g = 0;
						cloud->points[i].b = 255;
						cloud->points[i].a = 255;
						cout << i << endl;
						cout << cloud->points[i].x << endl;
					}
				}
			}


			visualization::PCLVisualizer viewer("Cloud Viewer");
			viewer.addPointCloud(cloud, "Cloud");
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
			//PointCloudT::Ptr mls_points(new PointCloudT);


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
		case 16:
		{
			PointCloudT::Ptr cloud(new PointCloudT);

			cout << "Enter name of first cloud to load (0*pi): ";
			string s1;
			cin >> s1;
			stringstream ss1;
			ss1 << s1 << ".pcd"; //

			if (io::loadPCDFile<PointT>(ss1.str(), *cloud) == -1)
			{
				PCL_ERROR("Couldnt read file.\n");
			}

			PointCloudT::Ptr cloud_voxel(new PointCloudT);

			cout << "Point Cloud before filtering: " << cloud->width * cloud->height << " data points (" << getFieldsList(*cloud) << ")." << endl;

			VoxelGrid<PointT> vxl;
			vxl.setInputCloud(cloud);
			//vxl.setFilterFieldName("z");
			//vxl.setFilterLimits(0.0, 2.1);
			vxl.setLeafSize(0.01f, 0.01f, 0.01f);
			vxl.filter(*cloud_voxel);

			cout << "PointCloud after filtering: " << cloud_voxel->width * cloud_voxel->height
				<< " data points (" << getFieldsList(*cloud_voxel) << ")." << endl;

			visualization::PCLVisualizer viewer("Cloud Viewer");
			viewer.addPointCloud(cloud, "Cloud");
			viewer.spin();

			cout << "Enter name for downsampled cloud." << endl;
			string sn;
			cin >> sn;
			stringstream ss;
			ss << sn << ".pcd"; //
			io::savePCDFileBinary(ss.str(), *cloud_voxel);


		}
		case 17: 
		{
			PointCloudT::Ptr cloud(new PointCloudT), cloud_f(new PointCloudT);
			PointCloudT::Ptr cloud_filtered(new PointCloudT);
			PointIndicesPtr ground(new PointIndices);
			vector<PointIndices> cluster_indices;

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
			cout << "Extracting clusters." << endl;

			// Creating the KdTree object for the search method of the extraction
			search::KdTree<PointT>::Ptr tree(new search::KdTree<PointT>);
			tree->setInputCloud(cloud_filtered);


			EuclideanClusterExtraction<PointT> ec;
			ec.setClusterTolerance(0.03); // 2cm
			ec.setMinClusterSize(50);
			ec.setMaxClusterSize(25000);
			ec.setSearchMethod(tree);
			ec.setInputCloud(cloud_filtered);
			ec.extract(cluster_indices);

			cout << "Clusters extracted." << endl;
			cout << "Saving individual clusters." << endl;

			PointT minmax2f[100];

			int j = 0;
			int vector_it = 0;
			for (vector<PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)//iterates through all the clusters stored in vector
			{
				cout << "entered 'FOR' loop with iteration being " << j << "." << endl;
				PointCloud<PointT>::Ptr cloud_cluster(new PointCloud<PointT>);
				//PointCloudT::Ptr minmaxpoints(new PointCloudT);
				PointT min, max;
				for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) //iterates through all points in a cluster
					cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //adds each point of the cluster at the end of the point cloud vector
				cloud_cluster->width = cloud_cluster->points.size();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true; //no points are invalid

				getMinMax3D(*cloud_cluster, min, max);
				/*minmaxpoints->points.push_back(min);
				minmaxpoints->points.push_back(max);
				minmaxpoints->width = minmaxpoints->points.size();
				minmaxpoints->height = 1;
				minmaxpoints->is_dense = true;*/ //no points are invalid
				minmax2f[vector_it] = min;
				minmax2f[vector_it + 1] = max;
				

				cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << endl;
				stringstream ss;
				ss << "cluster_" << j << ".pcd";
				io::savePCDFileBinary(ss.str(), *cloud_cluster); //saves the cloud of the current cluster
				j++;
				vector_it += 2;
			}

			PointCloudT::Ptr new_cloud(new PointCloudT);
			
			int point_counter = 1;
			cout << "Adding boxes around detected clusters." << endl;
			for (int it = 0; it < 2 * j; it += 2)
			{
				PointT minpoint = minmax2f[it];
				PointT maxpoint = minmax2f[it + 1];
				
				//cout << "Cluster " << it/2 << " min point is " << minpoint << ", and max point is " << maxpoint << endl;
				for (double x = minpoint.x; x <= maxpoint.x; x += 0.02)
				{
					for (double z = minpoint.z; z <= maxpoint.z; z += 0.02)
					{
						point_counter++;
					}
				}
				
			}

			cout << "Total number of points needed is " << point_counter << endl;
			new_cloud->width = point_counter;
			new_cloud->height = 1;
			new_cloud->is_dense = true; //no points are invalid
			new_cloud->resize(new_cloud->width * new_cloud->height);

			int curr_point = 1;
			for (int it = 0; it < 2 * j; it += 2)
			{
				PointT minpoint = minmax2f[it];
				PointT maxpoint = minmax2f[it + 1];

				//cout << "Cluster " << it / 2 << " min point is " << minpoint << ", and max point is " << maxpoint << endl;
				for (double x = minpoint.x; x <= maxpoint.x; x += 0.02)
				{
					for (double z = minpoint.z; z <= maxpoint.z; z += 0.02)
					{
						//cout << x << "," << z << "," << curr_point;

						new_cloud->points[curr_point].x = x;
						//cout << "x set to " << x << endl;
						new_cloud->points[curr_point].y = minpoint.y;
						//cout << "y set to " << minpoint.y << endl;
						new_cloud->points[curr_point].z = z;
						//cout << "z set to " << z << endl;
						new_cloud->points[curr_point].r = 255;
						//cout << "r set to 255" << endl;
						new_cloud->points[curr_point].g = 30;
						//cout << "g set to 30" << endl;
						new_cloud->points[curr_point].b = 30;
						//cout << "b set to 30" << endl;
						new_cloud->points[curr_point].a = 255;
						//cout << "a set to 255" << endl;

						++curr_point;
						//cout << "point counter incremented to " << curr_point << endl;
					}
				}
			}
			PointCloudT::Ptr final_cloud(new PointCloudT);

			*final_cloud = *cloud;
			*final_cloud += *new_cloud;

			visualization::PCLVisualizer viewer("Cloud Viewer");
			viewer.addPointCloud(final_cloud, "Cloud");
			viewer.spin();

			break;
		}
		case 18:
		{
			Colour_Filter_Sphere col_filt;
			col_filt.run();
			
			break;
		}
		case 19:
		{
			Colour_Filter_Circles col_filt;
			col_filt.run();

			break;
		}
		default:
			break;
		break;
		}
	}
	return (0);
}