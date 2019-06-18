#ifndef  Vis_Attempt
#define Vis_Attempt

//Visualiser functions used to view the point clouds


//standard headers
#include <iostream>
#include <thread>

//pcl headers
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>



//namespace declerations
using namespace std;
using namespace chrono_literals;
using namespace pcl;

//CODE------------
visualization::PCLVisualizer::Ptr simpleVis(PointCloud<PointXYZ>::ConstPtr cloud)
{
	//Open 3D viewer and add point Cloud
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<PointXYZ>(cloud, "Sample Cloud");
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Sample Cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return viewer;
}

visualization::PCLVisualizer::Ptr rgbVis(PointCloud<PointXYZRGB>::ConstPtr cloud)
{
	//open 3D viewer and add point cloud
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<PointXYZRGB>(cloud, rgb, "test");
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "test");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return viewer;
}

visualization::PCLVisualizer::Ptr rgbaVis(PointCloud<PointXYZRGBA>::ConstPtr cloud)
{
	//open 3D viewer and add point cloud
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	visualization::PointCloudColorHandlerRGBAField<PointXYZRGBA> rgba(cloud);
	viewer->addPointCloud<PointXYZRGBA>(cloud, rgba, "test");
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "test");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return viewer;
}

visualization::PCLVisualizer::Ptr customColourVis(PointCloud<PointXYZ>::ConstPtr cloud)
{
	//open 3D viewer and add point cloud
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	visualization::PointCloudColorHandlerCustom<PointXYZ>single_colour(cloud, 0, 255, 0);
	viewer->addPointCloud<PointXYZ>(cloud, single_colour, "SampleCloud");
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Sample Cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return viewer;
}

visualization::PCLVisualizer::Ptr normalsVis(PointCloud<PointXYZRGB>::ConstPtr cloud, PointCloud<Normal>::ConstPtr normals)
{
	//open 3D viewer and add point cloud
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<PointXYZRGB>(cloud, rgb, "SampleCloud");
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Sample Cloud");
	viewer->addPointCloudNormals<PointXYZRGB, Normal>(cloud, normals, 10, 0.05, "Normals");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return viewer;
}

visualization::PCLVisualizer::Ptr shapesVis(PointCloud<PointXYZRGB>::ConstPtr cloud)
{
	//open 3D viewer and add point cloud
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<PointXYZRGB>(cloud, rgb, "SampleCloud");
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Sample Cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	//Add shapes at cloud points
	viewer->addLine<PointXYZRGB>(cloud->points[0], cloud->points[cloud->size() - 1], "Line");
	viewer->addSphere(cloud->points[0], 0.2, 0.5, 0.5, 0.0, "Sphere");

	//Add shapes at other locations
	ModelCoefficients coeffs;
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	viewer->addPlane(coeffs, "plane");
	coeffs.values.clear();
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(5.0);
	viewer->addCone(coeffs, "cone");

	return viewer;
}

visualization::PCLVisualizer::Ptr viewportsVis(PointCloud<PointXYZRGB>::ConstPtr cloud, PointCloud<Normal>::ConstPtr normals1, PointCloud<Normal>::ConstPtr normals2)
{
	//Open 3D viewer and add point cloud and normals
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D viewer"));
	viewer->initCameraParameters();

	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
	visualization::PointCloudColorHandlerRGBField<PointXYZRGB>rgb(cloud);
	viewer->addPointCloud<PointXYZRGB>(cloud, rgb, "Sample cloud1", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	visualization::PointCloudColorHandlerCustom<PointXYZRGB>single_colour(cloud, 0, 255, 0);
	viewer->addPointCloud<PointXYZRGB>(cloud, single_colour, "Sample cloud2", v2);

	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Sample cloud1");
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Sample cloud2");
	viewer->addCoordinateSystem(1.0);

	viewer->addPointCloudNormals<PointXYZRGB, Normal>(cloud, normals1, 10, 0.05, "Normals1", v1);
	viewer->addPointCloudNormals<PointXYZRGB, Normal>(cloud, normals2, 10, 0.05, "Normals2", v2);

	return viewer;
}

unsigned int text_id = 0;
/*
void keyboardEventOccurred(const visualization::KeyboardEvent &event, void* viewer_void)
{
	visualization::PCLVisualizer *viewer = static_cast<visualization::PCLVisualizer *>(viewer_void);
	if (event.getKeySym() == "r" && event.keyDown())
	{
		cout << "r was pressed => removing all text" << endl;

		char str[512];
		for (unsigned int i = 0; i < text_id; ++i)
		{
			sprintf(str, "text#%03d", i);
			viewer->removeShape(str);
		}
		text_id = 0;
	}
}
*/
void mouseEventOccurred(const visualization::MouseEvent &event, void* viewer_void)
{
	visualization::PCLVisualizer *viewer = static_cast<visualization::PCLVisualizer *> (viewer_void);
	if (event.getButton() == visualization::MouseEvent::LeftButton && event.getType() == visualization::MouseEvent::MouseButtonRelease)
	{
		cout << "Left mouse button released at position " << event.getX() << "," << event.getY() << endl;

		char str[512];

		sprintf(str, "text#%03d", text_id++);
		viewer->addText("Clicked here.", event.getX(), event.getY(), str);
	}
}

visualization::PCLVisualizer::Ptr interactionCustomizationVis()
{
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);

	//sviewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
	viewer->registerMouseCallback(mouseEventOccurred, (void*)viewer.get());

	return viewer;
}
#endif // ! Vis_Attempt

