#pragma once
//Header file for custom point types

#ifndef CST_PNT_TYP
#define CST_PNT_TYP
#define PCL_NO_PRECOMPILE

//Headers
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <Eigen/Core>
#include <ostream>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/file_io.h>
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

namespace Custom_Point
{
	struct EIGEN_ALIGN16 _PointXYZRGBAL
	{
		PCL_ADD_POINT4D;
		PCL_ADD_RGB;
		uint32_t label;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	//PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZRGBAL& p);
	struct EIGEN_ALIGN16 PointXYZRGBAL : public _PointXYZRGBAL
	{
		inline PointXYZRGBAL(const _PointXYZRGBAL &p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			rgba = p.rgba;
			label = p.label;
		}

		inline PointXYZRGBAL()
		{
			x = y = z = 0.0f;
			data[3] = 1.0f;
			r = g = b = 0;
			a = 255;
			label = 0;
		}

		//friend std::ostream& operator << (std::ostream& os, const PointXYZRGBAL& p);
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	

} // namespace Custom_Point

POINT_CLOUD_REGISTER_POINT_STRUCT(Custom_Point::_PointXYZRGBAL,
(float, x, x)
(float, y, y)
(float, z, z)
(uint32_t, rgba, rgba)
(uint32_t, label, label)

)
POINT_CLOUD_REGISTER_POINT_WRAPPER(Custom_Point::PointXYZRGBAL, Custom_Point::_PointXYZRGBAL)

PCL_INSTANTIATE_PointCloud(Custom_Point::PointXYZRGBAL);

namespace Custom_Point {
	//typedef pcl::PointCloud<PointXYZRGBAL> PointCloud;
	//typedef boost::shared_ptr<PointCloud> PointCloudPtr;
	//typedef boost::shared_ptr<PointCloud const> PointCloudConstPtr;
}








#endif // !CST_PNT_TYP


