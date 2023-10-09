#ifndef VOLUME_H
#define VOLUME_H

#include <string>
#include <vector>
#include <array>
#include <algorithm>
#include <locale.h>
#include <time.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/geometry/mesh_conversion.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/common/shapes.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <pcl/common/colors.h>
#include <pcl/common/transforms.h>

#include <pcl/point_cloud.h>	
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/octree/octree_search.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAxesActor.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include "../libs/CSF/src/CSF.h"

namespace VolumeEfficent
{


    // 定义点云类型模板
    typedef pcl::PointXYZRGB PointT;
    // 体积计算结果
    struct VolumeResult {
        float positive_volume; // 正的体积[挖方]
        float negative_volume; // 负的体积[填方]
        float total_volume; // 挖方加上填方
        float diff_volume; // 挖方减去填方
        float positive_area; // 平面面积（正部分）
        float negative_area; // 平面面积（负部分）
        float total_area; // 全部的平面面积
        vtkSmartPointer<vtkPolyData> polyData; // 多边形数据
        pcl::PointCloud<PointT>::Ptr grid_top; // 挖方顶面网格点云 (立法体的顶面中心点)
        pcl::PointCloud<PointT>::Ptr grid_bottom; // 挖方顶面网格点云 (立法体的底面中心点)
    };




    void readPcd(const std::string& filename, pcl::PointCloud<PointT>::Ptr& cloud);
    VolumeResult getEfficientVolume(pcl::PointCloud<PointT>::Ptr cloud_src, Eigen::Vector3f normal, PointT point, float leaf_size);

 

    

}
#endif // VOLUME_H
#pragma once
