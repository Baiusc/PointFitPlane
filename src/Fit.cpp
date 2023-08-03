#pragma once
#include "MyFunc.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


using namespace MyFunc;


pcl::PointCloud<PointT>::Ptr cloud_input(new pcl::PointCloud<PointT>); //输入的隧道点云


#pragma region 详细步骤方法
void segmentPointCloud(const pcl::PointCloud<PointT>::Ptr cloud) {

    // 创建一个SACSegmentation对象，并设置模型类型为平面，方法类型为RANSAC
    pcl::SACSegmentation<PointT> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);

    // 设置距离阈值为0.01
    seg.setDistanceThreshold(0.01);

    // 设置输入点云
    seg.setInputCloud(cloud);

    // 创建一个模型系数对象和一个内点索引对象
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // 调用segment方法进行分割
    seg.segment(*inliers, *coefficients);

    // 输出分割结果
    std::cout << "Model coefficients: " << coefficients->values[0] << " "
        << coefficients->values[1] << " "
        << coefficients->values[2] << " "
        << coefficients->values[3] << std::endl;

    std::cout << "Model inliers: " << inliers->indices.size() << std::endl;
}
#pragma endregion


int main(int argc, char** argv) 
{

    PointT point; 	//输入：一个三维点 
    point.x = -1.70;
    point.y = -4.08;
    point.z = -5.24;
    std::cout << "点: ( " << point.x << " , " << point.y << " , " << point.z << " )" << std::endl;

    readPcd("../cloud/Tunnel.pcd", cloud_input); // 输入：隧道点云
    segmentPointCloud(cloud_input);

    return 0;
}

