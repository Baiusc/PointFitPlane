#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>



typedef pcl::PointXYZRGB PointT;
pcl::PointCloud<PointT>::Ptr cloud_src(new pcl::PointCloud<PointT>); //输入的隧道点云


#pragma region 详细步骤方法
void segmentPointCloud(const std::string& filename) {

    // 创建一个点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取点云文件
    pcl::PCDReader reader;
    reader.read(filename, *cloud);

    // 创建一个SACSegmentation对象，并设置模型类型为平面，方法类型为RANSAC
    pcl::SACSegmentation<pcl::PointXYZ> seg;
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

    readPcd("../cloud/Tunnel.pcd", cloud_src); // 输入：隧道点云
    std::string filename = "table_scene_lms400.pcd";
    segmentPointCloud(filename);

    return 0;
}

int main_a(int argc, char** argv)
{

	float leaf_size = 0.02; // 输入：分辨率
	readPcd("../cloud/Tunnel.pcd", cloud_src); // 输入：隧道点云
	//Eigen::Vector3f normal(0.0, 0.0, 1.0); 	// 输入：法向
	Eigen::Vector3f normal(0.35, 0.22, 0.91); // 模拟非z法向 
	PointT point; 	//输入：一个三维点 
	point.x = -1.70;
	point.y = -4.08;
	point.z = -5.24;
	std::cout << "点: ( " << point.x << " , " << point.y << " , " << point.z << " )" << std::endl;
	std::cout << "分辨率: " << leaf_size << "m" << std::endl;

	return 0;
}