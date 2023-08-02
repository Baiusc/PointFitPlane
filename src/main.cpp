#include "MyFunc.h"
using namespace MyFunc;
std::mutex UpdateMutex;
pcl::PointCloud<PointT>::Ptr cloud_src(new pcl::PointCloud<PointT>); //输入的隧道点云
//pcl::visualization::PCLVisualizer viewer("没有PCLVisualizer单开vtk会报错"); // Warning: Link to vtkInteractionStyle for default style selection
int main_a(int argc, char** argv)
{
#ifdef _OPENMP
	std::cout << "OpenMP support is enabled" << std::endl;
#else
	std::cout << "OpenMP support is disabled" << std::endl;
#endif
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