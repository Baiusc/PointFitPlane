#include "pch.h"

// 定义点云类型模板
typedef pcl::PointXYZRGB PointT;

pcl::PointCloud<PointT>::Ptr cloud_input(new pcl::PointCloud<PointT>); //输入的隧道点云
pcl::visualization::PCLVisualizer viewer("MutiViewer");

#pragma region 读写操作和格式转换
// 读pcd
void readPcd(const std::string& filename, pcl::PointCloud<PointT>::Ptr& cloud)
{
	pcl::PCDReader reader;
	reader.read(filename, *cloud);
	std::cout << "cloud_src has: " << cloud->points.size() << " data points." << std::endl;

}
/// <summary>
/// 保存点云为txt
/// </summary>
/// <param name="filename">路径</param>
/// <param name="pclCloud">pcl点云对象</param>
void save2txt(const std::string& filename, const pcl::PointCloud<PointT>::Ptr& pclCloud)
{
	if (filename == "") return;

	std::ofstream file(filename);
	if (file.is_open())
	{
		for (const auto& point : pclCloud->points)
		{
			file << point.x << " " << point.y << " " << point.z << std::endl;
		}
		file.close();
		std::cout << "Point cloud saved to " << filename << std::endl;
	}
	else
	{
		std::cerr << "Unable to open file " << filename << std::endl;
	}
}
// 0f~1.0f的hsv转成0f~1.0f的rgb
void hsv2rgb(float h, float s, float v, float* r, float* g, float* b)
{
	float f, x, y, z;
	int i;
	if (s == 0.0) {
		*r = *g = *b = v;
	}
	else {
		h = fmod(h * 360.0, 360.0) / 60.0;
		i = (int)h;
		f = h - i;
		x = v * (1.0 - s);
		y = v * (1.0 - (s * f));
		z = v * (1.0 - (s * (1.0 - f)));
		switch (i) {
		case 0: *r = v; *g = z; *b = x; break;
		case 1: *r = y; *g = v; *b = x; break;
		case 2: *r = x; *g = v; *b = z; break;
		case 3: *r = x; *g = y; *b = v; break;
		case 4: *r = z; *g = x; *b = v; break;
		case 5: *r = v; *g = x; *b = y; break;
		}
	}
}
#pragma endregion

#pragma region 通用点云算法

// 点云投影
void projectPointCloudToPlane(pcl::PointCloud<PointT>::Ptr& cloud, pcl::ModelCoefficients::Ptr& plane_coefficients, pcl::PointCloud<PointT>::Ptr& cloud_projected)
{
	auto time_start = std::clock();
	// 创建滤波器对象
	pcl::ProjectInliers<PointT> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(plane_coefficients);
	proj.filter(*cloud_projected);
	auto time_end = std::clock();
	cloud_projected->width = cloud_projected->size();
	std::cerr << "投影生成挖方底面，耗时：" << std::difftime(time_end, time_start) << "ms" << std::endl;
}

/// <summary>
/// 已知一个平面上的三维点和该平面的法向量，求该平面方程系数
/// </summary>
/// <param name="point">平面上的三维点</param>
/// <param name="normal">该平面的法向量</param>
void calcPlaneCoefficients(const PointT& point, const Eigen::Vector3f& normal, pcl::ModelCoefficients::Ptr& plane_coefficients)
{
	Eigen::Vector3f  unit_normal = normal.normalized();
	plane_coefficients->values.resize(4);
	plane_coefficients->values[0] = unit_normal.x();
	plane_coefficients->values[1] = unit_normal.y();
	plane_coefficients->values[2] = unit_normal.z();
	plane_coefficients->values[3] = -(unit_normal.x() * point.x + unit_normal.y() * point.y + unit_normal.z() * point.z);

	std::cout << "Plane coefficients: ";
	for (const float value : plane_coefficients->values)
	{
		std::cout << value << " ";
	}
	std::cout << std::endl;
}

// 已知两个向量分别为n1和n2，求旋转矩阵
Eigen::Matrix4f getRotationMatrix(const Eigen::Vector3f& n1, const Eigen::Vector3f& n2) {
	// 计算旋转轴
	Eigen::Vector3f axis = n1.cross(n2);

	// 计算旋转角度
	float cos_angle = n1.dot(n2) / (n1.norm() * n2.norm());
	float angle = acos(cos_angle);

	// 构造旋转矩阵
	Eigen::AngleAxisf rotation(angle, axis.normalized());
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform.block<3, 3>(0, 0) = rotation.matrix();
	return transform;
}

// 获取octree体素中心点
void getVoxelCenters_old(pcl::PointCloud<PointT>::Ptr cloud_src, float leaf_size, pcl::PointCloud<PointT>::Ptr voxel_cloud)
{
	auto time_start = std::clock();
	// 创建一个八叉树对象
	pcl::octree::OctreePointCloud<PointT> octree(leaf_size); // 设置体素分辨率
	// 设置输入点云数据
	octree.setInputCloud(cloud_src);
	// 构建八叉树索引
	octree.addPointsFromInputCloud();
	// 获取被占用体素中心点
	std::vector<PointT, Eigen::aligned_allocator<PointT>> voxel_centers;
	octree.getOccupiedVoxelCenters(voxel_centers);
	// 将体素中心点添加到输出点云中
	voxel_cloud->clear();
	for (const auto& point : voxel_centers) {
		voxel_cloud->push_back(point);
	}
	auto time_end = std::clock();
	std::cerr << "体素化，耗时：" << std::difftime(time_end, time_start) << "ms" << std::endl;
}
void getVoxelCenters(pcl::PointCloud<PointT>::Ptr cloud_src, float leaf_size, pcl::PointCloud<PointT>::Ptr voxel_cloud)
{
	auto time_start = std::clock();
	// 创建一个八叉树对象
	auto octree = std::make_shared<pcl::octree::OctreePointCloud<PointT>>(leaf_size); // 设置体素分辨率
	// 设置输入点云数据
	octree->setInputCloud(cloud_src);
	// 构建八叉树索引
	octree->addPointsFromInputCloud();
	// 获取被占用体素中心点
	auto voxel_centers = std::make_shared<std::vector<PointT, Eigen::aligned_allocator<PointT>>>();
	octree->getOccupiedVoxelCenters(*voxel_centers);
	// 将体素中心点添加到输出点云中
	voxel_cloud->clear();
	for (const auto& point : *voxel_centers) {
		voxel_cloud->push_back(point);
	}
	auto time_end = std::clock();
	std::cerr << "体素化，耗时：" << std::difftime(time_end, time_start) << "ms" << std::endl;
}


// 计算点到平面的距离 有正负
float calcPointToPlaneDistance(
	const PointT point,
	const pcl::ModelCoefficients::Ptr& plane_coefficients)
{
	// 距离值含正负  代表在平面的正侧还是反侧
	float distance = (point.x * plane_coefficients->values[0] +
		point.y * plane_coefficients->values[1] +
		point.z * plane_coefficients->values[2] +
		plane_coefficients->values[3]) / std::sqrtf(
			std::powf(plane_coefficients->values[0], 2) +
			std::powf(plane_coefficients->values[1], 2) +
			std::powf(plane_coefficients->values[2], 2)
		);;
	return distance;
}

/// <summary>
/// 两二维点云取交集 （kd树K邻近搜索）
/// </summary>
/// <param name="cloud_x"></param>
/// <param name="cloud_y"></param>
/// <param name="cloud_xy"></param>
/// <param name="radius">搜索半径，半径小于此值认为两个点相同</param>
void getIntersection_2d(const pcl::PointCloud<pcl::PointXY>::Ptr& cloud_x,
	const pcl::PointCloud<pcl::PointXY>::Ptr& cloud_y,
	pcl::PointCloud<pcl::PointXY>::Ptr& cloud_xy)
{
	auto time_start = std::clock();
	pcl::search::KdTree<pcl::PointXY>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXY>);
	kdtree->setInputCloud(cloud_y);
	int k = 1;
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;

	for (size_t i = 0; i < cloud_x->size(); i++)
	{
		if (kdtree->nearestKSearch(cloud_x->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			if (pointNKNSquaredDistance[0] < 0.00001)//如果搜索到的第一个点距为0，那么这个点为重合点（因为用A搜B中的点，不存在包含自身的情况）
			{
				cloud_xy->points.push_back(cloud_x->points[i]);
			}
		}
	}
	auto end1 = std::clock();
	std::cerr << "取交集，耗时：" << std::difftime(end1, time_start) << "ms" << std::endl;
}
// 以给定点为中心，分割出radius*height的圆柱体，输出圆柱体内的点云
void segmentRegionCylinder(pcl::PointCloud<PointT>::Ptr cloud_input, pcl::PointCloud<PointT>::Ptr cloud_cylinder, PointT selected_point, float radius, float height) {
	
	// 创建一个圆柱体对象
	pcl::ModelCoefficients cylinder_coeff;
	cylinder_coeff.values.resize(7);
	cylinder_coeff.values[0] = selected_point.x;
	cylinder_coeff.values[1] = selected_point.y;
	cylinder_coeff.values[2] = selected_point.z - height;
	cylinder_coeff.values[3] = 0;
	cylinder_coeff.values[4] = 0;
	cylinder_coeff.values[5] = height;
	cylinder_coeff.values[6] = radius;

	// 在可视化工具中添加圆柱体
	viewer.addCylinder(cylinder_coeff, "cylinder", 1);

	// 设置圆柱体的颜色和透明度
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "cylinder", 0);
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "cylinder", 0);

	// 遍历输入点云中的每个点
	for (const auto& point : cloud_input->points) {
		// 计算点到圆心的距离
		float distance = std::sqrt(std::pow(point.x - selected_point.x, 2) + std::pow(point.y - selected_point.y, 2));
		// 检查点是否在圆柱体内
		if (distance <= radius && point.z >= selected_point.z - height && point.z <= selected_point.z + height) {
			// 将点添加到筛选后的点云中
			cloud_cylinder->push_back(point);
		}
	}
}

// 以给定点为中心，分割出radius*height的圆柱体，输出圆柱体内的点云 (输出区域点云和索引数组)
void segmentRegionCylinder(pcl::PointCloud<PointT>::Ptr cloud_input, pcl::PointCloud<PointT>::Ptr cloud_cylinder, pcl::IndicesPtr indices, PointT selected_point, float radius, float height) {
	
	// 遍历输入点云中的每个点
	for (size_t i = 0; i < cloud_input->points.size(); ++i) {
		const auto& point = cloud_input->points[i];
		// 计算点到圆心的距离
		float distance = std::sqrt(std::pow(point.x - selected_point.x, 2) + std::pow(point.y - selected_point.y, 2));
		// 检查点是否在圆柱体内
		if (distance <= radius && point.z >= selected_point.z - height/2.0f && point.z <= selected_point.z + height/2.0f) {
			// 将点的索引添加到筛选后的索引数组中
			indices->push_back(i);
			// 将点添加到筛选后的点云中
			cloud_cylinder->push_back(point);
		}
	}
}




#pragma endregion

#pragma region 详细步骤方法
// 单次分割
void segmentCloud_Single(const pcl::PointCloud<PointT>::Ptr cloud) {

	// 创建一个SACSegmentation对象，方法类型为RANSAC，并设置模型类型为圆柱
	pcl::SACSegmentation<PointT> seg;
	seg.setModelType(pcl::SACMODEL_CYLINDER);
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

	// 创建一个新的点云来存储分割出的平面
	pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>);

	// 从原始点云中提取分割出的平面
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*plane);

	// 输出分割结果
	std::cout << "模型系数: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;

	std::cout << "模型内点: " << inliers->indices.size() << std::endl;

	// 将分割出的平面颜色改为红色
	for (size_t i = 0; i < plane->points.size(); ++i) {
		plane->points[i].r = 255;
		plane->points[i].g = 0;
		plane->points[i].b = 0;
	}

	// 将分割出的平面添加回原始点云中
	*cloud += *plane;


}
// 迭代分割
void segmentCloud(const pcl::PointCloud<PointT>::Ptr cloud, int selected_point_index)
{
	// 创建一个SACSegmentation对象，并设置模型类型为平面，方法类型为RANSAC
	pcl::SACSegmentation<PointT> seg;
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);

	// 设置距离阈值
	seg.setDistanceThreshold(0.1);

	// 深复制一份cloud
	pcl::PointCloud<PointT>::Ptr cloud_copy(new pcl::PointCloud<PointT>(*cloud));
	pcl::PointCloud<PointT>::Ptr cloud_draw(new pcl::PointCloud<PointT>(*cloud));

	// 设置输入点云
	seg.setInputCloud(cloud_copy);

	// 创建一个模型系数对象和一个内点索引对象
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// 迭代分割
	while (cloud_copy->points.size() > 0.001 * cloud->points.size()) {
		// 调用segment方法进行分割
		seg.segment(*inliers, *coefficients);

		// 检查是否分割成功
		if (inliers->indices.size() == 0) {
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// 创建一个新的点云来存储分割出的平面
		pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>);

		// 从原始点云中提取分割出的平面
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloud_copy);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*plane);

		// 计算选中点到平面的距离
		float distance = calcPointToPlaneDistance(cloud->points[selected_point_index], coefficients);

		// 检查选中点是否在分割出的平面上
		if (std::abs(distance) < 0.01) { // 将0.01替换为您想要使用的阈值
			// 选中点在分割出的平面上
			// 将分割出的平面颜色改为红色
			for (size_t i = 0; i < plane->points.size(); ++i) {

				plane->points[i].r = 0;
				plane->points[i].g = 255;
				plane->points[i].b = 0;
			}
			// 将分割出的平面添加回原始点云中
			*cloud_draw += *plane;
			std::cout << "找到了点所在的平面！" << std::endl;
			// 在查看器中更新点云
			std::string cloud_id = "cloud" + std::to_string(1); // 将1替换为适当的视口编号
			viewer.updatePointCloud(cloud_draw, cloud_id);
			break;
		}

		// 从原始点云中移除本次分割出的点们
		extract.setNegative(true);
		extract.filter(*cloud_copy);
	}
	std::cout << "while循环结束！" << std::endl;
}

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>


#include <pcl/features/normal_3d_omp.h>

void regionGrowingSegmentation(pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointIndices>& clusters)
{
	// 建立搜索KD树
	pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	// 计算点云法向
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_estimator;
	//pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree); // 搜索方法为kd树走索
	normal_estimator.setInputCloud(cloud);  // 填入点云
	normal_estimator.setKSearch(50);        // 设置搜索范围
	normal_estimator.compute(*normals);     // 将法相保存在normals
	pcl::IndicesPtr indices(new std::vector<int>);
	pcl::removeNaNFromPointCloud(*cloud, *indices); // 对点云建立索引

	pcl::RegionGrowing<PointT, pcl::Normal> reg; // 区域增长类
	reg.setMinClusterSize(50);                   // 设置最小的集合点数
	reg.setMaxClusterSize(1000000);               // 设置最大集合点数
	reg.setSearchMethod(tree);                    // 设置kd树搜索方法
	reg.setNumberOfNeighbours(10);                // 设置每次邻域搜索数(影响计算速度)
	reg.setInputCloud(cloud);                     // 设置输入点云
	reg.setIndices(indices);                      // 设置输入的索引
	reg.setInputNormals(normals);                 // 设置输入法向
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);      // 设置平滑度阈值（弧度）
	reg.setCurvatureThreshold(1.0);                      // 设置曲率阈值
	// 分类集合 并开始计算
	reg.extract(clusters);
	// 一系列输出
	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << std::endl;
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;
	std::size_t counter = 0;
	//while (counter < clusters[0].indices.size())
	//{
	//	std::cout << clusters[0].indices[counter] << ", ";
	//	counter++;
	//	if (counter % 10 == 0)
	//		std::cout << std::endl;
	//}
	std::cout << std::endl;
	// 显示出分割后的点云，并赋予不同颜色
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped())
	{
	}
}
/**
 * 计算选中点的邻域
 * @param cloud 输入点云
 * @param selected_point_index 选中点的索引
 * @return 选中点所在的光滑表面上的所有点的索引
 */
void computeSelectedPointNeighborhood(
	const pcl::PointCloud<PointT>::Ptr& cloud,
	int selected_point_index)
{
	// 建立搜索KD树
	pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

	// 计算点云法向
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree); // 搜索方法为kd树走索
	normal_estimator.setInputCloud(cloud);  // 填入点云
	normal_estimator.setKSearch(50);        // 设置搜索范围
	normal_estimator.compute(*normals);     // 将法相保存在normals

	// 区域增长分割
	pcl::RegionGrowing<PointT, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	// 设置选中点的索引
	pcl::IndicesPtr indices(new std::vector<int>);
	indices->push_back(selected_point_index);
	reg.setIndices(indices);

	// 提取光滑表面
	std::vector<pcl::PointIndices> clusters;
	reg.extract(clusters);

	// 获取选中点所在的光滑表面
	if (!clusters.empty())
	{
		pcl::PointIndices selected_cluster = clusters[0];
		//return selected_cluster.indices;
			// 显示出分割后的点云，并赋予不同颜色
		pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
		pcl::visualization::CloudViewer viewer("Selected Cluster viewer");
		viewer.showCloud(colored_cloud);
		while (!viewer.wasStopped())
		{
		}
	}

	//return std::vector<int>();
}








#pragma endregion

#pragma region PCL可视化
// 可视化圆柱体几何形状
void addCylinder(pcl::visualization::PCLVisualizer& viewer, PointT selected_point, float radius, float height ) {
	// 创建一个圆柱体对象
	pcl::ModelCoefficients cylinder_coeff;
	cylinder_coeff.values.resize(7);
	cylinder_coeff.values[0] = selected_point.x;
	cylinder_coeff.values[1] = selected_point.y;
	cylinder_coeff.values[2] = selected_point.z - height/2.0f;
	cylinder_coeff.values[3] = 0;
	cylinder_coeff.values[4] = 0;
	cylinder_coeff.values[5] = height;
	cylinder_coeff.values[6] = radius;

	// 在可视化工具中添加圆柱体
	viewer.addCylinder(cylinder_coeff, "cylinder", 1);

	// 设置圆柱体的颜色和透明度
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "cylinder", 1);
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "cylinder", 1);
}

// 鼠标单击事件回调函数
void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* viewer_void) {
	std::cout << "[INFO] Point picking event occurred." << std::endl;
	// 检查是否获取到了选中的点
	if (event.getPointIndex() == -1)
		return;
	float x, y, z;
	event.getPoint(x, y, z); 	// 获取选中点的坐标
	PointT selected_point;
	selected_point.x = x;
	selected_point.y = y;
	selected_point.z = z;
	int idx = event.getPointIndex(); 	// 获取选中点的索引
	// 在终端输出选中点的坐标
	std::cout << "选点：x=" << x << ", y=" << y << ", z=" << z << ", idx=" << idx << std::endl;

	// 重绘选中的点 变色 变大
	pcl::PointCloud<PointT>::Ptr selected_point_cloud(new pcl::PointCloud<PointT>);
	selected_point_cloud->push_back(selected_point);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red_color(selected_point_cloud, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> green_color(selected_point_cloud, 0, 255, 0);
	// 检查"selected_point"是否已存在，若已存在，则先从viewer 中remove "selected_point"
	if (viewer.contains("selected_point")) {
		viewer.removePointCloud("selected_point", 1);
	}
	viewer.addPointCloud(selected_point_cloud, red_color, "selected_point",1);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 30, "selected_point");

	// 分割出圆柱区域
	float radius = 2.0f;
	float height = 30.0f;
	pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>);
	pcl::IndicesPtr region_indices(new std::vector<int>);
	addCylinder(viewer, selected_point, radius, height);
	segmentRegionCylinder(cloud_input, cloud_cylinder, region_indices, selected_point, radius, height);
	// 将分割出的平面颜色改为红色
	for (size_t i = 0; i < region_indices->size(); ++i) {
		int idx = (*region_indices)[i];
		cloud_input->points[idx].r = 0;
		cloud_input->points[idx].g = 255;
		cloud_input->points[idx].b = 0;
	}
	// 更新可视化工具中的点云数据
	viewer.updatePointCloud(cloud_input, "cloud1");

	// 区域拟合平面
	//segmentCloud_Single(cloud_cylinder);
	//computeSelectedPointNeighborhood(cloud_input, idx);
}


// 初始化viewer
void initViewer(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<PointT>::Ptr& cloud, PointT selected_point)
{
	viewer.initCameraParameters();
	viewer.setBackgroundColor(0, 0, 0);
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);
	viewer.setCameraPosition(centroid[0], centroid[1], centroid[2] + 10.0f, centroid[0], centroid[1], centroid[2], 0, 1, 0);
	viewer.registerPointPickingCallback(pointPickingCallback, (void*)&viewer);
}
// 添加viewport (从1开始)
void addViewport(pcl::visualization::PCLVisualizer& viewer, int viewport, int count = 1, double r = 6.0/255.0, double g = 60.0/255.0, double b = 90.0/255.0)
{
	double x_min = (viewport - 1) * (1.0 / count);
	double x_max = viewport * (1.0 / count);
	viewer.createViewPort(x_min, 0.0, x_max, 1.0, viewport);
	//viewer.createViewPort((viewport - 1) * 0.25, 0.0, viewport * 0.25, 1.0, viewport); // 这种写法只能保证4个视口不重叠排列
	viewer.setBackgroundColor(r, g, b, viewport);
	//viewer.addCoordinateSystem(3, "coordinate", viewport);
	std::string x_label = "x_label_v" + std::to_string(viewport);
	std::string y_label = "y_label_v" + std::to_string(viewport);
	std::string z_label = "z_label_v" + std::to_string(viewport);
	//viewer.addText3D("x", pcl::PointXYZ(11, 0, 0), 1.0, 1.0, 1.0, 1.0, x_label, viewport);
	//viewer.addText3D("y", pcl::PointXYZ(0, 11, 0), 1.0, 1.0, 1.0, 1.0, y_label, viewport);
	//viewer.addText3D("z", pcl::PointXYZ(0, 0, 11), 1.0, 1.0, 1.0, 1.0, z_label, viewport);
	// 标题
	viewer.addText("viewport " + std::to_string(viewport), 10, 10, std::to_string(viewport), viewport);
}
// 添加点云 不改变颜色
void addCloud(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<PointT>::Ptr& cloud, int viewport)
{
	std::string cloud_id = "cloud" + std::to_string(viewport);
	viewer.addPointCloud(cloud, cloud_id, viewport);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_id);
}
// 添加点云 Z值着色
void addCloud(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<PointT>::Ptr& cloud, int viewport, std::string axis)
{
	pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(cloud, axis); // 按Z轴值着色
	std::string cloud_id = "cloud" + std::to_string(viewport);
	viewer.addPointCloud(cloud, color_handler, cloud_id, viewport);
}
// 添加点云 自定义hsv
void addCloud(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<PointT>::Ptr& cloud, float hue, int viewport)
{
	// 将HSV值转换为RGB
	float r, g, b;
	hsv2rgb(hue, 1.0, 1.0, &r, &g, &b);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> color(cloud, r * 255.0, g * 255.0, b * 255.0);
	std::string cloud_id = "cloud" + std::to_string(viewport) + std::to_string(hue);
	viewer.addPointCloud(cloud, color, cloud_id, viewport);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_id);
}

#pragma endregion


int main(int argc, char** argv)
{
	PointT selected_point; 	//输入：一个三维点 

	readPcd("../cloud/Cylinder.pcd", cloud_input); // 输入：隧道点云


	std::vector<pcl::PointIndices> clusters;
	// 调用区域增长分割方法
	//regionGrowingSegmentation(cloud_input, clusters);
	//segmentCloud(cloud_input, point);

	// PCL处理过程可视化

	initViewer(viewer, cloud_input, selected_point);
	addViewport(viewer, 1);
	/*  addViewport(viewer, 2);
	  addViewport(viewer, 3);
	  addViewport(viewer, 4);*/

	addCloud(viewer, cloud_input, 1);
	//addCloud(viewer, cloud_input, 1, "z");


	//viewer.spin();
	while (!viewer.wasStopped()) 
	{
		viewer.spinOnce(100); 
	}

	return 0;
}
