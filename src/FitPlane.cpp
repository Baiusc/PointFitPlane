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
#pragma endregion

#pragma region 详细步骤方法
// 单次分割
void segmentCloud_Single(const pcl::PointCloud<PointT>::Ptr cloud) {

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
	std::cout << "模型系数: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;

	std::cout << "模型内点: " << inliers->indices.size() << std::endl;

	// 创建一个新的点云来存储分割出的平面
	pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>);

	// 从原始点云中提取分割出的平面
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*plane);

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
	seg.setDistanceThreshold(0.01);

	// 深复制一份cloud
	pcl::PointCloud<PointT>::Ptr cloud_copy(new pcl::PointCloud<PointT>(*cloud));

	// 设置输入点云
	seg.setInputCloud(cloud_copy);

	// 创建一个模型系数对象和一个内点索引对象
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// 迭代分割
	while (cloud_copy->points.size() > 0.3 * cloud->points.size()) {
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

		// 计算平面的包围盒
		Eigen::Vector4f min_pt, max_pt;
		pcl::getMinMax3D(*plane, min_pt, max_pt);

		// 检查选中点是否在分割出的平面上
		if (cloud->points[selected_point_index].x >= min_pt[0] && cloud->points[selected_point_index].x <= max_pt[0] &&
			cloud->points[selected_point_index].y >= min_pt[1] && cloud->points[selected_point_index].y <= max_pt[1] &&
			cloud->points[selected_point_index].z >= min_pt[2] && cloud->points[selected_point_index].z <= max_pt[2]) {
			// 选中点在分割出的平面上
			// 将分割出的平面颜色改为红色
			for (size_t i = 0; i < plane->points.size(); ++i) {
				plane->points[i].r = 255;
				plane->points[i].g = 0;
				plane->points[i].b = 0;
			}
			// 将分割出的平面添加回原始点云中
			*cloud += *plane;
			std::cout << "找到了点所在的平面！" << std::endl;

			break;
		}

		// 从原始点云中移除本次分割出的点们
		extract.setNegative(true);
		extract.filter(*cloud_copy);
	}
	std::cout << "while循环结束！" << std::endl;
}



#pragma endregion

#pragma region PCL可视化

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
	// 选点拟合平面
	segmentCloud(cloud_input, idx);
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
void addViewport(pcl::visualization::PCLVisualizer& viewer, int viewport, int count = 1, double r = 0.0, double g = 0.0, double b = 0.0)
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

	readPcd("../cloud/table.pcd", cloud_input); // 输入：隧道点云
	//segmentCloud(cloud_input, point);

	// PCL处理过程可视化

	initViewer(viewer, cloud_input, selected_point);
	addViewport(viewer, 1);
	/*  addViewport(viewer, 2);
	  addViewport(viewer, 3);
	  addViewport(viewer, 4);*/

	addCloud(viewer, cloud_input, 1);



	//viewer.spin();
	while (!viewer.wasStopped()) 
	{
		viewer.spinOnce(100); 
	}

	return 0;
}
