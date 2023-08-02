#include "CMakeProject1.h"
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
#include <pcl/visualization/common/shapes.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/geometry/mesh_conversion.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>	
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/common.h>
#include <pcl/common/colors.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
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
// 通用输入输出点云类型
typedef pcl::PointXYZRGB PointT;
// 所有对象

// PCL点云库读取器，使用它读取PCD格式的点云文件
pcl::PCDReader reader;
// 空间过滤器，通过保留指定范围内的点云，来滤波处理
pcl::PassThrough<PointT> pass;
// 法线估计器，计算点云表面的法线信息
pcl::NormalEstimation<PointT, pcl::Normal> ne;
// 法线约束的平面分割器，用于将点云分割成局内点 (inliers) 和局外点 (outliers)
//pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
// PCD文件写入器，可以将点云数据保存为PCD格式的文件
pcl::PCDWriter writer;
// 提取局内点的点云提取器
pcl::ExtractIndices<PointT> extract;
// 提取局内点法线信息的提取器
pcl::ExtractIndices<pcl::Normal> extract_normals;
// KdTree搜索对象，用于在3D空间中搜索点云
pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

// Datasets 数据
pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>); //原点云
pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_ground(new pcl::PointCloud<PointT>); //地面
pcl::PointCloud<PointT>::Ptr cloud_offground(new pcl::PointCloud<PointT>); //非地面
pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients); //点法确定挖方底平面系数
pcl::PointCloud<PointT>::Ptr cloud_top(new pcl::PointCloud<PointT>); //挖方顶面点云
pcl::PointCloud<PointT>::Ptr cloud_top_voxelized(new pcl::PointCloud<PointT>); //体素化后的点云
pcl::PointCloud<PointT>::Ptr cloud_top_standard(new pcl::PointCloud<PointT>); //网格体素化后的挖方顶面点云
pcl::PointCloud<PointT>::Ptr cloud_bottom(new pcl::PointCloud<PointT>); //挖方底面点云
pcl::PointCloud<PointT>::Ptr grid_bottom(new pcl::PointCloud<PointT>); //网格体素化后的挖方底面点云
pcl::PointCloud<PointT>::Ptr grid_bottom_repairX(new pcl::PointCloud<PointT>); //修补X的结果
pcl::PointCloud<PointT>::Ptr grid_bottom_repairY(new pcl::PointCloud<PointT>); //修补Y的结果
pcl::PointCloud<PointT>::Ptr grid_bottom_repair(new pcl::PointCloud<PointT>); //修补后的挖方底面网格点云
pcl::PointCloud<PointT>::Ptr grid_top_repair(new pcl::PointCloud<PointT>); //修补后的挖方顶面网格点云

pcl::PointCloud<PointT>::Ptr cloud_bottom_cloth(new pcl::PointCloud<PointT>); //挖方底面网格点云的简单外轮廓
pcl::PointCloud<PointT>::Ptr cloud_concave_hull(new pcl::PointCloud<PointT>); //挖方底面网格点云的凹壳
pcl::PointCloud<PointT>::Ptr cloud_mid(new pcl::PointCloud<PointT>); //挖方内部点云
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
//PCLVisualizer 可视化
pcl::visualization::PCLVisualizer viewer("Cylinder Segmentation666");

int step_count;

//#pragma region 没用到的方法
///// <summary>
///// 按轴坐标最小值和最大值进行过滤
///// </summary>
//void pass_through_filter()
//{
//	// 创建空间过滤器，去除无效的NaN值
//	pass.setInputCloud(cloud);
//	pass.setFilterFieldName("z"); // 指定在哪个轴上过滤
//	pass.setFilterLimits(0, 1.5); // 滤波范围
//	pass.filter(*cloud_filtered); // 输出过滤后的结果
//	std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;
//}
///// <summary>
///// 估计点云法线
///// </summary>
//void normals_estimate()
//{
//	// 估计点云法线
//	ne.setSearchMethod(tree);
//	ne.setInputCloud(cloud_filtered);
//	ne.setKSearch(50); // 搜索邻居的点数
//	ne.compute(*cloud_normals); // 输出法线结果
//}
///// <summary>
///// 平面分割
///// </summary>
//void plane_seg()
//{
//	// 创建一个用于分割平面模型的对象，并设置所有参数
//	seg.setOptimizeCoefficients(true);
//	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
//	seg.setNormalDistanceWeight(0.1);
//	seg.setMethodType(pcl::SAC_RANSAC);
//	seg.setMaxIterations(100);
//	seg.setDistanceThreshold(0.03);
//	seg.setInputCloud(cloud_filtered);
//	seg.setInputNormals(cloud_normals);
//	// 获取平面模型的内点以及系数
//	seg.segment(*inliers_plane, *coefficients_plane);
//	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
//}
//
//void get_plane()
//{
//	// 从输入点云中提取平面内的点
//	extract.setInputCloud(cloud_filtered);
//	extract.setIndices(inliers_plane); // 指定平面内的内点索引
//	extract.setNegative(false);
//
//	// 写出平面内的点到文件中
//
//	extract.filter(*cloud_plane);
//	std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
//	writer.write("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);
//}
//
//void remove_plane()
//{
//	// Remove the planar inliers, extract the rest
//	extract.setNegative(true);
//	extract.filter(*cloud_filtered2);
//	extract_normals.setNegative(true);
//	extract_normals.setInputCloud(cloud_normals);
//	extract_normals.setIndices(inliers_plane);
//	extract_normals.filter(*cloud_normals2);
//}
//
//void cylinder_seg()
//{
//	// Create the segmentation object for cylinder segmentation and set all the parameters
//	seg.setOptimizeCoefficients(true);
//	seg.setModelType(pcl::SACMODEL_CYLINDER);
//	seg.setMethodType(pcl::SAC_RANSAC);
//	seg.setNormalDistanceWeight(0.1);
//	seg.setMaxIterations(10000);
//	seg.setDistanceThreshold(0.05);
//	seg.setRadiusLimits(0, 0.1);
//	seg.setInputCloud(cloud_filtered2);
//	seg.setInputNormals(cloud_normals2);
//
//	// Obtain the cylinder inliers and coefficients
//	seg.segment(*inliers_cylinder, *coefficients_cylinder);
//	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
//}
//void get_cylinder()
//{
//	extract.setInputCloud(cloud_filtered2);
//	extract.setIndices(inliers_cylinder);
//	extract.setNegative(false);
//
//	extract.filter(*cloud_cylinder);
//	if (cloud_cylinder->points.empty())
//		std::cerr << "Can't find the cylindrical component." << std::endl;
//	else
//	{
//		std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;
//		writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
//	}
//}
//
//#pragma endregion

#pragma region 纯数学计算


// 0f~1.0f的hsv转成0f~1.0f的rgb
void hsv2rgb(float h, float s, float v, float* r, float* g, float* b)
{
	float f, x, y, z;
	int i;
	//v *= 255.0; 如果转成0~255的RGB，则启用这一句
	if (s == 0.0) {
		*r = *g = *b = (int)v;
	}
	else {
		while (h < 0)
			h += 360;
		h = fmod(h, 360) / 60.0;
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

#pragma region 类型转换
/// <summary>
/// pcl和csf库的点云对象类型转换
/// </summary>
/// <param name="pclCloud">pcl库的点云对象</param>
/// <param name="csfCloud">csf库的点云对象</param>
void pcl2csf(const pcl::PointCloud<PointT> pclCloud, csf::PointCloud& csfCloud)
{
	for (const auto& pclPoint : pclCloud)
	{
		csf::Point csfPoint;
		csfPoint.x = pclPoint.x;
		csfPoint.y = pclPoint.y; // 将pcl点云的z坐标赋值给csf点云的y坐标
		csfPoint.z = pclPoint.z; // 将pcl点云的y坐标取负并赋值给csf点云的z坐标
		csfCloud.push_back(csfPoint);
	}
}
void csf2pcl(const csf::PointCloud& csfCloud, pcl::PointCloud<PointT>& pclCloud)
{
	for (const auto& csfPoint : csfCloud)
	{
		PointT pclPoint;
		pclPoint.x = csfPoint.x;
		pclPoint.y = csfPoint.z; // 将csf点云的z坐标赋值给pcl点云的y坐标
		pclPoint.z = -csfPoint.y; // 将csf点云的y坐标取负并赋值给pcl点云的z坐标
		pclCloud.push_back(pclPoint);
	}
}
#pragma endregion

#pragma region 可复用的方法
// 计算点到平面的距离 有正负
float calcPointToPlaneDistance(
	const PointT point,
	const pcl::ModelCoefficients::Ptr& plane_coefficients)
{
	// 距离值含正负  代表在平面的正侧还是反侧
	float distance = (point.x * plane_coefficients->values[0] +
		point.y * plane_coefficients->values[1] +
		point.z * plane_coefficients->values[2] +
		plane_coefficients->values[3]) / std::sqrt(
			std::pow(plane_coefficients->values[0], 2) +
			std::pow(plane_coefficients->values[1], 2) +
			std::pow(plane_coefficients->values[2], 2)
		);;
	return distance;
}
/// <summary>
/// 由网格底面的点反算出对应顶面点 （反投影）
/// </summary>
/// <param name="A">挖方底面网格的某点</param>
/// <param name="plane_coefficients">挖方底面的平面方程系数</param>
/// <param name="distance">挖方顶面对应点到point_bottom的距离 有正负</param>
/// <returns>对应顶面点坐标</returns>
PointT calcTopPoint(const PointT& A, const pcl::ModelCoefficients::Ptr& plane_coefficients, float distance)
{
	// 平面法向量
	Eigen::Vector3f plane_normal(plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2]);
	// 单位化平面法向量
	Eigen::Vector3f plane_unit_normal = plane_normal.normalized();
	// 计算点B的坐标
	PointT B;
	B.x = A.x + distance * plane_unit_normal.x();
	B.y = A.y + distance * plane_unit_normal.y();
	B.z = A.z + distance * plane_unit_normal.z();

	return B;
}
/// <summary>
/// 由网格底面的点反算出对应顶面点
/// </summary>
/// <param name="A">挖方底面网格的某点</param>
/// <param name="normal">挖方底面的法向量之一</param>
/// <param name="distance">挖方顶面对应点到point_bottom的距离 有正负</param>
/// <returns>对应顶面点坐标</returns>
PointT calcTopPoint(const PointT& A, const Eigen::Vector3f& normal, float distance)
{
	// 单位化平面法向量
	Eigen::Vector3f plane_unit_normal = normal.normalized();
	// 计算点B的坐标
	PointT B;
	B.x = A.x + distance * plane_unit_normal.x();
	B.y = A.y + distance * plane_unit_normal.y();
	B.z = A.z + distance * plane_unit_normal.z();

	return B;
}
/// <summary>
/// 两点云取交集 （kd树K邻近搜索）
/// </summary>
/// <param name="cloud_x"></param>
/// <param name="cloud_y"></param>
/// <param name="cloud_xy"></param>
/// <param name="radius">搜索半径，半径小于此值认为两个点相同</param>
void getIntersection_NK(const pcl::PointCloud<PointT>::Ptr& cloud_x,
	const pcl::PointCloud<PointT>::Ptr& cloud_y,
	pcl::PointCloud<PointT>::Ptr& cloud_xy)
{
	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	kdtree->setInputCloud(cloud_y);
	int k = 1;
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;

	cloud_xy->points.clear();
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
}
/// <summary>
/// 两点云取交集 （kd树半径搜索）
/// </summary>
/// <param name="cloud_x"></param>
/// <param name="cloud_y"></param>
/// <param name="cloud_xy"></param>
/// <param name="radius">搜索半径，半径小于此值认为两个点相同</param>
void getIntersection_R(const pcl::PointCloud<PointT>::Ptr& cloud_x,
	const pcl::PointCloud<PointT>::Ptr& cloud_y,
	pcl::PointCloud<PointT>::Ptr& cloud_xy,
	float radius = 0.00001)
{
	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	kdtree->setInputCloud(cloud_y);
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	cloud_xy->points.clear();
	for (size_t i = 0; i < cloud_x->size(); i++)
	{
		if (kdtree->radiusSearch(cloud_x->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			cloud_xy->points.push_back(cloud_x->points[i]);
		}
	}
}
// 计算点云的凹壳
void calcConcaveHull(const pcl::PointCloud<PointT>::Ptr& cloud, pcl::PointCloud<PointT>::Ptr& hull, float alpha = 0.15) {
	// 创建 ConcaveHull 对象
	pcl::ConcaveHull<PointT> concave_hull;
	concave_hull.setInputCloud(cloud);
	concave_hull.setAlpha(alpha);
	// 计算凹壳
	concave_hull.reconstruct(*hull);
}
// 计算点云的凸包
void computeConvexHull(const pcl::PointCloud<PointT>::Ptr& cloud, pcl::PointCloud<PointT>::Ptr& hull) {
	pcl::ConvexHull<PointT> convex_hull;
	convex_hull.setInputCloud(cloud);
	convex_hull.setDimension(3); // 三维或二维
	convex_hull.reconstruct(*hull);
}
#include <pcl/kdtree/kdtree_flann.h>

// 计算待修补的空洞点  第三步
void computeHoles(const pcl::PointCloud<PointT>::Ptr& solid_grid,
	const pcl::PointCloud<PointT>::Ptr& grid_bottom,
	pcl::PointCloud<PointT>::Ptr& holes) {
	// 创建 KdTree 对象
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(grid_bottom);

	// 待修补的空洞点 （挖方底面）
	holes->clear();

	// 遍历实心网格点云
	for (const auto& point : solid_grid->points) {
		// 搜索最近邻点
		std::vector<int> indices;
		std::vector<float> distances;
		if (kdtree.nearestKSearch(point, 1, indices, distances) == 0) {
			// 如果没有找到最近邻点，则将当前点视为带修补的空洞点
			holes->push_back(point);
		}
	}
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
/// <summary>
/// 隐藏所有点云
/// </summary>
/// <param name="viewer"></param>
void hideAllPointClouds(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, "cloud"); // 隐藏初始点云
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, "ground"); // 隐藏地面点云
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, "offGround"); // 隐藏非地面点云
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, "cloud_bottom"); // 隐藏挖方底面点云
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, "plane_bottom"); // 隐藏挖方底平面
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, "top_voxelized"); 
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, "top_standard"); 
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, "grid_bottom"); 
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, "grid_bottom_repair"); 
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, "grid_top_repair"); 
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, "grid_bottom_repairX"); 
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, "grid_bottom_repairY");
}
/// <summary>
/// 键盘事件
/// </summary>
/// <param name="event"></param>
/// <param name="nothing"></param>
void keyboard_event_occurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
	{
		hideAllPointClouds(viewer);
		++step_count;
	}
}
#pragma endregion

#pragma region vtk相关
/// <summary>
/// 构建vtkIdList （存储一系列整数的列表）
/// </summary>
/// <param name="ids"></param>
/// <returns></returns>
static vtkIdList* mkVtkIdList(std::initializer_list<int> ids) {
	vtkIdList* vil = vtkIdList::New();
	for (auto id : ids) {
		vil->InsertNextId(id);
	}
	return vil;
}
/// <summary>
/// vtk通用show方法
/// </summary>
/// <param name="cube"></param>
static void myShow(vtkPolyData* cube, float scalar_min = 0, float scalar_max = 7) {
	vtkPolyDataMapper* cubeMapper = vtkPolyDataMapper::New();
	cubeMapper->SetInputData(cube);

	cubeMapper->SetScalarRange(scalar_min, scalar_max);

	vtkActor* cubeActor = vtkActor::New();
	cubeActor->SetMapper(cubeMapper);
	cubeActor->GetProperty()->SetColor(1.0, 0.0, 0.0);

	vtkCamera* camera = vtkCamera::New();
	camera->SetPosition(1, 1, 1);
	camera->SetFocalPoint(0, 0, 0);

	vtkRenderer* renderer = vtkRenderer::New();
	vtkAxesActor* axes = vtkAxesActor::New();
	renderer->AddActor(axes);

	vtkRenderWindow* renWin = vtkRenderWindow::New();
	renWin->AddRenderer(renderer);

	vtkRenderWindowInteractor* iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	renderer->AddActor(cubeActor);
	renderer->SetActiveCamera(camera);
	renderer->ResetCamera();
	renderer->SetBackground(0, 0, 0);

	renWin->SetSize(300, 300);
	renWin->Render();
	iren->Start();

	cubeMapper->Delete();
	cubeActor->Delete();
	camera->Delete();
	renderer->Delete();
	renWin->Delete();
	iren->Delete();
}
/// <summary>
/// 测试用 随机生成100万个相邻的立方体
/// </summary>
static void createRandomHugeCubes() {
	// 创建相应的 vtkPolyData 对象以及相关的数据容器，如 vtkPoints、vtkCellArray 和 vtkFloatArray。
	vtkPolyData* polyData = vtkPolyData::New();
	vtkPoints* points = vtkPoints::New();
	vtkCellArray* polys = vtkCellArray::New();
	vtkFloatArray* scalars = vtkFloatArray::New();

	// 定义基准立方体的顶点和顶点索引
	std::vector<std::tuple<std::vector<std::tuple<double, double, double>>, std::vector<std::tuple<int, int, int, int>>, double>> cubes;
	std::vector<std::tuple<double, double, double>> base_vertex = {
		{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0},
		{0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}, {1.0, 1.0, 1.0}, {0.0, 1.0, 1.0}
	};
	std::vector<std::tuple<int, int, int, int>> base_indices = {
		{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4},
		{1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}
	};
	// 定义立方体数量、边长和间距
	int num_cubes = 10 * 10000;
	double offset = 1.2;
	int side_length = std::ceil(std::sqrt(num_cubes));
	// 生成立方体群cubes
	for (int i = 0; i < num_cubes; i++) {
		double x_offset = (i % side_length) * offset;
		double y_offset = (i / side_length) * offset;

		std::vector<std::tuple<double, double, double>> vertex;
		std::vector<std::tuple<int, int, int, int>> indices;
		double height = 2 + static_cast<double>(rand()) / RAND_MAX * (20 - 2);

		for (auto point : base_vertex) {
			vertex.push_back({ std::get<0>(point) + x_offset, std::get<1>(point) + y_offset, std::get<2>(point) });
		}

		for (auto plane : base_indices) {
			indices.push_back({ std::get<0>(plane) + 8 * i, std::get<1>(plane) + 8 * i, std::get<2>(plane) + 8 * i, std::get<3>(plane) + 8 * i });
		}

		cubes.push_back({ vertex, indices, height });
	}
	// 构建 polyData
	for (int i = 0; i < cubes.size(); i++) {
		std::vector<std::tuple<double, double, double>> vertex;
		std::vector<std::tuple<int, int, int, int>> indices;
		double height;
		std::tie(vertex, indices, height) = cubes[i];
		for (auto point : vertex) {
			points->InsertNextPoint(std::get<0>(point), std::get<1>(point), std::get<2>(point));
		}
		for (auto plane : indices) {
			polys->InsertNextCell(mkVtkIdList({ std::get<0>(plane), std::get<1>(plane), std::get<2>(plane), std::get<3>(plane) }));
		}
		for (int j = 0; j < 8; j++) {
			scalars->InsertNextValue(height); // 添加标量值
		}
	}
	// 将数据设置到 vtkPolyData 中
	polyData->SetPoints(points);
	points->Delete();

	polyData->SetPolys(polys);
	polys->Delete();

	polyData->GetPointData()->SetScalars(scalars);
	scalars->Delete();
	// 调用 myShow 函数来显示 polyData。
	myShow(polyData);
	polyData->Delete();
}
/// <summary>
/// 创建立方体群，并打开vtk窗口显示
/// </summary>
/// <param name="cloud_top"></param>
/// <param name="cloud_bottom"></param>
/// <param name="side_length"></param>
/// <returns></returns>
static void createCubes(const pcl::PointCloud<PointT>::Ptr cloud_top,const pcl::PointCloud<PointT>::Ptr cloud_bottom, const pcl::ModelCoefficients::Ptr plane_coefficients, float side_length) {
	// cloud_top 和 cloud_bottom 大小相同
	int n = cloud_top->size();
	int m = cloud_bottom->size();
	// 如果 n 和 m 不相同，打印信息并返回 nullptr
	if (n != m) {
		std::cout << "Error: cloud_top and cloud_bottom sizes are not equal!" << std::endl;
		return;
	}
	// 创建智能指针vtkSmartPointer
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> polys = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkFloatArray> scalars = vtkSmartPointer<vtkFloatArray>::New();
	scalars->SetName("Scalars");
	double half_side_length = side_length / 2.0;
	int cube_vertex_num = 8; // 立方体顶点数为8
	//double max_height = 0.0; // 初始化最大值为0
	//double min_height = 0.0; // 初始化最小值为0
	for (int i = 0; i < n; i++) {
		// 获取当前立方体的顶面和底面中心点
		PointT pt_top = cloud_top->points[i];
		PointT pt_bottom = cloud_bottom->points[i];
		// 计算立方体的高度 有正负
		float cube_height =  calcPointToPlaneDistance(pt_top, plane_coefficients);
		// 添加顶点坐标到 vtkPoints
		std::vector<std::tuple<double, double, double>> vertex;
		vertex.push_back(std::make_tuple(pt_top.x - half_side_length, pt_top.y - half_side_length, pt_top.z));
		vertex.push_back(std::make_tuple(pt_top.x + half_side_length, pt_top.y - half_side_length, pt_top.z));
		vertex.push_back(std::make_tuple(pt_top.x + half_side_length, pt_top.y + half_side_length, pt_top.z));
		vertex.push_back(std::make_tuple(pt_top.x - half_side_length, pt_top.y + half_side_length, pt_top.z));
		vertex.push_back(std::make_tuple(pt_bottom.x - half_side_length, pt_bottom.y - half_side_length, pt_bottom.z));
		vertex.push_back(std::make_tuple(pt_bottom.x + half_side_length, pt_bottom.y - half_side_length, pt_bottom.z));
		vertex.push_back(std::make_tuple(pt_bottom.x + half_side_length, pt_bottom.y + half_side_length, pt_bottom.z));
		vertex.push_back(std::make_tuple(pt_bottom.x - half_side_length, pt_bottom.y + half_side_length, pt_bottom.z));
		for (auto v : vertex) {
			points->InsertNextPoint(std::get<0>(v), std::get<1>(v), std::get<2>(v));
		}
		// 添加顶点索引到 vtkCellArray
		int offset = cube_vertex_num * i;
		polys->InsertNextCell(mkVtkIdList({ 0 + offset, 1 + offset, 2 + offset, 3 + offset }));
		polys->InsertNextCell(mkVtkIdList({ 4 + offset, 5 + offset, 6 + offset, 7 + offset }));
		polys->InsertNextCell(mkVtkIdList({ 0 + offset, 1 + offset, 5 + offset, 4 + offset }));
		polys->InsertNextCell(mkVtkIdList({ 1 + offset, 2 + offset, 6 + offset, 5 + offset }));
		polys->InsertNextCell(mkVtkIdList({ 2 + offset, 3 + offset, 7 + offset, 6 + offset }));
		polys->InsertNextCell(mkVtkIdList({ 3 + offset, 0 + offset, 4 + offset, 7 + offset }));
		// 添加标量值
		for (int j = 0; j < cube_vertex_num; j++) {
			//float scalarValue;
			//if (cube_height > 0) {
			//	scalarValue = (cube_height / 7) * 180; // 根据高度计算标量值（着色映射）
			//}
			//else {
			//	scalarValue = 180 + (cube_height / 7) * 120; // 根据高度计算标量值（着色映射）
			//}
			scalars->InsertNextValue(cube_height);
		}
	}
	// 添加数据到 vtkPolyData
	polyData->SetPoints(points);
	polyData->SetPolys(polys);
	polyData->GetPointData()->SetScalars(scalars);
	// 调用 myShow 函数来显示 polyData。
	myShow(polyData);
	return;
}
/// <summary>
/// 创建像素阵列，并打开vtk窗口显示
/// </summary>
/// <param name="cloud_top"></param>
/// <param name="cloud_bottom"></param>
/// <param name="side_length"></param>
/// <returns></returns>
static void createRectangle(pcl::PointCloud<PointT>::Ptr cloud, double side_length) {
	// 创建智能指针vtkSmartPointer
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> polys = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkFloatArray> scalars = vtkSmartPointer<vtkFloatArray>::New();
	double half_side_length = side_length / 2.0;
	int cube_vertex_num = 4; // 矩形的顶点数为4
	for (int i = 0; i < cloud->size();  i++) {
		// 获取当前像素的中心点
		PointT center_point = cloud->points[i];
		// 计算立方体的顶点坐标
		double x_center = center_point.x;
		double y_center = center_point.y;
		double z_center = center_point.z;
		// 添加顶点坐标到 vtkPoints
		std::vector<std::tuple<double, double, double>> vertex;
		vertex.push_back(std::make_tuple(x_center - half_side_length, y_center - half_side_length, z_center));
		vertex.push_back(std::make_tuple(x_center + half_side_length, y_center - half_side_length, z_center));
		vertex.push_back(std::make_tuple(x_center + half_side_length, y_center + half_side_length, z_center));
		vertex.push_back(std::make_tuple(x_center - half_side_length, y_center + half_side_length, z_center));
		for (auto v : vertex) {
			points->InsertNextPoint(std::get<0>(v), std::get<1>(v), std::get<2>(v));
		}
		// 添加顶点索引到 vtkCellArray
		int offset = cube_vertex_num * i;
		polys->InsertNextCell(mkVtkIdList({ 0 + offset, 1 + offset, 2 + offset, 3 + offset }));
	}
	// 添加数据到 vtkPolyData
	polyData->SetPoints(points);
	polyData->SetPolys(polys);
	// 调用 myShow 函数来显示 polyData。
	myShow(polyData);
	return;
}



#pragma endregion

#pragma region 详细步骤
/// <summary>
/// CSF地面分割
/// </summary>
/// <param name="pclCloud">pcl原点云</param>
/// <param name="segCloud">pcl地面点云</param>
void csf_ground_segmentation(const pcl::PointCloud<PointT>::Ptr pclCloud, const pcl::PointCloud<PointT>::Ptr cloud_ground, const pcl::PointCloud<PointT>::Ptr cloud_offground)
{
	csf::PointCloud csfCloud;
	std::vector<int> groundIndexes, offGroundIndexes;
	pcl2csf(*pclCloud, csfCloud);
	// 定义一个 CSF 类型的变量 csf
	CSF csf;
	// 从文件中读取点云数据并存储到 csf 对象中
	csf.setPointCloud(csfCloud);
	// 设置 csf 对象的参数
	csf.params.bSloopSmooth = true;
	csf.params.class_threshold = 0.5;
	csf.params.cloth_resolution = 0.5; // 布料网格的分辨率
	csf.params.interations = 500;
	csf.params.rigidness = 1; // 布料刚性
	csf.params.time_step = 0.65;
	// 调用 csf 对象的 do_filtering 方法进行滤波，并将结果存储到 groundIndexes 和 offGroundIndexes 中
	csf.do_filtering(groundIndexes, offGroundIndexes);
	// 将地面点和非地面点分别存储到对应的点云对象中
	for (const int& i : groundIndexes)
	{
		PointT pclPoint;
		pclPoint.x = csfCloud[i].x;
		pclPoint.y = csfCloud[i].y; // 将 csf 点云的 z 坐标赋值给 pcl 点云的 y 坐标
		pclPoint.z = csfCloud[i].z; // 将 csf 点云的 y 坐标取负并赋值给 pcl 点云的 z 坐标
		cloud_ground->points.push_back(pclPoint);
	}
	for (const int& i : offGroundIndexes)
	{
		PointT pclPoint;
		pclPoint.x = csfCloud[i].x;
		pclPoint.y = csfCloud[i].y; // 将 csf 点云的 z 坐标赋值给 pcl 点云的 y 坐标
		pclPoint.z = csfCloud[i].z; // 将 csf 点云的 y 坐标取负并赋值给 pcl 点云的 z 坐标
		cloud_offground->points.push_back(pclPoint);
	}
	pcl::io::savePCDFileBinary("../file/ground.pcd", *cloud_ground);
	//save2txt("../file/ground.txt", cloud_ground);

}


/// <summary>
/// 计算体积
/// </summary>
/// <param name="cloud"></param>
/// <param name="sliceThickness"></param>
/// <returns></returns>
float calculateVolume(const typename pcl::PointCloud<PointT>::Ptr& cloud, float sliceThickness)
{
	// 计算点云的z轴范围
	float minZ = std::numeric_limits<float>::max();
	float maxZ = std::numeric_limits<float>::min();
	for (const auto& point : cloud->points)
	{
		minZ = std::min(minZ, point.z);
		maxZ = std::max(maxZ, point.z);
	}

	// 计算切片数量
	int sliceCount = static_cast<int>((maxZ - minZ) / sliceThickness) + 1;

	// 对每个切片进行体积计算
	float totalVolume = 0;
	for (int i = 0; i < sliceCount; i++)
	{
		float sliceMinZ = minZ + i * sliceThickness;
		float sliceMaxZ = sliceMinZ + sliceThickness;

		// 提取当前切片中的点
		typename pcl::PointCloud<PointT>::Ptr sliceCloud(new pcl::PointCloud<PointT>());
		for (const auto& point : cloud->points)
		{
			if (point.z >= sliceMinZ && point.z < sliceMaxZ)
			{
				sliceCloud->points.push_back(point);
			}
		}

		// 计算当前切片的体积
		typename pcl::PointCloud<PointT>::Ptr hullCloud(new pcl::PointCloud<PointT>());
		float sliceVolume = 0;
		if (!sliceCloud->points.empty())
		{
			// 计算当前切片中点的凸包

			pcl::ConvexHull<PointT> hull;
			hull.setInputCloud(sliceCloud);
			hull.setDimension(2);
			hull.reconstruct(*hullCloud);

			// 计算凸包的面积
			double hullArea = hull.getDimension() == 2 ? hull.getTotalArea() : 0;

			// 计算当前切片的体积
			sliceVolume = static_cast<float>(hullArea) * sliceThickness;
		}

		totalVolume += sliceVolume;

		// 释放内存
		sliceCloud->points.clear();
		sliceCloud->width = 0;
		sliceCloud->height = 0;
		if (!hullCloud->points.empty())
		{
			hullCloud->points.clear();
			hullCloud->width = 0;
			hullCloud->height = 0;
		}
	}

	return totalVolume;
}
/// <summary>
/// 计算体积
/// </summary>
/// <param name="cloud"></param>
/// <param name="grid_size"></param>
/// <returns></returns>
float calcVolume(const typename pcl::PointCloud<PointT>::Ptr& cloud, float grid_size)
{
	// 计算点云的边界框
	Eigen::Vector4f min_pt, max_pt;
	pcl::getMinMax3D(*cloud, min_pt, max_pt);

	// 计算网格数量
	int grid_x = static_cast<int>((max_pt.x() - min_pt.x()) / grid_size) + 1;
	int grid_y = static_cast<int>((max_pt.y() - min_pt.y()) / grid_size) + 1;

	// 初始化网格
	std::vector<std::vector<float>> grid(grid_x, std::vector<float>(grid_y, min_pt.z()));

	// 将点云中的每个点分配到对应的网格中
	for (const auto& point : cloud->points)
	{
		int x = static_cast<int>((point.x - min_pt.x()) / grid_size);
		int y = static_cast<int>((point.y - min_pt.y()) / grid_size);
		grid[x][y] = std::max(grid[x][y], point.z);
	}

	// 计算每个网格对应单元的体积并相加求和
	float total_volume = 0;
	for (int x = 0; x < grid_x; x++)
	{
		for (int y = 0; y < grid_y; y++)
		{
			// 将网格高度的初始值设置为点云中的最小高度,并将其高度更新为单元内最高点的高度。
			total_volume += grid_size * grid_size * std::max(grid[x][y] - min_pt.z(), 0.0f);
		}
	}

	return total_volume;
}
/// <summary>
/// 输入一个平面上的三维点和该平面的法向量，输出该平面方程系数
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

void projectPointCloudToPlane(pcl::PointCloud<PointT>::Ptr& cloud, pcl::ModelCoefficients::Ptr& plane_coefficients, pcl::PointCloud<PointT>::Ptr& cloud_projected)
{
	// 创建滤波器对象
	pcl::ProjectInliers<PointT> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(plane_coefficients);
	proj.filter(*cloud_projected);
}
/// <summary>
/// 挖方顶面点云向底平面投影
/// </summary>
/// <param name="cloud_top">挖方顶面点云</param>
/// <param name="plane_coefficients">挖方底平面方程系数</param>
/// <param name="cloud_bottom">挖方底面点云</param>
void projectPointsToPlane(const typename pcl::PointCloud<PointT>::Ptr& cloud_top, const pcl::ModelCoefficients::Ptr& plane_coefficients, pcl::PointCloud<PointT>::Ptr& cloud_bottom)
{
	// 创建一个新的点云来存储投影后的点
	pcl::PointCloud<PointT>::Ptr projected_cloud(new pcl::PointCloud<PointT>);

	// 将点投影到平面上
	for (const auto& point : cloud_top->points)
	{

		PointT projected_point;
		// 计算点到平面的距离，使用向量点乘的方式计算，即点的坐标与平面法向量的乘积再加上平面方程的偏移量。如果 distance 的结果是负值，表示点在平面的反向（即位于平面背面），而正值则表示点在平面的同向（即位于平面正面）。
		float distance = point.x * plane_coefficients->values[0] + point.y * plane_coefficients->values[1] + point.z * plane_coefficients->values[2] + plane_coefficients->values[3];
		// 计算投影点的坐标，即将原始点的坐标减去投影长度与法向量的乘积。
		projected_point.x = point.x - distance * plane_coefficients->values[0];
		projected_point.y = point.y - distance * plane_coefficients->values[1];
		projected_point.z = point.z - distance * plane_coefficients->values[2];
		projected_cloud->points.push_back(projected_point);
	}

	// 将投影后的点云赋值给 cloud_bottom
	*cloud_bottom = *projected_cloud;
}
/// <summary>
/// 将cloud_top和cloud_mid中的点用线条连接起来
/// </summary>
/// <param name="cloud_top">挖方顶面点云</param>
/// <param name="cloud_bottom">挖方底面点云</param>
void fillSandWich(const typename pcl::PointCloud<PointT>::Ptr& cloud_top, pcl::PointCloud<PointT>::Ptr& cloud_bottom)
{
	// 将顶层点云和底层点云中对应的点连接起来
	for (size_t i = 0; i < cloud_top->size(); ++i)
	{
		const PointT& top_point = cloud_top->points[i];
		const PointT& bottom_point = cloud_bottom->points[i];
		viewer.addLine<PointT>(top_point, bottom_point);
	}
}



/// <summary>
/// 添加立方体，颜色渐变
/// </summary>
/// <param name="cloud_top"></param>
/// <param name="cloud_bottom"></param>
/// <param name="cube_size"></param>
/// <param name="viewer"></param>
void addCubes(const pcl::PointCloud<PointT>::Ptr cloud_top,
	const pcl::PointCloud<PointT>::Ptr cloud_bottom, float cube_size)
{
	// Find the maximum possible height of a cube
	float max_height = 0.0f;
	for (size_t i = 0; i < cloud_top->size(); ++i)
	{
		const PointT& top_point = cloud_top->points[i];
		const PointT& bottom_point = cloud_bottom->points[i];
		float cube_height = abs(top_point.z - bottom_point.z);
		if (cube_height > max_height)
			max_height = cube_height;
	}

	// 遍历所有点
	for (size_t i = 0; i < cloud_top->size(); ++i)
	{
		// 获取当前点的坐标
		const PointT& top_point = cloud_top->points[i];
		const PointT& bottom_point = cloud_bottom->points[i];

		// 计算立方体的中心点
		pcl::PointXYZ cube_center((top_point.x + bottom_point.x) / 2.0f,
			(top_point.y + bottom_point.y) / 2.0f,
			(top_point.z + bottom_point.z) / 2.0f);

		// 设置立方体的尺寸
		float cube_length = cube_size;
		float cube_width = cube_size;
		float cube_height = abs(top_point.z - bottom_point.z);

		// 为每个立方体生成一个唯一的 ID
		std::stringstream cube_id;
		cube_id << "cube_" << i;

		// 计算颜色
		float normalized_height = cube_height / max_height;
		float hue = normalized_height ; // 范围：-1~1


		float r, g, b;
		hsv2rgb(hue, 1.0, 1.0, &r, &g, &b);	// HSV模型转成RGB三原色

		// 添加立方体到可视化对象中
		viewer.addCube(static_cast<float>(cube_center.x - cube_length / 2.0f), static_cast<float>(cube_center.x + cube_length / 2.0f),
			static_cast<float>(cube_center.y - cube_width / 2.0f), static_cast<float>(cube_center.y + cube_width / 2.0f),
			static_cast<float>(cube_center.z - cube_height / 2.0f), static_cast<float>(cube_center.z + cube_height / 2.0f),
			r, g, b, cube_id.str());
	}
}


/// <summary>
/// 添加立方体 精简版
/// </summary>
/// <param name="cloud_top"></param>
/// <param name="cloud_bottom"></param>
/// <param name="cube_size"></param>
void addCube_lite(const pcl::PointCloud<PointT>::Ptr cloud_top,
	const pcl::PointCloud<PointT>::Ptr cloud_bottom, double cube_size)
{
	// 遍历所有顶点
	for (size_t i = 0; i < cloud_top->size(); ++i)
	{
		// 获取当前顶面和底面点的坐标
		const PointT& top_point = cloud_top->points[i];
		const PointT& bottom_point = cloud_bottom->points[i];
		// 添加立方体到可视化对象中
		viewer.addCube(top_point.x - cube_size / 2.0f, top_point.x + cube_size / 2.0f,
			top_point.y - cube_size / 2.0f, top_point.y + cube_size / 2.0f,
			bottom_point.z, top_point.z,
			0.0f, 1.0f, 0.0f);
	}
}

/// <summary>
/// 初始版本
/// </summary>
/// <param name="cloud_top"></param>
/// <param name="cloud_bottom"></param>
/// <param name="cube_size"></param>
/// <param name="viewer"></param>
void addCubes_origin(const pcl::PointCloud<PointT>::Ptr cloud_top,
	const pcl::PointCloud<PointT>::Ptr cloud_bottom, float cube_size)
{
	// 遍历所有点
	for (size_t i = 0; i < cloud_top->size(); ++i)
	{
		// 获取当前点的坐标
		const PointT& top_point = cloud_top->points[i];
		const PointT& bottom_point = cloud_bottom->points[i];

		// 计算立方体的中心点
		pcl::PointXYZ cube_center((top_point.x + bottom_point.x) / 2.0f,
			(top_point.y + bottom_point.y) / 2.0f,
			(top_point.z + bottom_point.z) / 2.0f);

		// 设置立方体的尺寸
		float cube_length = cube_size;
		float cube_width = cube_size;
		float cube_height = abs(top_point.z - bottom_point.z);

		// 为每个立方体生成一个唯一的 ID
		std::stringstream cube_id;
		cube_id << "cube_" << i;

		// 添加立方体到可视化对象中
		viewer.addCube(static_cast<float>(cube_center.x - cube_length / 2.0f), static_cast<float>(cube_center.x + cube_length / 2.0f),
			static_cast<float>(cube_center.y - cube_width / 2.0f), static_cast<float>(cube_center.y + cube_width / 2.0f),
			static_cast<float>(cube_center.z - cube_height / 2.0f), static_cast<float>(cube_center.z + cube_height / 2.0f),
			1.0f, 1.0f, 0.0f, cube_id.str());
		// 设置立方体的渲染属性
		//viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, cube_id.str());
	}
}
/// <summary>
/// 体素化滤波
/// </summary>
/// <param name="input_cloud">输入点云</param>
/// <param name="output_cloud">输出点云</param>
/// <param name="leaf_size">单元网格大小</param>
void filterVoxelGrid(const typename pcl::PointCloud<PointT>::Ptr& input_cloud,
	pcl::PointCloud<PointT>::Ptr& output_cloud, float leaf_size)
{
	pcl::VoxelGrid<PointT> voxel_grid;
	voxel_grid.setInputCloud(input_cloud);
	//voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size); 
	voxel_grid.setLeafSize(leaf_size, leaf_size, std::numeric_limits<float>::max()); // z轴禁用下采样
	voxel_grid.filter(*output_cloud);
}
/// <summary>
/// 标准网格化体素点云
/// </summary>
/// <param name="input_cloud">输入点云</param>
/// <param name="output_cloud">输出点云</param>
/// <param name="leaf_size">单元网格大小</param>
void standardVoxelGrid(
	const typename pcl::PointCloud<PointT>::Ptr& input_cloud,
	pcl::PointCloud<PointT>::Ptr& output_cloud, float leaf_size) {
	// 获取输入点云的边界框
	PointT min_pt, max_pt;
	pcl::getMinMax3D(*input_cloud, min_pt, max_pt);

	// 计算新网格的大小
	int num_x = std::ceil((max_pt.x - min_pt.x) / leaf_size);
	int num_y = std::ceil((max_pt.y - min_pt.y) / leaf_size);
	int num_z = std::ceil((max_pt.z - min_pt.z) / leaf_size);

	// 创建一个新的点云来存储新网格
	pcl::PointCloud<PointT>::Ptr grid_cloud(new pcl::PointCloud<PointT>); 

	// 使用嵌套循环在三维空间中添加点，使它们沿x、y和z轴均匀分布
	for (int i = 0; i < num_x; ++i) {
		for (int j = 0; j < num_y; ++j) {
			for (int k = 0; k < num_z; ++k) {
				PointT point;
				point.x = min_pt.x + i * leaf_size;
				point.y = min_pt.y + j * leaf_size;
				point.z = min_pt.z + k * leaf_size;
				grid_cloud->push_back(point);
			}
		}
	}

	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	// 进行半径搜索
	float search_radius = leaf_size/2; // 设置搜索半径
	std::vector<int> nearest_indices;
	std::vector<float> nearest_distances;
	kdtree->setInputCloud(input_cloud); // 设置输入点云数据
	kdtree->setSortedResults(false); // 设置是否对搜索结果进行排序
	size_t num_points = grid_cloud->size();
	size_t ten_percent = num_points / 10;
	size_t num_points_with_neighbors = 0;

	for (size_t i = 0; i < num_points; ++i) {
		const auto& point = (*grid_cloud)[i];
		if (kdtree->radiusSearch(point, search_radius, nearest_indices, nearest_distances) > 0) {
			output_cloud->push_back(point);
			num_points_with_neighbors++;
			// 打印进度
			if (i % ten_percent == 0) {
				float progress = static_cast<float>(i) / static_cast<float>(num_points) * 100.0f;
				// 获取当前时间
				auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::tm tm_struct = *std::localtime(&now);
				std::cout << "半径搜索进度: " << progress << "%" << std::endl;
				std::cout << ", 当前时间: " << std::put_time(&tm_struct, "%H:%M:%S") << std::endl;
			}
		}
	}
	std::cout << "找到的近邻点的数量: " << num_points_with_neighbors << "/" << num_points << std::endl;
}
/// <summary>
/// 计算立方体群的体积，只打印结果不输出
/// </summary>
/// <param name="grid_bottom">底面网格点云</param>
/// <param name="grid_top">顶面网格点云</param>
/// <param name="leaf_size">分辨率</param>
void calcCubeClusterVolume(const pcl::PointCloud<PointT>::Ptr& grid_bottom,
	const pcl::PointCloud<PointT>::Ptr& grid_top,
	float leaf_size) 
	{
	float height_sum = 0.0;
	float positive_height_sum = 0.0;
	float negative_height_sum = 0.0;
	float positive_point_count = 0.0;
	float negative_point_count = 0.0;
	int total_point_count = grid_bottom->size();

	// 计算高度差累积和区分正方向和负方向的高度差累积
	for (size_t i = 0; i < total_point_count; ++i) {
		const PointT& point_top = grid_top->at(i);
		const PointT& point_bottom = grid_bottom->at(i);
		float height = calcPointToPlaneDistance(point_top, plane_coefficients);
		height_sum += std::abs(height);
		if (height > 0)
		{
			positive_height_sum += height;
			positive_point_count++;
		}
		else
		{
			negative_height_sum += height;
			negative_point_count++;
		}
	}
	float positive_volume = leaf_size * leaf_size * positive_height_sum; //挖方的体积
	float negative_volume = leaf_size * leaf_size * std::abs(negative_height_sum); //填方的体积
	float total_volume = positive_volume + negative_volume; //挖方加上填方的体积
	float diff_volume = positive_volume - negative_volume; //挖方减去填方的体积
	float positive_area = leaf_size * leaf_size * positive_point_count; //平面面积（正部分）
	float negative_area = leaf_size * leaf_size * negative_point_count; //平面面积（负部分）
	float total_area = leaf_size * leaf_size * total_point_count; //平面面积（全部）
	// 打印
	std::cout << "\r\n挖方与填方" << std::endl;;
	std::cout << "正的体积[挖方]：" << positive_volume << std::endl;
	std::cout << "负的体积[填方]：" << negative_volume << std::endl;
	std::cout << "挖方减去填方：" << diff_volume << std::endl;
	std::cout << "挖方加上填方：" << total_volume << std::endl;
	std::cout << "\r\n面积" << std::endl;;
	std::cout << "平面面积（正部分）：" << positive_area << std::endl;
	std::cout << "平面面积（负部分）：" << negative_area << std::endl;
	std::cout << "全部的平面面积：" << total_area << std::endl;
}
/// <summary>
/// 计算立方体群的体积，并输出结果
/// </summary>
/// <param name="grid_bottom">底面网格点云</param>
/// <param name="grid_top">顶面网格点云</param>
/// <param name="leaf_size">分辨率</param>
/// <param name="positive_volume">正的体积（挖方）</param>
/// <param name="negative_volume">负的体积（填方）</param>
/// <param name="total_volume">挖方加上填方</param>
/// <param name="diff_volume">挖方减去填方</param>
/// <param name="positive_area">平面面积（正部分）</param>
/// <param name="negative_area">平面面积（负部分）</param>
/// <param name="total_area">全部的平面面积</param>
void calcCubeClusterVolume(const pcl::PointCloud<PointT>::Ptr& grid_bottom,
	const pcl::PointCloud<PointT>::Ptr& grid_top,
	float leaf_size,
	float& positive_volume,
	float& negative_volume,
	float& total_volume,
	float& diff_volume,
	float& positive_area,
	float& negative_area,
	float& total_area) {
	float height_sum = 0.0;
	float positive_height_sum = 0.0;
	float negative_height_sum = 0.0;
	float positive_point_count = 0.0;
	float negative_point_count = 0.0;
	int total_point_count = grid_bottom->size();

	// 计算高度差累积，区分正方向和负方向的高度差累积
	for (size_t i = 0; i < total_point_count; ++i) {
		const PointT& point_top = grid_top->at(i);
		const PointT& point_bottom = grid_bottom->at(i);
		float height = point_top.z - point_bottom.z;
		height_sum += std::abs(height);
		if (height > 0)
		{
			positive_height_sum += height;
			positive_point_count++;
		}
		else
		{
			negative_height_sum += height;
			negative_point_count++;
		}
	}
	positive_volume = leaf_size * leaf_size * positive_height_sum; //挖方的体积
	negative_volume = leaf_size * leaf_size * negative_height_sum; //填方的体积
	total_volume = positive_volume + negative_volume; //挖方加上填方的体积
	diff_volume = positive_volume - negative_volume; //挖方减去填方的体积
	positive_area = leaf_size * leaf_size * positive_point_count; //平面面积（正部分）
	negative_area = leaf_size * leaf_size * negative_point_count; //平面面积（负部分）
	total_area = leaf_size * leaf_size * total_point_count; //平面面积（全部）
	// 打印
	std::cout << "\r\n挖方与填方";
	std::cout << "正的体积[挖方]：" << positive_volume << std::endl;
	std::cout << "负的体积[填方]：" << negative_volume << std::endl;
	std::cout << "挖方减去填方：" << diff_volume << std::endl;
	std::cout << "挖方加上填方：" << total_volume << std::endl;
	std::cout << "\r\n面积";
	std::cout << "平面面积（正部分）：" << positive_area << std::endl;
	std::cout << "平面面积（负部分）：" << negative_area << std::endl;
	std::cout << "全部的平面面积：" << total_area << std::endl;
}
/// <summary>
/// 挖方底面二维网格化
/// </summary>
/// <param name="cloud_bottom">挖方底面点云</param>
/// <param name="plane_coefficients">底平面方程系数</param>
/// <param name="leaf_size">分辨率</param>
/// <param name="grid_bottom">输出：网格底面</param>
void createGridBottom(
	const typename pcl::PointCloud<PointT>::Ptr& cloud_bottom, 
	const pcl::ModelCoefficients::Ptr& plane_coefficients,
	float leaf_size,
	pcl::PointCloud<PointT>::Ptr& grid_bottom) 
{
	// 获取挖方底面点云的边界框
	PointT min_pt, max_pt;
	pcl::getMinMax3D(*cloud_bottom, min_pt, max_pt); 

	// 计算标准矩形网格的大小
	int num_x = std::ceil((max_pt.x - min_pt.x) / leaf_size);
	int num_y = std::ceil((max_pt.y - min_pt.y) / leaf_size);
	// 标准矩形网格 二维
	pcl::PointCloud<PointT>::Ptr grid_cloud(new pcl::PointCloud<PointT>);

	// 使用嵌套循环在二维空间中添加点，使它们沿x、y、z轴均匀分布
	for (int i = 0; i < num_x; ++i) {
		for (int j = 0; j < num_y; ++j) {
			PointT point;
			point.x = min_pt.x + i * leaf_size;
			point.y = min_pt.y + j * leaf_size;
			// 根据平面方程 Ax + By + Cz + D = 0 得 z = (-A * x - B * y - D) / C
			point.z = (-plane_coefficients->values[0] * point.x - plane_coefficients->values[1] * point.y - plane_coefficients->values[3]) / plane_coefficients->values[2];
			grid_cloud->push_back(point);
		}
	}
	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	// 进行半径搜索
	float search_radius = leaf_size/2; // 设置搜索半径
	std::vector<int> nearest_indices;
	std::vector<float> nearest_distances;
	kdtree->setInputCloud(cloud_bottom); // 设置输入点云数据
	kdtree->setSortedResults(false); // 设置是否对搜索结果进行排序
	size_t num_points = grid_cloud->size();
	size_t progress_interval = num_points / 4; // 打印进度间隔
	size_t counter = 0;
	// 17s
	for (size_t i = 0; i < num_points; ++i) {
		const auto& point = (*grid_cloud)[i];
		if (kdtree->radiusSearch(point, search_radius, nearest_indices, nearest_distances) > 0) {
			// 网格点的底
			PointT bottom_point(point.x, point.y, point.z); 
			grid_bottom->push_back(bottom_point);
			counter++;
			// 打印进度
			//if (counter >= progress_interval) {
			//	int percent = static_cast<int>(((i + 1.0) / num_points) * 100.0);
			//	// 获取当前时间
			//	auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			//	std::tm tm_struct = *std::localtime(&now);
			//	std::cout << "createGridBottom...半径搜索进度: " << percent << "%" << std::endl;
			//	std::cout << "当前时间: " << std::put_time(&tm_struct, "%H:%M:%S") << std::endl;
			//	counter = 0; // 重置计数器
			//}
		}
	}
	std::cout << "\r\n网格大小: " << num_x << "列 × " << num_y << "行 " << std::endl;
	std::cout << "分辨率: " << leaf_size << "m" << std::endl;
	std::cout << "X-范围: " << min_pt.x << "m 到 " << max_pt.x << "m" << std::endl;
	std::cout << "Y-范围: " << min_pt.y << "m 到 " << max_pt.y << "m" << std::endl;
	std::cout << "Z-范围: " << min_pt.z << "m 到 " << max_pt.z << "m" << std::endl;
	// 计算体积
	//calcCubeClusterVolume(grid_bottom, grid_top,leaf_size);
}
/// <summary>
/// 挖方底面二维网格化+挖方顶面网格化
/// </summary>
/// <param name="cloud_bottom">挖方底面</param>
/// <param name="cloud_top">挖方顶面</param>
/// <param name="grid_bottom">网格底面</param>
/// <param name="grid_top">挖方顶面</param>
/// <param name="leaf_size">分辨率</param>
void standardVoxelGrid2d(
	const typename pcl::PointCloud<PointT>::Ptr& cloud_bottom,
	const typename pcl::PointCloud<PointT>::Ptr& cloud_top,
	pcl::PointCloud<PointT>::Ptr& grid_bottom,
	pcl::PointCloud<PointT>::Ptr& grid_top,
	float leaf_size) {
	// 获取挖方底面点云的边界框
	PointT min_pt, max_pt;
	pcl::getMinMax3D(*cloud_bottom, min_pt, max_pt);

	// 计算标准矩形网格的大小
	int num_x = std::ceil((max_pt.x - min_pt.x) / leaf_size);
	int num_y = std::ceil((max_pt.y - min_pt.y) / leaf_size);

	// 标准矩形网格 二维
	pcl::PointCloud<PointT>::Ptr grid_cloud(new pcl::PointCloud<PointT>);

	// 使用嵌套循环在二维空间中添加点，使它们沿x、y轴均匀分布，z轴值固定
	for (int i = 0; i < num_x; ++i) {
		for (int j = 0; j < num_y; ++j) {
			PointT point;
			point.x = min_pt.x + i * leaf_size;
			point.y = min_pt.y + j * leaf_size;
			point.z = min_pt.z; // todo 应该用挖方底面点对应的z坐标
			grid_cloud->push_back(point);
		}
	}

	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	// 进行半径搜索
	float search_radius = leaf_size / 2; // 设置搜索半径
	std::vector<int> nearest_indices;
	std::vector<float> nearest_distances;
	kdtree->setInputCloud(cloud_bottom); // 设置输入点云数据
	kdtree->setSortedResults(false); // 设置是否对搜索结果进行排序
	size_t num_points = grid_cloud->size();
	size_t progress_interval = num_points / 20; // 打印进度间隔
	size_t counter = 0;

	for (size_t i = 0; i < num_points; ++i) {
		const auto& point = (*grid_cloud)[i];
		if (kdtree->radiusSearch(point, search_radius, nearest_indices, nearest_distances) > 0) {
			// 获取邻近点中z坐标绝对值最大的点的z坐标
			float max_abs_z = 0.0f; // 最大的<z坐标绝对值>
			float max_z = 0.0f; // <z坐标绝对值>最大的点
			for (size_t i = 0; i < nearest_indices.size(); i++) {
				float abs_z = std::abs((*cloud_top)[nearest_indices[i]].z);
				if (abs_z > max_abs_z) {
					max_abs_z = abs_z;
					max_z = (*cloud_top)[nearest_indices[i]].z;
				}
			}
			float top_z = max_z;
			// 网格点的底和顶
			PointT bottom_point(point.x, point.y, point.z); // 应该用挖方底面点对应的z坐标
			PointT top_point(point.x, point.y, top_z);
			if (kdtree->radiusSearch(point, search_radius, nearest_indices, nearest_distances) > 1)
			{
				std::string msg = "";
			}
			grid_bottom->push_back(bottom_point);
			grid_top->push_back(top_point);
			counter++;
			// 打印进度
			if (counter >= progress_interval) {
				int percent = static_cast<int>((i + 1) / num_points * 100.0);
				// 获取当前时间
				auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::tm tm_struct = *std::localtime(&now);
				std::cout << "半径搜索进度: " << percent << "%" << std::endl;
				std::cout << ", 当前时间: " << std::put_time(&tm_struct, "%H:%M:%S") << std::endl;
				counter = 0; // 重置计数器
			}
		}
	}
	// 计算底面网格点云的凹壳 
	//calcConcaveHull(grid_bottom, cloud_concave_hull,2.0);
	// 拟合底面网格点云的外轮廓

	// 依据底面网格点云的外轮廓生成实心平面网格点云

	// 求出实心平面网格点云中所有待修补的空洞点

	// 依据grid_top对所有空洞点进行插值修补

	//computeConvexHull(grid_bottom, cloud_concave_hull);

	std::cout << "\r\n网格大小: " << num_x << "列 × " << num_y << "行 " << std::endl;
	std::cout << "分辨率: " << leaf_size << "m" << std::endl;
	std::cout << "X-范围: " << min_pt.x << "m 到 " << max_pt.x << "m" << std::endl;
	std::cout << "Y-范围: " << min_pt.y << "m 到 " << max_pt.y << "m" << std::endl;
	std::cout << "Z-范围: " << min_pt.z << "m 到 " << max_pt.z << "m" << std::endl;
	// 计算体积
	//calcCubeClusterVolume(grid_bottom, grid_top,leaf_size);


}
/// <summary>
/// 对每一个X坐标下，取Ymin和Ymax，进行插值修补
/// </summary>
/// <param name="cloud">输入网格点云</param>
/// <param name="plane_coefficients">挖方底面平面方程系数</param>
/// <param name="leaf_size">网格分辨率</param>
/// <param name="cloud_x">输出：修补后的网格点云</param>
void repairY(const pcl::PointCloud<PointT>::Ptr& cloud, 
	const pcl::ModelCoefficients::Ptr& plane_coefficients,
	float leaf_size,
	pcl::PointCloud<PointT>::Ptr& cloud_y)
{
	// 按x坐标从小到大对点云进行排序
	auto compareXCoordinate = [](const PointT& p1, const PointT& p2)
	{
		return p1.x < p2.x;
	};
	std::sort(cloud->points.begin(), cloud->points.end(), compareXCoordinate);
	// 遍历点云
	float current_x = cloud->points[0].x;
	float current_z = cloud->points[0].z;
	float minY = cloud->points[0].y;
	float maxY = cloud->points[0].y;
	PointT point_CurMinY;	// 当前最小Y值的点
	PointT point_CurMaxY;	// 当前最大Y值的点
	for (std::size_t idx = 0; idx < cloud->points.size(); ++idx) 
	{
		const PointT& point = cloud->points[idx];
		// 当x坐标发生变化时，将当前的最小和最大y值点加入相应的容器中
		if (!(std::abs(point.x - current_x) < 0.00001))
		{
			// 根据leaf_size、minY、maxY，取等间距补点
			float interval = leaf_size;
			for (float y = minY ; y < maxY; y += interval) {
				PointT point_add;
				point_add.x = current_x;
				point_add.y = y;
				//point_add.z = current_z; // todo
				point_add.z = (-plane_coefficients->values[0] * point_add.x - plane_coefficients->values[1] * point_add.y - plane_coefficients->values[3]) / plane_coefficients->values[2];
				cloud_y->points.push_back(point_add);
			}
			current_x = point.x;
			current_z = point.z;
			minY = point.y;
			maxY = point.y;
		}
		// 更新最小和最大y值
		minY = std::min(minY, point.y);
		maxY = std::max(maxY, point.y);
		// 如果是最后一个x坐标
		if (idx == cloud->points.size() - 1)
		{
			// 对最后一个x坐标进行补点
			float interval = leaf_size;
			for (float y = minY; y < maxY; y += interval) {
				PointT point_add;
				point_add.x = current_x;
				point_add.y = y;
				//point_add.z = current_z; // todo
				point_add.z = (-plane_coefficients->values[0] * point_add.x - plane_coefficients->values[1] * point_add.y - plane_coefficients->values[3]) / plane_coefficients->values[2];
				cloud_y->points.push_back(point_add);
			}
		}
	}
}
/// <summary>
/// 对每一个Y坐标下，取Xmin和Xmax，进行插值修补
/// </summary>
/// <param name="cloud">输入网格点云</param>
/// <param name="plane_coefficients">挖方底面平面方程系数</param>
/// <param name="leaf_size">网格分辨率</param>
/// <param name="cloud_x">输出：修补后的网格点云</param>
void repairX(const pcl::PointCloud<PointT>::Ptr& cloud, 
	const pcl::ModelCoefficients::Ptr& plane_coefficients,
	float leaf_size,
	pcl::PointCloud<PointT>::Ptr& cloud_x)
{
	// 按y坐标从小到大对点云进行排序
	auto compareYCoordinate = [](const PointT& p1, const PointT& p2)
	{
		return p1.y < p2.y;
	};
	std::sort(cloud->points.begin(), cloud->points.end(), compareYCoordinate);

	// 遍历点云
	float current_y = cloud->points[0].y;
	float current_z = cloud->points[0].z;
	float minX = cloud->points[0].x;
	float maxX = cloud->points[0].x;

	for (std::size_t idx = 0; idx < cloud->points.size(); ++idx)
	{
		const PointT& point = cloud->points[idx];

		// 当y坐标发生变化时，将当前的最小和最大x值点加入相应的容器中
		if (!(std::abs(point.y - current_y) < 0.00001))
		{
			// 根据leaf_size、minX、maxX，取等间距补点
			float interval = leaf_size;
			for (float x = minX; x < maxX; x += interval) {
				PointT point_add;
				point_add.x = x;
				point_add.y = current_y;
				//point_add.z = current_z; // todo
				point_add.z = (-plane_coefficients->values[0] * point_add.x - plane_coefficients->values[1] * point_add.y - plane_coefficients->values[3]) / plane_coefficients->values[2];
				cloud_x->points.push_back(point_add);
			}
			current_y = point.y;
			current_z = point.z;
			minX = point.x;
			maxX = point.x;
		}
		// 更新最小和最大x值
		minX = std::min(minX, point.x);
		maxX = std::max(maxX, point.x);
		// 如果是最后一个y坐标
		if (idx == cloud->points.size() - 1)
		{
			// 对最后一个y坐标进行补点
			float interval = leaf_size;
			for (float x = minX; x < maxX; x += interval) {
				PointT point_add;
				point_add.x = x;
				point_add.y = current_y;
				//point_add.z = current_z; // todo
				point_add.z = (-plane_coefficients->values[0] * point_add.x - plane_coefficients->values[1] * point_add.y - plane_coefficients->values[3]) / plane_coefficients->values[2];
				cloud_x->points.push_back(point_add);
			}
		}
	}
}

/// <summary>
/// 修补挖方底面网格点云
/// </summary>
/// <param name="grid">挖方底面网格点云</param>
/// <param name="plane_coefficients">挖方底面平面方程系数</param>
/// <param name="leaf_size">分辨率</param>
/// <param name="grid_repair">输出：修补后的挖方底面网格点云</param>
void repairGridBottom(const pcl::PointCloud<PointT>::Ptr& grid, 
	const pcl::ModelCoefficients::Ptr& plane_coefficients,
	float leaf_size,
	pcl::PointCloud<PointT>::Ptr& grid_repair)
{
	repairX(grid, plane_coefficients, leaf_size, grid_bottom_repairX);
	repairY(grid, plane_coefficients, leaf_size, grid_bottom_repairY);
	// 取cloud_x和cloud_y的交集
	getIntersection_NK(grid_bottom_repairX, grid_bottom_repairY, grid_repair);
	std::cout << "grid_bottom_repairX size: " << grid_bottom_repairX->size() << std::endl;
	std::cout << "grid_bottom_repairY size: " << grid_bottom_repairY->size() << std::endl;
	std::cout << "Intersection_NK size: " << grid_repair->size() << std::endl;
}

/// <summary>
/// 生成并修补挖方顶面网格
/// </summary>
/// <param name="cloud_bottom">挖方底面</param>
/// <param name="cloud_top">挖方顶面</param>
/// <param name="grid_bottom">挖方底面网格</param>
/// <param name="grid_top">挖方顶面网格</param>
/// <param name="leaf_size">分辨率</param>
void createGridTop(
	const typename pcl::PointCloud<PointT>::Ptr& cloud_bottom,
	const typename pcl::PointCloud<PointT>::Ptr& cloud_top,
	const typename pcl::PointCloud<PointT>::Ptr& grid_bottom,
	pcl::PointCloud<PointT>::Ptr& grid_top,
	float leaf_size) {
	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	
	// 进行半径搜索
	float search_radius = leaf_size / 2; // 设置搜索半径
	int k = 10;
	std::vector<int> nearest_indices;
	std::vector<float> nearest_distances;
	kdtree->setInputCloud(cloud_bottom); // 设置输入点云数据
	kdtree->setSortedResults(false); // 设置是否对搜索结果进行排序
	size_t num_points = grid_bottom->size();
	for (size_t i = 0; i < num_points; ++i) {
		const auto& point = (*grid_bottom)[i];
		if (kdtree->radiusSearch(point, search_radius, nearest_indices, nearest_distances) > 0) {
			// 计算(*cloud_top)[nearest_indices[i]]点到平面plane_coefficients的距离（有正负）绝对值最大的distance原始值
			float max_abs_distance = 0.0f;
			float max_distance = 0.0f;
			for (size_t i = 0; i < nearest_indices.size(); i++) {
				float distance = calcPointToPlaneDistance((*cloud_top)[nearest_indices[i]], plane_coefficients);
				if (abs(distance) > max_abs_distance) {
					max_abs_distance = abs(distance);
					max_distance = distance;
				}
			}
			
			// 顶面网格的对应点
			PointT top_point = calcTopPoint(point,plane_coefficients, max_distance);
			
			grid_top->push_back(top_point);
		}
		else
		{
			kdtree->nearestKSearch(point, k, nearest_indices, nearest_distances);
			float max_abs_distance = 0.0f;
			float max_distance = 0.0f;
			for (size_t i = 0; i < nearest_indices.size(); i++) {
				float distance = calcPointToPlaneDistance((*cloud_top)[nearest_indices[i]], plane_coefficients);
				if (abs(distance) > max_abs_distance) {
					max_abs_distance = abs(distance);
					max_distance = distance;
				}
			}
			// 顶面网格的对应点
			PointT top_point = calcTopPoint(point, plane_coefficients, max_distance);
			grid_top->push_back(top_point);
		}
	}
	// 计算体积
	calcCubeClusterVolume(grid_bottom, grid_top, leaf_size);
}






#pragma endregion





int main3(int argc, char** argv)
{
	//createRandomHugeCubes(); // 随机创建立方体群并调用vtk显示
	step_count = 1;
	std::string str = "STEP";
	//初始化PCLVisualizer
	//viewer.addCoordinateSystem(1);
	//viewer.setBackgroundColor(0, 0, 255);
	viewer.addText(str + std::to_string(step_count), 10, 10, 16, 200, 200, 100, "text");

	viewer.registerKeyboardCallback(&keyboard_event_occurred, (void*)NULL);


	// 读点云
	reader.read("file/SuiDao.pcd", *cloud);
	std::cout << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

	// 显示原始点云
	//viewer.addPointCloud(cloud, "cloud");

	
	// CSF地面分割（得到挖方的顶面）
	csf_ground_segmentation(cloud, cloud_ground, cloud_offground);

	// 对挖方顶面进行修补
	cloud_top = cloud_ground;

    // 定义挖方底平面。根据用户输入的：一个三维点xyz，一个法向量（例如：(0,0,1)）
	float offset_x = -503582.61;
	float offset_y = -3407981.37;
	float offset_z = -166.27;
	PointT point; 	//输入的三维点 
	point.x = 407963.2053 + offset_x;
	point.y = 503592.5872 + offset_y;
	point.z = 160.5930 + offset_z;
	// realworks保留2位小数：-95619.40 m; -2904388.78 m; -5.68 m
	point.x = -95619.40;
	point.y = -2904388.78;
	point.z = -5.68;
	// 斜面
	point.x = -1.70;
	point.y = -4.08;
	point.z = -5.24;
	std::cout << std::fixed;  // 设置输出为固定小数位数
	std::cout.precision(5);  // 设置小数点后的位数为5位
	std::cout << "SelectedPoint: ( " << point.x << " , " << point.y << " , " << point.z << " )" << std::endl;
	//Eigen::Vector3f normal(0.0, 0.0, 1.0); 	//输入的法向量
	Eigen::Vector3f normal(0.35 ,0.22, 0.91); //输入的法向量 
    // 单位化平面法向量
	normal = normal.normalized();

	float leaf_size = 0.2; // 体素大小
	// 平面方程系数
	calcPlaneCoefficients(point, normal, plane_coefficients);
	// 投影生成挖方底面 
	projectPointCloudToPlane(cloud_top, plane_coefficients, cloud_bottom); //2.7s
	// 挖方底面二维网格化
	createGridBottom(cloud_bottom, plane_coefficients, leaf_size, grid_bottom); // 
	// 底面修补
	repairGridBottom(grid_bottom, plane_coefficients, leaf_size, grid_bottom_repair);
	// 生成顶面并修补
	createGridTop(cloud_bottom, cloud_top, grid_bottom_repair, grid_top_repair, leaf_size);
	// 创建立方体群
	createCubes(grid_top_repair, grid_bottom_repair, plane_coefficients, leaf_size*0.8);
	//createRectangle(cloud_grid_bottom, leaf_size * 1.0);

	//保存顶面和底面点云为pcd文件
	//pcl::io::savePCDFileASCII("cloud_top.pcd", *cloud_top_voxelized);
	//pcl::io::savePCDFileASCII("cloud_bottom.pcd", *cloud_bottom);

	// 设置背景颜色
	viewer.setBackgroundColor(0, 0, 0);
	// 添加坐标轴
	viewer.addCoordinateSystem(10.0, Eigen::Affine3f::Identity(), "coordinate");
	// 添加轴标签
	viewer.addText3D("x", pcl::PointXYZ(11, 0, 0), 1.0, 1.0, 1.0, 1.0, "x_label");
	viewer.addText3D("y", pcl::PointXYZ(0, 11, 0), 1.0, 1.0, 1.0, 1.0, "y_label");
	viewer.addText3D("z", pcl::PointXYZ(0, 0, 11), 1.0, 1.0, 1.0, 1.0, "z_label");
	// 添加非地面点云
	pcl::visualization::PointCloudColorHandlerCustom<PointT> nonGroundColor(cloud_offground, 204, 58, 41);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> groundColor(cloud_ground, 41, 204, 146);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> xldColor(cloud_ground, 255, 255, 0);
	viewer.addPointCloud<PointT>(cloud_offground, nonGroundColor, "offGround");
	// 添加地面点云
	viewer.addPointCloud<PointT>(cloud_ground, groundColor, "ground");
	// 添加体素化后的挖方顶面点云
	viewer.addPointCloud<PointT>(cloud_top_voxelized, groundColor, "top_voxelized");
	// 添加网格体素化后的挖方顶面点云
	viewer.addPointCloud<PointT>(cloud_top_standard, groundColor, "top_standard");
	// 添加网格体素化后的挖方底面点云
	viewer.addPointCloud<PointT>(grid_bottom, nonGroundColor, "grid_bottom");
	// 添加网格化后的挖方底面的凹壳
	viewer.addPointCloud<PointT>(cloud_concave_hull, xldColor, "cloud_concave_hull");
	// 添加挖方底平面
	viewer.addPlane(*plane_coefficients, "plane_bottom");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "plane_bottom");
	// 添加挖方底面点云
	viewer.addPointCloud<PointT>(cloud_bottom, nonGroundColor, "cloud_bottom");
	// 修补后的挖方底面网格点云
	viewer.addPointCloud<PointT>(grid_bottom_repairX, xldColor, "grid_bottom_repairX");
	viewer.addPointCloud<PointT>(grid_bottom_repairY, xldColor, "grid_bottom_repairY");
	viewer.addPointCloud<PointT>(grid_bottom_repair, xldColor, "grid_bottom_repair");
	viewer.addPointCloud<PointT>(grid_top_repair, nonGroundColor, "grid_top_repair");
	
	// 重置相机视角
	viewer.resetCamera();
	// 计算点云的质心
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);
	// 设置相机的位置和方向
	viewer.setCameraPosition(centroid[0] - 30.0, centroid[1] - 50.0, centroid[2] + 10.0,
		centroid[0], centroid[1], centroid[2], 0.0, 0.0, 1.0);
	// 设置点云显示属性，如点大小等
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_bottom");
	//隐藏所有点云
	hideAllPointClouds(viewer);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		if (step_count < 9)
		{
			viewer.updateText(str + std::to_string(step_count), 10, 10, 16, 50, 100, 200, "text");
			switch (step_count)
			{
			case 1 :
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "cloud"); //显示初始点云
				break;
			case 2: 
				//隧道顶和隧道底
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "ground"); 
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "offGround"); 
				break;
			case 3:
				//挖方顶和挖方底
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "ground"); 
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "cloud_bottom"); 
			case 4:
				//挖方底面网格
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "grid_bottom"); 
				break;
			case 5:
				//修补X
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "grid_bottom_repairX");
				break;
			case 6:
				//修补Y
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "grid_bottom_repairY");
				break;
			case 7:
				//修补后的底面网格
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "grid_bottom_repair");
				break;
			case 8:
				//修补后的顶面和底面网格
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "grid_bottom_repair");
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "grid_top_repair");
				break;
			default:
				break;
			}
		}
		else {
			break;
		}

	}

	return (0);
}

