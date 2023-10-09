#include "pch.h"

// 定义点云类型模板
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBNormal PointTN;

pcl::PointCloud<PointT>::Ptr cloud_input(new pcl::PointCloud<PointT>); //输入的隧道点云
pcl::PointCloud<PointT>::Ptr cloud_voxel(new pcl::PointCloud<PointT>); //体素点云
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>); // 法向
pcl::PointCloud<PointT>::Ptr cloud_seg(new pcl::PointCloud<PointT>); //分割后的点云
pcl::visualization::PCLVisualizer viewer("MutiViewer");
pcl::console::TicToc tt;
static float leaf_size = 0.1f; // 降采样分辨率

#pragma region 读写操作和格式转换
// 读pcd
void readPcd(const std::string& filename, pcl::PointCloud<PointT>::Ptr& cloud)
{
	std::cerr << "读点云...\n", tt.tic();//下采样
	//pcl::PCDReader reader;
	//reader.read(filename, *cloud);
	pcl::io::loadPCDFile(filename, *cloud);
	std::cerr << ">> Done: " << tt.toc() << " ms, " << cloud->points.size() << " points\n";
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

int getNearestPointInCloud(PointT search_point, pcl::PointCloud<PointT>::Ptr cloud_src)
{
	// 创建一个 kd 树搜索对象
	pcl::search::KdTree<PointT> kdtree;
	kdtree.setInputCloud(cloud_src);
	// 进行 K 近邻搜索
	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);
	kdtree.nearestKSearch(search_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
	// 返回最近邻点的索引
	return pointIdxNKNSearch[0];
}


#include <pcl/filters/crop_box.h>
// 根据包围盒裁切点云
void filterCropBox(pcl::PointCloud<PointT>::Ptr cloud, Eigen::Vector4f min_pt, Eigen::Vector4f max_pt, pcl::PointCloud<PointT>::Ptr cloudFiltered, pcl::PointIndices::Ptr indices) {
	std::cerr << "\n根据包围盒裁切点云...\n", tt.tic();
	pcl::CropBox<PointT> boxFilter;
	boxFilter.setMin(min_pt);
	boxFilter.setMax(max_pt);
	boxFilter.setInputCloud(cloud);
	boxFilter.filter(*cloudFiltered);
	boxFilter.filter(indices->indices);
	std::cerr << ">> Done: " << tt.toc() << " ms, " << indices->indices.size() << " points\n";

}

// DBSCAN 密度聚类
void getDbscanCluster(const pcl::PointCloud<PointT>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices)
{
	std::cerr << "\nDBSCAN 密度聚类...\n", tt.tic();
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);
	DBSCANKdtreeCluster<PointT> ec;
	ec.setCorePointMinPts(10); // 核心点最小邻居数
	ec.setClusterTolerance(0.9); // 邻域半径
	//ec.setMinClusterSize(100);//设置过小聚类的标准
	ec.setMinClusterSize(cloud->points.size() / 1000);//设置过小聚类的标准
	ec.setMaxClusterSize(cloud->points.size() / 1);//设置过大聚类的标准
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);
	std::cerr << ">> Done: " << tt.toc() << " ms, 提取到 " << cluster_indices.size() << " 个簇\n";
}

// 区域生长分割
void regionGrowingSegmentation(pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointIndices>& clusters)
{
	std::cerr << "\n区域生长分割...\n", tt.tic();
	// 建立搜索KD树
	pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	// 计算点云法向
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_estimator;
	//pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree); // 搜索方法为kd树走索
	normal_estimator.setInputCloud(cloud);  // 填入点云
	normal_estimator.setKSearch(30);        // 设置搜索范围
	//normal_estimator.setRadiusSearch(0.1);        // 设置搜索范围
	normal_estimator.compute(*normals);     // 将法相保存在normals
	pcl::IndicesPtr indices(new std::vector<int>);
	pcl::removeNaNFromPointCloud(*cloud, *indices); // 对点云建立索引
	pcl::RegionGrowing<PointT, pcl::Normal> reg; // 区域增长类
	reg.setMinClusterSize(cloud->points.size() / 1000);//设置过小聚类的标准
	reg.setMaxClusterSize(cloud->points.size() );//设置过大聚类的标准
	reg.setSearchMethod(tree);                    // 设置kd树搜索方法
	
	reg.setNumberOfNeighbours(30);                // 设置每次邻域搜索数(影响计算速度)
	reg.setInputCloud(cloud);                     // 设置输入点云
	reg.setIndices(indices);                      // 设置输入的索引
	reg.setInputNormals(normals);                 // 设置输入法向
	reg.setSmoothnessThreshold(6.0 / 180.0 * M_PI);      // 设置平滑度阈值（弧度） 两个相邻点之间的法向量夹角的最大值 
	reg.setCurvatureThreshold(0.1);                      // 设置曲率阈值 每个点的曲率的最大值 （达到阈值则为簇群边界点）
	reg.extract(clusters);

	std::cerr << ">> Done: " << tt.toc() << " ms, 提取到 " << clusters.size() << " 个簇\n";
}
// SAC平面分割
void getSacPlane(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr plane) {

	std::cerr << "\nSAC平面分割...\n", tt.tic();
	// 建立搜索KD树
	pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	// 计算点云法向
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_estimator;
	//pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree); // 搜索方法为kd树走索
	normal_estimator.setInputCloud(cloud);  // 填入点云
	normal_estimator.setKSearch(30);        // 设置搜索范围
	//normal_estimator.setRadiusSearch(0.1);        // 设置搜索范围
	normal_estimator.compute(*normals);     // 将法相保存在normals

	// 创建一个SACSegmentation对象，方法类型为RANSAC，并设置模型类型为圆柱
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	// 设置距离阈值
	seg.setDistanceThreshold(leaf_size);
	// 设置最大迭代次数
	//seg.setMaxIterations(100);
	// 设置概率
	//seg.setProbability(0.6);
	seg.setInputCloud(cloud); // 设置输入点云
	seg.setInputNormals(normals);  // 设置输入法向
	// 创建一个模型系数对象和一个内点索引对象
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// 调用segment方法进行分割
	seg.segment(*inliers, *coefficients);


	// 检查是否分割成功
	if (inliers->indices.size() == 0) {
		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		return;
	}

	// 分割出的形状点云



	// 从原始点云中提取分割出的形状点云
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*plane);
	std::cerr << ">> Done: " << tt.toc() << " ms, " << inliers->indices.size() << " points\n";
}


#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>//条件欧式聚类
// 自定义条件
bool customRegionGrowing(const PointTN& point_a, const PointTN& point_b, float squared_distance)//区域生长
{
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(), point_b_normal = point_b.getNormalVector3fMap();
	float cos = fabs(point_a_normal.dot(point_b_normal));
	if (cos > 0.9) // 若接近平行
		return (true); // 属于同一簇
	return (false); // 不属于同一簇
}
// 条件欧氏聚类
void conditionalEuclidean(const pcl::PointCloud<PointT>::Ptr& cloud_input, pcl::PointCloud<PointT>::Ptr cloud_out)
{
	//建立条件欧式聚类对象
	std::cerr << "Segmenting to clusters...\n", tt.tic();
	pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters), small_clusters(new pcl::IndicesClusters), large_clusters(new pcl::IndicesClusters);//创建索引
	pcl::ConditionalEuclideanClustering<PointTN> cec(true);//创建条件聚类分割对象，并进行初始化
	pcl::PointCloud<PointTN>::Ptr cloud_with_normals(new pcl::PointCloud<PointTN>);
	pcl::concatenateFields(*cloud_input, *cloud_normals, *cloud_with_normals);
	cec.setInputCloud(cloud_with_normals);//设置输入点云
	cec.setConditionFunction(&customRegionGrowing);//设置搜索函数
	cec.setClusterTolerance(0.20);//设置聚类参考点的所搜距离
	cec.setMinClusterSize(cloud_with_normals->points.size() / 1000);//设置过小聚类的标准
	cec.setMaxClusterSize(cloud_with_normals->points.size() / 5);//设置过大聚类的标准
	cec.segment(*clusters);//获取聚类的结果，分割结果保存在点云索引的向量中
	cec.getRemovedClusters(small_clusters, large_clusters);//获取无效尺寸的聚类
	std::cerr << ">> Done: " << tt.toc() << " ms\n";
	//使用强度通道对输出进行延迟可视化
	pcl::copyPointCloud(*cloud_input,*cloud_out);
	for (int i = 0; i < small_clusters->size(); ++i)
		for (int j = 0; j < (*small_clusters)[i].indices.size(); ++j)
			cloud_out->points[(*small_clusters)[i].indices[j]].b = 255;
	for (int i = 0; i < large_clusters->size(); ++i)
		for (int j = 0; j < (*large_clusters)[i].indices.size(); ++j)
			cloud_out->points[(*large_clusters)[i].indices[j]].r = 255;
	for (int i = 0; i < clusters->size(); ++i)
	{
		int r = rand() % 200;
		int g = rand() % 200;
		int b = rand() % 200;
		for (int j = 0; j < (*clusters)[i].indices.size(); ++j)
		{
			cloud_out->points[(*clusters)[i].indices[j]].r = r;
			cloud_out->points[(*clusters)[i].indices[j]].g = g;
			cloud_out->points[(*clusters)[i].indices[j]].b = b;
		}

	}
}
// 可视化簇群
void showClusters(const pcl::PointCloud<PointT>::Ptr cloud, const std::vector<pcl::PointIndices>& cluster_indices, std::string name ,int viewport = 1)
{
	// 构造颜色句柄
	std::vector<pcl::visualization::PointCloudColorHandlerCustom<PointT>> color_handlers;
	for (size_t i = 0; i < cluster_indices.size(); ++i)
	{
		float r, g, b;
		float hue = static_cast<float>(i % 13 ) / 12.0f;
		hsv2rgb(hue, 1.0, 1.0, &r, &g, &b);
		color_handlers.emplace_back(cloud, r * 255.0f, g * 255.0f, b * 255.0f);
	}
	// 添加点云到可视化对象中
	for (size_t i = 0; i < cluster_indices.size(); ++i)
	{
		pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
		for (auto index : cluster_indices[i].indices)
			cluster->push_back(cloud->points[index]);
		name += std::to_string(i);
		if (viewer.contains(name)) 
			viewer.removePointCloud(name, viewport);
		viewer.addPointCloud(cluster, color_handlers[i], name, viewport);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
	}
}

// 欧式聚类分割
void euclideanClusterExtraction(pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointIndices>& cluster_indices, float tolerance = 0.1)
{
	std::cerr << "\n欧氏聚类...\n", tt.tic();
	// 创建 KdTree 对象
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);

	// 创建欧式聚类分割对象
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(tolerance);
	ec.setMinClusterSize(cloud->points.size() / 1000);
	ec.setMaxClusterSize(cloud->points.size() / 1);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);

	// 执行欧式聚类分割
	ec.extract(cluster_indices); 
	std::cerr << ">> Done: " << tt.toc() << " ms, 提取到 " << cluster_indices.size() << " 个簇\n";

}



// 在可视化工具中添加2d圆
void addCircle2D(pcl::visualization::PCLVisualizer& viewer, const pcl::ModelCoefficients& circle2d_coeff, const std::string& id) {
	// 在可视化工具中添加圆
	viewer.addCircle(circle2d_coeff, id, 1);

	// 设置圆的颜色和透明度
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, id);
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.9, id);
}
// 添加点云rgb
void addCloudRGB(pcl::PointCloud<PointT>::Ptr new_cloud, int r, int g, int b) {
	for (size_t i = 0; i < new_cloud->points.size(); ++i) {
		new_cloud->points[i].r = r;
		new_cloud->points[i].g = g;
		new_cloud->points[i].b = b;
	}
	*cloud_input += *new_cloud;

	// 更新可视化工具中的点云数据
	viewer.updatePointCloud(cloud_input, "cloud1");
}
// 计算圆柱轴端点
std::pair<PointT, PointT> getCylinderAxisEndPoints(const pcl::PointCloud<PointT>::Ptr cloud_cylinder, const pcl::ModelCoefficients::Ptr coefficients)
{
	// 圆柱轴起点
	float x = coefficients->values[0];
	float y = coefficients->values[1];
	float z = coefficients->values[2];
	Eigen::Vector3f origin(x, y, z);

	// 圆柱轴方向
	float dx = coefficients->values[3];
	float dy = coefficients->values[4];
	float dz = coefficients->values[5];
	Eigen::Vector3f axis(dx, dy, dz);

	// 计算点云中所有点在圆柱轴方向上的投影
	std::vector<float> projections(cloud_cylinder->points.size());
	for (size_t i = 0; i < cloud_cylinder->points.size(); ++i) {
		const auto& point = cloud_cylinder->points[i];
		Eigen::Vector3f point_vec(point.x, point.y, point.z);
		projections[i] = axis.dot(point_vec - origin);
	}

	// 找到投影值的最大值和最小值
	auto minmax = std::minmax_element(projections.begin(), projections.end());

	// 计算极小值点和极大值点在圆柱轴上的投影点
	Eigen::Vector3f min_proj_point = origin + axis * (*minmax.first);
	Eigen::Vector3f max_proj_point = origin + axis * (*minmax.second);

	// 极小值点和极大值点在圆柱轴上的投影点
	PointT min_proj_pt;
	min_proj_pt.x = min_proj_point[0];
	min_proj_pt.y = min_proj_point[1];
	min_proj_pt.z = min_proj_point[2];

	PointT max_proj_pt;
	max_proj_pt.x = max_proj_point[0];
	max_proj_pt.y = max_proj_point[1];
	max_proj_pt.z = max_proj_point[2];

	return std::make_pair(min_proj_pt, max_proj_pt);
}




#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <boost/make_shared.hpp>
// 由已知圆柱模型系数筛出形状点云
void extractCylinder(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<PointT>::Ptr cylinder_cloud)
{
	// 创建一个SampleConsensusModelCylinder对象
	pcl::SampleConsensusModelCylinder<PointT, pcl::Normal> model_cylinder(cloud_input);

	// 创建一个PointIndices对象来存储内点索引
	pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices);

	// 获取模型系数向量
	Eigen::VectorXf coeff(coefficients->values.size());
	for (size_t i = 0; i < coefficients->values.size(); ++i)
		coeff[i] = coefficients->values[i];
	std::cout << "coeff: " << coeff.transpose() << std::endl;

	// 调用selectWithinDistance函数选择位于圆柱模型上的点
	model_cylinder.selectWithinDistance(coeff, 0.01, inliers_ptr->indices);

	// 创建一个ExtractIndices对象
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);

	// 设置内点索引
	extract.setIndices(inliers_ptr);

	// 设置提取模式为非负（保留内点）
	extract.setNegative(false);

	// 提取圆柱体
	extract.filter(*cylinder_cloud);

}
// octree体素搜索
void getPointsInVoxels(pcl::PointCloud<PointT>::Ptr voxel_centers, pcl::PointCloud<PointT>::Ptr cloud_src, float leaf_size, pcl::PointIndices::Ptr voxel_indices)
{
	std::cerr << "\noctree体素搜索...\n", tt.tic();
	// 创建一个八叉树搜索对象
	pcl::octree::OctreePointCloudSearch<PointT> octree(leaf_size); 
	octree.setInputCloud(cloud_src); 
	octree.addPointsFromInputCloud();
	// 遍历所有的体素中心点
	for (size_t i = 0; i < voxel_centers->points.size(); ++i)
	{
		// 获取当前体素的中心点
		PointT voxelCenter = voxel_centers->points[i];
		// 获取与该点相对应的体素中所有点的索引
		std::vector<int> indices;
		octree.voxelSearch(voxelCenter, indices);
		for (int index : indices)
			voxel_indices->indices.push_back(index);
	}
	std::cerr << ">> Done: " << tt.toc() << " ms, " << voxel_indices->indices.size() << " points\n";
}

#include <unordered_set>
void inverse_downsample(pcl::PointCloud<PointT>::Ptr region_voxel, pcl::PointCloud<PointT>::Ptr region_src, pcl::PointIndices::Ptr region_src_indices) {
	std::cerr << "\n采样还原...\n", tt.tic();
	// 计算包围盒
	Eigen::Vector4f min_pt1, max_pt1, min_pt2, max_pt2;
	pcl::getMinMax3D(*region_voxel, min_pt1, max_pt1);
	float err_range = leaf_size * 2.0f;
	// 扩大包围盒
	min_pt1[0] -= err_range;
	min_pt1[1] -= err_range;
	min_pt1[2] -= err_range;
	max_pt1[0] += err_range;
	max_pt1[1] += err_range;
	max_pt1[2] += err_range;
	pcl::PointCloud<PointT>::Ptr box_src(new pcl::PointCloud<PointT>);
	pcl::PointIndices::Ptr box_indices(new pcl::PointIndices);
	// 根据包围盒裁切点云
	//filterCropBox(cloud_input, min_pt1, max_pt1, box_src, box_indices);
	//pcl::PointIndices::Ptr temp_indices(new pcl::PointIndices);
	// octree体素搜索 采样还原
	getPointsInVoxels(region_voxel, cloud_input,leaf_size, region_src_indices);


	//for (int index : temp_indices->indices)
	//	region_src_indices->indices.push_back(box_indices->indices[index]);
	// 根据索引提取点云
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud_input);
	extract.setIndices(region_src_indices);
	extract.setNegative(false);
	extract.filter(*region_src);

	pcl::getMinMax3D(*region_src, min_pt2, max_pt2);
	// 绘制包围盒
	std::string name_box = "box";
	std::string name_extend = "box_extend";
	int viewport = 3;
	if (viewer.contains(name_box))
		viewer.removeShape(name_box);
	if (viewer.contains(name_extend))
		viewer.removeShape(name_extend);
	viewer.addCube(min_pt1[0], max_pt1[0], min_pt1[1], max_pt1[1], min_pt1[2], max_pt1[2], 0.0, 1.0, 1.0, name_extend, viewport);
	viewer.addCube(min_pt2[0], max_pt2[0], min_pt2[1], max_pt2[1], min_pt2[2], max_pt2[2], 1.0, 0.0, 1.0, name_box, viewport);
	// 设置立方体的透明度
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, name_box, viewport);
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name_extend, viewport);

	// 打印包围盒
	std::cerr << "扩张后的包围盒: (" << min_pt1[0] << ", " << min_pt1[1] << ", " << min_pt1[2] << ") - (" << max_pt1[0] << ", " << max_pt1[1] << ", " << max_pt1[2] << ")" << std::endl;
	std::cerr << "实际包围盒: (" << min_pt2[0] << ", " << min_pt2[1] << ", " << min_pt2[2] << ") - (" << max_pt2[0] << ", " << max_pt2[1] << ", " << max_pt2[2] << ")" << std::endl;

	std::cerr << ">> Done: " << tt.toc() << " ms, " << region_src->points.size() << " points\n";

	//viewer.addPointCloud(region_src, "cloud4");
	//将cloud_input中的region_src_indices索引对应的点的颜色改为绿色
	pcl::PointCloud<PointT>::Ptr cloud_draw(new pcl::PointCloud<PointT>);
	pcl::copyPointCloud(*cloud_input, *cloud_draw);
	for (const auto& index : region_src_indices->indices) {
		cloud_draw->points[index].r = 0;
		cloud_draw->points[index].g = 255;
		cloud_draw->points[index].b = 0;
	}
	viewer.updatePointCloud(cloud_draw, "cloud1");


}

void inverse_downsample_old(pcl::PointCloud<PointT>::Ptr region_voxel, pcl::PointCloud<PointT>::Ptr region_src, pcl::PointIndices::Ptr region_src_indices) {
	std::cerr << "\n采样还原...\n", tt.tic();
	// 计算包围盒
	Eigen::Vector4f min_pt1, max_pt1, min_pt2, max_pt2;
	pcl::getMinMax3D(*region_voxel, min_pt1, max_pt1);
	float err_range = leaf_size *2.0f;
	// 扩大包围盒
	min_pt1[0] -= err_range;
	min_pt1[1] -= err_range;
	min_pt1[2] -= err_range;
	max_pt1[0] += err_range;
	max_pt1[1] += err_range;
	max_pt1[2] += err_range;
	pcl::PointCloud<PointT>::Ptr box_src(new pcl::PointCloud<PointT>);
	pcl::PointIndices::Ptr box_indices(new pcl::PointIndices);
	filterCropBox(cloud_input,min_pt1, max_pt1, box_src, box_indices);
	pcl::search::KdTree<PointT> kdtree;
	kdtree.setInputCloud(box_src);
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	std::unordered_set<int> added_indices; 
	int num = 0;
	for (size_t i = 0; i < region_voxel->points.size(); ++i) {
		PointT searchPoint = region_voxel->points[i];
		if (kdtree.radiusSearch(searchPoint, leaf_size*1.0, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
			for (size_t j = 0; j < pointIdxNKNSearch.size(); ++j) {
				if (added_indices.count(pointIdxNKNSearch[j]) == 0) // 去重
				{
					PointT pt = box_src->points[pointIdxNKNSearch[j]]; // 点
					auto pt_idx = box_indices->indices[pointIdxNKNSearch[j]]; // 点在cloud_input中的索引
					region_src->push_back(pt);
					added_indices.insert(pt_idx);
				}
			}
		}
	}
	region_src_indices->indices.assign(added_indices.begin(), added_indices.end()); // 元素复制
	pcl::getMinMax3D(*region_src, min_pt2, max_pt2);
	// 绘制包围盒
	std::string name_box = "box";
	std::string name_extend = "box_extend";
	int viewport = 4;
	if (viewer.contains(name_box))
		viewer.removeShape(name_box);
	if (viewer.contains(name_extend))
		viewer.removeShape(name_extend);
	viewer.addCube(min_pt1[0], max_pt1[0], min_pt1[1], max_pt1[1], min_pt1[2], max_pt1[2], 0.0, 1.0, 1.0, name_extend, viewport);
	viewer.addCube(min_pt2[0], max_pt2[0], min_pt2[1], max_pt2[1], min_pt2[2], max_pt2[2], 1.0, 0.0, 1.0, name_box, viewport);
	// 设置立方体的透明度
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, name_box, viewport);
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name_extend, viewport);

	// 打印包围盒
	std::cerr << "扩张后的包围盒: (" << min_pt1[0] << ", " << min_pt1[1] << ", " << min_pt1[2] << ") - (" << max_pt1[0] << ", " << max_pt1[1] << ", " << max_pt1[2] << ")" << std::endl;
	std::cerr << "实际包围盒: (" << min_pt2[0] << ", " << min_pt2[1] << ", " << min_pt2[2] << ") - (" << max_pt2[0] << ", " << max_pt2[1] << ", " << max_pt2[2] << ")" << std::endl;

	std::cerr << ">> Done: " << tt.toc() << " ms, " << region_src->points.size() << " points\n";

	//viewer.addPointCloud(region_src, "cloud4");
	//将cloud_input中的region_src_indices索引对应的点的颜色改为绿色
	pcl::PointCloud<PointT>::Ptr cloud_draw(new pcl::PointCloud<PointT>);
	pcl::copyPointCloud(*cloud_input, *cloud_draw);
	for (const auto& index : region_src_indices->indices) {
		cloud_draw->points[index].r = 0;
		cloud_draw->points[index].g = 255;
		cloud_draw->points[index].b = 0;
	}
	viewer.updatePointCloud(cloud_draw, "cloud3");
}

// octree 体素
void getVoxelCenters(pcl::PointCloud<PointT>::Ptr cloud_src, float leaf_size, pcl::PointCloud<PointT>::Ptr voxel_cloud)

{

	std::cerr << "octree体素化...\n", tt.tic();//下采样

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

	std::cerr << ">> Done: " << tt.toc() << " ms, " << voxel_cloud->points.size() << " points\n";
}

// 体素化滤波（重心）
void filterVoxelGrid(const pcl::PointCloud<PointT>::Ptr& input_cloud,
	pcl::PointCloud<PointT>::Ptr& output_cloud, float leaf_size)
{
	std::cerr << "DownSampling...\n", tt.tic();//下采样
	pcl::VoxelGrid<PointT> voxel_grid;
	voxel_grid.setInputCloud(input_cloud);
	//voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size); 
	voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size); 
	voxel_grid.setDownsampleAllData(false); // true:所有字段都进行下采样。false:仅对 XYZ 进行下采样
	voxel_grid.filter(*output_cloud);
	std::cerr << ">> Done: " << tt.toc() << " ms, " << output_cloud->points.size() << " points\n";
}

// 计算点云法向
void computeNormals(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {
	std::cerr << "计算法向...\n", tt.tic();
	// 建立搜索KD树
	pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	// 计算点云法向
	pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree); // 搜索方法为kd树走索
	normal_estimator.setInputCloud(cloud);  // 填入点云
	//normal_estimator.setKSearch(50);        // 设置搜索范围
	normal_estimator.setRadiusSearch(0.07);//设置搜索半径大小
	normal_estimator.setNumberOfThreads(16); // 线程数
	normal_estimator.compute(*normals);     // 将法相保存在normals
	auto end_1 = std::clock();
	std::cerr << ">> Done: " << tt.toc() << " ms\n"; //计算法线估计所使用的时间
}

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


// 计算底面和顶面的圆心 （kdtree+SAC）
std::pair<PointT, PointT> calcBottomTopCenter(const pcl::PointCloud<PointT>::Ptr cloud_cylinder, pcl::ModelCoefficients::Ptr coefficients, const PointT& axis_pt_min, const PointT& axis_pt_max)
{
	double radius = coefficients->values[6]; // 圆柱半径
	// 创建一个KdTreeFLANN对象
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud_cylinder);
	// 创建一个点云对象，用于存储底面点云

	pcl::PointCloud<PointT>::Ptr bottom_cloud(new pcl::PointCloud<PointT>);
	// 在KdTree中搜索axis_pt_min附近的点
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	if (kdtree.radiusSearch(axis_pt_min, radius*3.0, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
			bottom_cloud->points.push_back(cloud_cylinder->points[pointIdxRadiusSearch[i]]);
		}
	}

	// 创建一个点云对象，用于存储顶面点云
	pcl::PointCloud<PointT>::Ptr top_cloud(new pcl::PointCloud<PointT>);
	// 在KdTree中搜索axis_pt_max附近的点
	if (kdtree.radiusSearch(axis_pt_max, radius * 3.0, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
			top_cloud->points.push_back(cloud_cylinder->points[pointIdxRadiusSearch[i]]);
		}
	}

	addCloudRGB(bottom_cloud, 255, 127, 0);
	addCloudRGB(top_cloud, 255, 127, 0);

	// 对底面点云进行圆拟合
	pcl::SACSegmentation<PointT> seg_bottom;
	pcl::ModelCoefficients::Ptr coefficients_bottom(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_bottom(new pcl::PointIndices);
	seg_bottom.setOptimizeCoefficients(true);
	seg_bottom.setModelType(pcl::SACMODEL_CIRCLE2D);
	seg_bottom.setMethodType(pcl::SAC_RANSAC);
	seg_bottom.setDistanceThreshold(0.05);
	seg_bottom.setInputCloud(bottom_cloud);
	seg_bottom.segment(*inliers_bottom, *coefficients_bottom);

	PointT bottom_center;
	bottom_center.x = coefficients_bottom->values[0];
	bottom_center.y = coefficients_bottom->values[1];
	bottom_center.z = axis_pt_min.z;

	// 对顶面点云进行圆拟合
	pcl::SACSegmentation<PointT> seg_top;
	pcl::ModelCoefficients::Ptr coefficients_top(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_top(new pcl::PointIndices);
	seg_top.setOptimizeCoefficients(true);
	seg_top.setModelType(pcl::SACMODEL_CIRCLE2D);
	seg_top.setMethodType(pcl::SAC_RANSAC);
	seg_top.setDistanceThreshold(0.05);
	seg_top.setInputCloud(top_cloud);
	seg_top.segment(*inliers_top, *coefficients_top);

	PointT top_center;
	top_center.x = coefficients_top->values[0];
	top_center.y = coefficients_top->values[1];
	top_center.z = axis_pt_max.z;

	// 创建一个点云对象，用于存储拟合出的圆
	pcl::PointCloud<PointT>::Ptr circle_cloud(new pcl::PointCloud<PointT>);
	for (size_t i = 0; i < inliers_bottom->indices.size(); ++i) {
		circle_cloud->points.push_back(bottom_cloud->points[inliers_bottom->indices[i]]);
	}
	for (size_t i = 0; i < inliers_top->indices.size(); ++i) {
		circle_cloud->points.push_back(top_cloud->points[inliers_top->indices[i]]);
	}

	//addCircle2D(viewer,*coefficients_bottom,"circle_bottom");
	//addCircle2D(viewer, *coefficients_top, "circle_top");
	// 使用addCloudRGB方法可视化拟合出的圆
	addCloudRGB(circle_cloud, 255, 255, 0); 
	// 使用addCloudRGB方法可视化拟合出的圆
	addCloudRGB(circle_cloud, 255, 255, 0); 

	return std::make_pair(bottom_center, top_center);
}
// 在可视化工具中添加线段
void addLine(pcl::visualization::PCLVisualizer& viewer, PointT point1, PointT point2, const std::string& line_id, double r, double g, double b)
{
	// 在可视化工具中添加线段
	viewer.addLine(point1, point2, r, g, b, line_id);

	// 设置线段的颜色和宽度
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, line_id);
}

// 在可视化工具中添加线段
void addLine(pcl::visualization::PCLVisualizer& viewer, PointT point1, PointT point2)
{
	if (viewer.contains("line")) {
		viewer.removeShape("line", 1);
	}
	// 在可视化工具中添加线段
	viewer.addLine(point1, point2, "line");

	// 设置线段的颜色和宽度
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 1.0, "line");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "line");
}
// 单次SAC分割
void segmentCloud_Single(const pcl::PointCloud<PointT>::Ptr cloud , pcl::IndicesPtr indices_region) {

	auto start = std::clock();
	// 创建一个SACSegmentation对象，方法类型为RANSAC，并设置模型类型为圆柱
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	// 设置距离阈值
	seg.setDistanceThreshold(0.05);
	// 设置最大迭代次数
	//seg.setMaxIterations(100);
	// 设置概率
	//seg.setProbability(0.6);
	
	seg.setInputCloud(cloud); // 设置输入点云
	seg.setInputNormals(cloud_normals);  // 设置输入法向
	// 创建一个模型系数对象和一个内点索引对象
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// 调用segment方法进行分割
	seg.segment(*inliers, *coefficients);


	// 检查是否分割成功
	if (inliers->indices.size() == 0) {
		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		return;
	}

	// 分割出的形状点云
	pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>);
	//extractCylinder(cloud, coefficients, plane);

	// 从原始点云中提取分割出的形状点云
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_cylinder);

	auto end_1 = std::clock();
	std::cout << "SAC分割，耗时：" << std::difftime(end_1, start) << "ms" << std::endl;

	// 输出分割结果
	std::cout << "模型系数: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;

	std::cout << "模型内点: " << inliers->indices.size() << std::endl;

	//addCloudRGB(cloud_cylinder,255,0,0);

	//pcl::PointCloud<PointT>::Ptr cloud_display(new pcl::PointCloud<PointT>); //显示用的点云
	//pcl::copyPointCloud(*cloud_input, *cloud_display);
	//for (size_t i = 0; i < inliers->indices.size(); ++i) {
	//	size_t idx = (*indices_region)[inliers->indices[i]];
	//	cloud_display->points[idx].r = 255.0;
	//	cloud_display->points[idx].g = 0.0;
	//	cloud_display->points[idx].b = 0.0;
	//}

	// 创建颜色处理器
	pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(cloud_cylinder, 0, 255, 0);

	// 更新可视化工具中的点云数据
	if (viewer.contains("cloud_cylinder")) {
		viewer.updatePointCloud(cloud_cylinder, color_handler, "cloud_cylinder");
	}
	else
	{
		viewer.addPointCloud(cloud_cylinder, color_handler, "cloud_cylinder");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_cylinder");
	}


	// 更新可视化工具中的点云数据
	//viewer.updatePointCloud(cloud_display, "cloud1");

	float x = coefficients->values[0]; // 圆柱轴起点的x坐标
	float y = coefficients->values[1]; // 圆柱轴起点的y坐标
	float z = coefficients->values[2]; // 圆柱轴起点的z坐标
	float dx = coefficients->values[3]; // 圆柱轴方向的x分量
	float dy = coefficients->values[4]; // 圆柱轴方向的y分量
	float dz = coefficients->values[5]; // 圆柱轴方向的z分量
	float radius = coefficients->values[6]; // 圆柱半径

	// 计算圆柱轴端点
	auto axis_end_pts = getCylinderAxisEndPoints(cloud_cylinder, coefficients); 
	addLine(viewer, axis_end_pts.first, axis_end_pts.second); // 可视化圆柱轴线

	

	// 计算xy偏距
	float dist_xy = sqrt(pow(axis_end_pts.first.x - axis_end_pts.second.x, 2) + pow(axis_end_pts.first.y - axis_end_pts.second.y, 2));

	// 计算圆柱轴线高度
	float height_axis = sqrt(pow(axis_end_pts.first.x - axis_end_pts.second.x, 2) + pow(axis_end_pts.first.y - axis_end_pts.second.y, 2) + pow(axis_end_pts.first.z - axis_end_pts.second.z, 2));

	// 计算偏差比
	float ratio = dist_xy / height_axis;

	// 打印结果
	std::cout << "轴底坐标: (" << axis_end_pts.first.x << ", " << axis_end_pts.first.y << ", " << axis_end_pts.first.z << ")" << std::endl;
	std::cout << "轴顶坐标: (" << axis_end_pts.second.x << ", " << axis_end_pts.second.y << ", " << axis_end_pts.second.z << ")" << std::endl;
	std::cout << "xy偏距: " << dist_xy << std::endl;
	std::cout << "圆柱轴线高度: " << height_axis << std::endl;
	std::cout << "偏差比: " << ratio << std::endl;

	int x_cur = 60;
	int y_cur = 60;
	int y_offset = 20;
	double r = 1.0;
	double g = 1.0;
	double b = 1.0;
	int viewport = 1;
	if (viewer.contains("text_1")) 
	{
		viewer.updateText("Axis bottom coordinates: (" + std::to_string(axis_end_pts.first.x) + ", " + std::to_string(axis_end_pts.first.y) + ", " + std::to_string(axis_end_pts.first.z) + ")", x_cur, y_cur, "text_1");
		y_cur += y_offset;
		viewer.updateText("Axis top coordinates: (" + std::to_string(axis_end_pts.second.x) + ", " + std::to_string(axis_end_pts.second.y) + ", " + std::to_string(axis_end_pts.second.z) + ")", x_cur, y_cur, "text_2");
		y_cur += y_offset;
		viewer.updateText("XY offset: " + std::to_string(dist_xy), x_cur, y_cur, "text_3");
		y_cur += y_offset;
		viewer.updateText("Axis height: " + std::to_string(height_axis), x_cur, y_cur, "text_4");
		y_cur += y_offset;
		viewer.updateText("Ratio: " + std::to_string(ratio), x_cur, y_cur, "text_5");
	}
	else
	{
		viewer.addText("Axis bottom coordinates: (" + std::to_string(axis_end_pts.first.x) + ", " + std::to_string(axis_end_pts.first.y) + ", " + std::to_string(axis_end_pts.first.z) + ")", x_cur, y_cur, r, g, b, "text_1", 1);
		y_cur += y_offset;
		viewer.addText("Axis top coordinates: (" + std::to_string(axis_end_pts.second.x) + ", " + std::to_string(axis_end_pts.second.y) + ", " + std::to_string(axis_end_pts.second.z) + ")", x_cur, y_cur, r, g, b, "text_2", 1);
		y_cur += y_offset;
		viewer.addText("XY offset: " + std::to_string(dist_xy), x_cur, y_cur, r, g, b, "text_3", 1);
		y_cur += y_offset;
		viewer.addText("Axis height: " + std::to_string(height_axis), x_cur, y_cur, r, g, b, "text_4", 1);
		y_cur += y_offset;
		viewer.addText("Ratio: " + std::to_string(ratio), x_cur, y_cur, r, g, b, "text_5", 1);
	}


	//viewer.addText("轴底坐标: (" + std::to_string(axis_end_pts.first.x) + ", " + std::to_string(axis_end_pts.first.y) + ", " + std::to_string(axis_end_pts.first.z) + ")", x_cur, y_cur, r, g, b, "text_1", 1);
	//y_cur += y_offset;
	//viewer.addText("轴顶坐标: (" + std::to_string(axis_end_pts.second.x) + ", " + std::to_string(axis_end_pts.second.y) + ", " + std::to_string(axis_end_pts.second.z) + ")", x_cur, y_cur, r, g, b, "text_2", 1);
	//y_cur += y_offset;
	//viewer.addText("xy偏距: " + std::to_string(dist_xy), x_cur, y_cur, r, g, b, "text_3", 1);
	//y_cur += y_offset;
	//viewer.addText("圆柱轴线高度: " + std::to_string(height_axis), x_cur, y_cur, r, g, b, "text_4", 1);
	//y_cur += y_offset;
	//viewer.addText("偏差比: " + std::to_string(ratio), x_cur, y_cur, r, g, b, "text_5", 1);




	// 计算底面和顶面的圆心
	//auto circle2d_center_pts = calcBottomTopCenter(cloud_cylinder, coefficients, axis_end_pts.first, axis_end_pts.second);
	//addLine(viewer, circle2d_center_pts.first, circle2d_center_pts.second,"line2",1.0,0.0,1.0); // 可视化圆心连线
	//std::cout << "底面圆心坐标: (" << circle2d_center_pts.first.x << ", " << circle2d_center_pts.first.y << ", " << circle2d_center_pts.first.z << ")" << std::endl;
	//std::cout << "顶面圆心坐标: (" << circle2d_center_pts.second.x << ", " << circle2d_center_pts.second.y << ", " << circle2d_center_pts.second.z << ")" << std::endl;

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
		if (std::abs(distance) < 0.01) { // 将0.01替换为想要使用的阈值
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
// 分割SAC平面
void getSACplane(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr plane) {

	auto start = std::clock();
	// 创建一个SACSegmentation对象，方法类型为RANSAC，并设置模型类型为圆柱
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	// 设置距离阈值
	seg.setDistanceThreshold(0.05);
	// 设置最大迭代次数
	//seg.setMaxIterations(100);
	// 设置概率
	//seg.setProbability(0.6);

	seg.setInputCloud(cloud); // 设置输入点云
	seg.setInputNormals(cloud_normals);  // 设置输入法向
	// 创建一个模型系数对象和一个内点索引对象
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// 调用segment方法进行分割
	seg.segment(*inliers, *coefficients);


	// 检查是否分割成功
	if (inliers->indices.size() == 0) {
		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		return;
	}

	// 分割出的形状点云



	// 从原始点云中提取分割出的形状点云
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*plane);
}
// 找point的邻域
void getPointNeiboor(const pcl::PointCloud<PointT>::Ptr cloud, PointT point, double radius, pcl::PointCloud<PointT>::Ptr neiboor)
{
	std::cerr << "\nkdtree半径搜索point的邻域...\n", tt.tic();
	// 创建一个kdtree对象
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);

	// 进行半径搜索
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	kdtree.radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

	for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
		neiboor->points.push_back(cloud->points[pointIdxRadiusSearch[i]]);
	}
	std::cerr << ">> Done: " << tt.toc() << " ms, " << neiboor->points.size() << " neiboor points\n";
}
// 找立方体形状的邻域
void getPointNeiboor(const pcl::PointCloud<PointT>::Ptr& all_points, int POINTS_Index, float radius, pcl::PointCloud<PointT>::Ptr& neiboor)
{
	std::cerr << "\n找point的邻域（立方体）...\n", tt.tic();
	PointT center_point = all_points->points[POINTS_Index];
	float Xb = center_point.x -  radius;
	float Yb = center_point.y -  radius;
	float Zb = center_point.z -  radius;
	float Xb2 = center_point.x +  radius;
	float Yb2 = center_point.y +  radius;
	float Zb2 = center_point.z +  radius;

	for (size_t i = 0; i < all_points->size(); ++i)
	{
		PointT current_point = all_points->points[i];
		if ((current_point.x >= Xb && current_point.x <= Xb2) &&
			(current_point.y >= Yb && current_point.y <= Yb2) &&
			(current_point.z >= Zb && current_point.z <= Zb2))
		{
			neiboor->points.push_back(current_point);
		}
	}
	std::cerr << ">> Done: " << tt.toc() << " ms, " << neiboor->points.size() << " neiboor points\n";
}


// 在点云中迭代进行SAC，每次找出平面时，根据平面与已知点的距离阈值 和 平面包围盒与已知点的关系 判断是否保留该平面
void getPlaneBySacPoint(const pcl::PointCloud<PointT>::Ptr cloud, PointT point, pcl::ModelCoefficients::Ptr coeff)
{
	std::cerr << "\n邻域内迭代SAC...\n", tt.tic();

	// 创建一个SACSegmentation对象，并设置模型类型为平面，方法类型为RANSAC
	pcl::SACSegmentation<PointT> seg;
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);

	// 设置距离阈值
	seg.setDistanceThreshold(0.1);

	// 深复制一份cloud
	pcl::PointCloud<PointT>::Ptr cloud_copy(new pcl::PointCloud<PointT>(*cloud));

	// 设置输入点云
	seg.setInputCloud(cloud_copy);

	// 创建一个模型系数对象和一个内点索引对象
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// 迭代分割
	while (cloud_copy->points.size() > 0.001 * cloud->points.size())
	{
		// 调用segment方法进行分割
		seg.segment(*inliers, *coefficients);

		// 检查是否分割成功
		if (inliers->indices.size() == 0)
		{
			std::cerr << "分割出的SAC模型为空" << std::endl;
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

		// 计算输入点到平面的距离
		float distance = calcPointToPlaneDistance(point, coefficients);

		Eigen::Vector4f min, max;
		pcl::getMinMax3D(*plane, min, max); // 获取平面的最小包围盒
		bool in_box = point.x >= min[0] && point.x <= max[0] &&
			point.y >= min[1] && point.y <= max[1] && // 检查y坐标是否在范围内
			point.z >= min[2] && point.z <= max[2];

		// 检查输入点是否在分割出的平面上 且 是否在平面的最小包围盒内
		if (std::abs(distance) < 0.1 && in_box) // 将0.01替换为想要使用的阈值
		{
			// 输入点在分割出的平面上
			*coeff = *coefficients;
			// 获取模型系数向量
			Eigen::VectorXf coeff_eigen(coefficients->values.size());
			for (size_t i = 0; i < coefficients->values.size(); ++i)
				coeff_eigen[i] = coefficients->values[i];
			std::cout << "找到了邻域中已知点所在的平面！coeff: " << coeff_eigen.transpose() << std::endl;
			break;
		}

		// 从cloud_copy中移除本次分割出的点们
		extract.setNegative(true);
		extract.filter(*cloud_copy);
	}

	std::cerr << ">> Done: " << tt.toc() << " ms" << std::endl;
}


#include <pcl/filters/model_outlier_removal.h>
// 模型异常值剔除 。 从点云中提取与模型距离小于阈值的点
void modelOutlierRemoval(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::ModelCoefficients::Ptr coeff, float distanceThreshold, pcl::PointCloud<PointT>::Ptr cloud_filtered) {
	std::cerr << "\n模型异常值剔除...\n", tt.tic();
	pcl::ModelOutlierRemoval<PointT> filter;
	filter.setModelCoefficients(*coeff);
	filter.setThreshold(distanceThreshold);
	filter.setModelType(pcl::SACMODEL_PLANE);
	filter.setInputCloud(cloud);
	filter.filter(*cloud_filtered);
	std::cerr << ">> Done: " << tt.toc() << " ms, 模型异常值剔除后剩余：" << cloud_filtered->points.size() << " points\n";
}

// 从簇群中找出已知点所在的簇
void findClusterContainingPoint(const PointT& point,
	const pcl::PointCloud<PointT>::Ptr& region,
	const std::vector<pcl::PointIndices>& clusters,
	pcl::PointCloud<PointT>::Ptr& plane) {

	std::cerr << "\n从簇群中找出已知点所在的簇...\n", tt.tic();
	int containing_cluster_index = -1;

	for (std::size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
		const pcl::PointIndices& cluster_indices = clusters[cluster_index];
		for (const int index : cluster_indices.indices) {
			const PointT& cluster_point = region->points[index];
			if (cluster_point.x == point.x && cluster_point.y == point.y && cluster_point.z == point.z) {
				containing_cluster_index = static_cast<int>(cluster_index);
				break;
			}
		}

		if (containing_cluster_index != -1) {

			// Extract the indices of the points in the specified cluster
			const pcl::PointIndices& selected_indices = clusters[containing_cluster_index];

			// Create a new point cloud containing the points in the cluster
			plane.reset(new pcl::PointCloud<PointT>);
			pcl::copyPointCloud(*region, selected_indices.indices, *plane);

			std::cerr << ">> Done: " << tt.toc() << " ms, 已知点在簇 " << containing_cluster_index << "（"<< plane->points.size() <<" points）中 \n" ;
			return;
		}
	}

	std::cout << "Point is not found in any cluster" << std::endl;

}
// 自定义多种聚类
void mutiCluster(const PointT& point, pcl::PointCloud<PointT>::Ptr cloud, float leaf_size, pcl::PointCloud<PointT>::Ptr& plane) {
	// 欧氏聚类
	std::vector<pcl::PointIndices> cluster_euc;
	euclideanClusterExtraction(cloud, cluster_euc, leaf_size*4.0f);
	//showClusters(cloud, cluster_euc, "cluster_eu", 1);
	// 找出已知点所在的簇
	pcl::PointCloud<PointT>::Ptr plane_euc(new pcl::PointCloud<PointT>);
	findClusterContainingPoint(point, cloud, cluster_euc, plane_euc);

	// 密度聚类
	//std::vector<pcl::PointIndices> cluster_db;
	//getDbscanCluster(plane_euc, cluster_db);
	//showClusters(plane_euc, cluster_db, "cluster_db", 2);
	// 找出已知点所在的簇
	//pcl::PointCloud<PointT>::Ptr plane_db(new pcl::PointCloud<PointT>);
	//findClusterContainingPoint(point, plane_euc, cluster_db, plane_db);

	// 区域生长分割
	std::vector<pcl::PointIndices> cluster_rg;
	regionGrowingSegmentation(plane_euc, cluster_rg);



	//showClusters(plane_euc, cluster_rg, "cluster_rg", 2);
	// 找出已知点所在的簇
	//pcl::PointCloud<PointT>::Ptr plane_rg(new pcl::PointCloud<PointT>);
	findClusterContainingPoint(point, plane_euc, cluster_rg, plane);



	//pcl::visualization::PointCloudColorHandlerCustom<PointT> color(plane, 0.0f, 255.0f, 0.0f);
	//std::string name = "plane";
	//if (viewer.contains(name))
	//	viewer.removePointCloud(name);
	//viewer.addPointCloud(plane, color, name, 2);
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
}



// 拟合平面
void getPlane(const pcl::PointCloud<PointT>::Ptr cloud, int point_idx, float nearSearchRadius, float removalDistThresh)
{
	PointT point = cloud->points[point_idx];
	int seedIndex = 0;
	pcl::PointCloud<PointT>::Ptr neiboor(new pcl::PointCloud<PointT>);
	//getPointNeiboor(cloud, point, nearSearchRadius, neiboor); // kdtree找邻域

	getPointNeiboor(cloud, point_idx, nearSearchRadius, neiboor); // 找邻域（立方体）
	pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
	getPlaneBySacPoint(neiboor, point, coeff); // 找模型
	// 检查找模型是否成功
	if (coeff == nullptr) {
		std::cerr << "没有找出模型！" << std::endl;
		return;
	}

	pcl::PointCloud<PointT>::Ptr region(new pcl::PointCloud<PointT>);
	//*region = *cloud;
	modelOutlierRemoval(cloud, coeff, removalDistThresh, region); // 模型异常值剔除
	//addCloudRGB(region, 0, 255, 255);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	//computeNormals(region, normals); // 在region内计算法线
	float distanceThreshold;
	float curvatureThreshold;
	float normalThreshold;

	//std::vector<pcl::PointIndices> cluster_eu; // 簇 欧氏
	//std::vector<pcl::PointIndices> cluster_db; // 簇 DB密度
	//std::vector<pcl::PointIndices> cluster_rg; // 簇 区域生长
	//euclideanClusterExtraction(region, cluster_eu, 0.3); // 欧氏聚类
	//getDbscanCluster(region, cluster_db); // DBSCAN密度聚类
	//regionGrowingSegmentation(region, cluster_rg); // 区域生长分割簇群

	//showClusters(region, cluster_eu, "cluster_eu", 1);
	//showClusters(region, cluster_db, "cluster_db", 1);	// 可视化簇群
	//showClusters(region, cluster_rg, "cluster_rg", 1);



	pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>);
	mutiCluster(point, region, leaf_size, plane);
	//findClusterContainingPoint(point, region, cluster_rg, plane); // 找出已知点所在的簇
	//float random_hue = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX); //将时间值转换为无符号整数类型作为种子。这样，每次程序运行时，种子都会不同，从而产生不同的随机序列
	//float r, g, b;
	//hsv2rgb(random_hue, 1.0, 1.0, &r, &g, &b);
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> color(plane, r * 255.0f, g * 255.0f, b * 255.0f);
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> color(plane, 0.0f, 255.0f, 0.0f);
	//std::string name = "plane";
	//if (viewer.contains(name))
	//	viewer.removePointCloud(name);
	//viewer.addPointCloud(plane, color, name, 2);
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);

	pcl::PointCloud<PointT>::Ptr plane_src(new pcl::PointCloud<PointT>);
	pcl::PointIndices::Ptr plane_src_indices(new pcl::PointIndices);
	inverse_downsample(plane, plane_src, plane_src_indices);

	//pcl::PointCloud<PointT>::Ptr plane_sac(new pcl::PointCloud<PointT>);
	//getSacPlane(plane_src, plane_sac); // SAC
	//pcl::PointCloud<PointT>::Ptr plane_sac(new pcl::PointCloud<PointT>);
	//getSACplane(plane_src,plane_sac);
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> color(plane, 255.0f, 0.0f, 0.0f);
	//std::string name = "plane_sac";
	//if (viewer.contains(name))
	//	viewer.removePointCloud(name);
	//viewer.addPointCloud(plane_sac, color, name, 4);
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}


#include <vector>
#include <queue>
#include <cmath>

struct Point {
	float x, y, z;
};

float squaredDistance(const PointT& a, const PointT& b) {
	return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
}



std::vector<int> regionGrowing(const pcl::PointCloud<PointT>::Ptr cloud, int seedIndex, float distanceThreshold, float curvatureThreshold, float normalThreshold) {
	// 计算法向量
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	tree->setInputCloud(cloud);
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setKSearch(20);
	ne.compute(*normals);

	// 查找每个点的k个最近邻
	int k = 20;
	int points_num = cloud->points.size();
	std::vector<int> k_nebor_index;
	std::vector<float> k_nebor_index_dis;
	std::vector<std::vector<int>> point_k_index(points_num, k_nebor_index);
	for (size_t i = 0; i < points_num; i++) {
		if (tree->nearestKSearch(cloud->points[i], k, k_nebor_index, k_nebor_index_dis)) {
			point_k_index[i].swap(k_nebor_index);
		}
		else {
			PCL_ERROR("WARNING:FALSE NEARERTKSEARCH");
		}
	}

	// 计算每个点的曲率值
	std::vector<std::pair<float, int>> vec_curvature;
	for (size_t i = 0; i < points_num; i++) {
		vec_curvature.push_back(std::make_pair(normals->points[i].curvature, i));
	}
	std::sort(vec_curvature.begin(), vec_curvature.end());

	// 初始化种子点
	int seed_orginal = seedIndex;
	int counter_0 = 0;
	int segment_laber(0);
	std::vector<int> segmen_num;
	std::vector<int> point_laber(points_num, -1);

	while (counter_0 < points_num) {
		std::queue<int> seed;
		seed.push(seed_orginal);
		point_laber[seed_orginal] = segment_laber;
		int counter_1(1);

		while (!seed.empty()) {
			int curr_seed = seed.front();
			seed.pop();
			int curr_nebor_index(0);

			while (curr_nebor_index < k) {
				bool is_a_seed = false;
				int cur_point_idx = point_k_index[curr_seed][curr_nebor_index];
				if (point_laber[cur_point_idx] != -1) {
					curr_nebor_index++;
					continue;
				}

				Eigen::Map<Eigen::Vector3f> vec_curr_point(static_cast<float*>(normals->points[curr_seed].normal));
				Eigen::Map<Eigen::Vector3f> vec_nebor_point(static_cast<float*>(normals->points[cur_point_idx].normal));
				float dot_normals = fabsf(vec_curr_point.dot(vec_nebor_point));

				if (dot_normals < normalThreshold) {
					is_a_seed = false;
				}
				else if (normals->points[cur_point_idx].curvature > curvatureThreshold) {
					is_a_seed = false;
				}
				else {
					is_a_seed = true;
				}

				if (!is_a_seed) {
					curr_nebor_index++;
					continue;
				}

				point_laber[cur_point_idx] = segment_laber;
				counter_1++;
				if (is_a_seed) {
					seed.push(cur_point_idx);
				}
				curr_nebor_index++;
			}
		}

		segment_laber++;
		counter_0 += counter_1;
		segmen_num.push_back(counter_1);

		for (size_t i = 0; i < points_num; i++) {
			int index_curvature = vec_curvature[i].second;
			if (point_laber[index_curvature] == -1) {
				seed_orginal = index_curvature;
				break;
			}
		}
	}

	std::vector<int> cluster;
	for (int i = 0; i < points_num; i++) {
		if (point_laber[i] == 0) {
			cluster.push_back(i);
		}
	}

	return cluster;
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



// 在可视化工具中添加圆柱体
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

bool isDKeyPressed = false;
// 键盘事件
void keyboardEventCallback(const pcl::visualization::KeyboardEvent& event, void* viewer_void) {
	if (event.getKeySym() == "c" && event.keyDown()) {
		isDKeyPressed = true;
	}
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

// 鼠标单击事件回调函数
void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* viewer_void) {

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
	if (z==0||idx<=10) 
	{
		std::cerr << "选点失败!请重新选点!" << idx << std::endl;
		if (viewer.contains("text_selected_point"))
		{
			viewer.updateText("Invalid point, Please select a new point ! ", 40, 40, 1.0, 1.0, 0.0, "text_selected_point");
		}
		else
		{
			viewer.addText("Invalid point, Please select a new point ! ", 40, 40, 1.0, 1.0, 0.0, "text_selected_point");
		}
		return;
	}
	std::cout << "选点：x=" << x << ", y=" << y << ", z=" << z << ", idx=" << idx << std::endl;
	if (viewer.contains("text_selected_point"))
	{
		viewer.updateText("Selected point: x=" + std::to_string(x) + ", y=" + std::to_string(y) + ", z=" + std::to_string(z), 40, 40, 0.0, 1.0, 0.0, "text_selected_point");
	}
	else
	{
		viewer.addText("Selected point: x=" + std::to_string(x) + ", y=" + std::to_string(y) + ", z=" + std::to_string(z), 40, 40, 0.0, 1.0, 0.0, "text_selected_point", 1);
	}
	// 删除上一次的簇
	viewer.removeAllPointClouds();
	addCloud(viewer, cloud_input, 1);
	// 重绘选中的点 变色 变大
	pcl::PointCloud<PointT>::Ptr selected_point_cloud(new pcl::PointCloud<PointT>);
	selected_point_cloud->push_back(selected_point);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red_color(selected_point_cloud, 255, 255, 255);
	// 更新可视化工具中的点云数据
	if (viewer.contains("selected_point")) {
		viewer.updatePointCloud(selected_point_cloud, red_color, "selected_point");
	}
	else
	{
		viewer.addPointCloud(selected_point_cloud, red_color, "selected_point");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "selected_point");
	}

	int idx_voxel = getNearestPointInCloud(selected_point, cloud_voxel);
	getPlane(cloud_voxel, idx_voxel,leaf_size*5.0f,3.0f);

}

// 鼠标单击事件回调函数
void pointPickingCallback_Dev(const pcl::visualization::PointPickingEvent& event, void* viewer_void) {

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
	if (z == 0 || idx <= 10)
	{
		std::cerr << "选点失败!请重新选点!" << idx << std::endl;
		if (viewer.contains("text_selected_point"))
		{
			viewer.updateText("Invalid point, Please select a new point ! ", 40, 40, 1.0, 1.0, 0.0, "text_selected_point");
		}
		else
		{
			viewer.addText("Invalid point, Please select a new point ! ", 40, 40, 1.0, 1.0, 0.0, "text_selected_point");
		}
		return;
	}
	std::cout << "选点：x=" << x << ", y=" << y << ", z=" << z << ", idx=" << idx << std::endl;
	if (viewer.contains("text_selected_point"))
	{
		viewer.updateText("Selected point: x=" + std::to_string(x) + ", y=" + std::to_string(y) + ", z=" + std::to_string(z), 40, 40, 0.0, 1.0, 0.0, "text_selected_point");
	}
	else
	{
		viewer.addText("Selected point: x=" + std::to_string(x) + ", y=" + std::to_string(y) + ", z=" + std::to_string(z), 40, 40, 0.0, 1.0, 0.0, "text_selected_point", 1);
	}
	// 删除上一次的簇
	viewer.removeAllPointClouds();
	addCloud(viewer, cloud_voxel, 1);
	addCloud(viewer, cloud_voxel, 2);
	addCloud(viewer, cloud_input, 3);
	//addCloud(viewer, cloud_input, 4);
	// 重绘选中的点 变色 变大
	pcl::PointCloud<PointT>::Ptr selected_point_cloud(new pcl::PointCloud<PointT>);
	selected_point_cloud->push_back(selected_point);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red_color(selected_point_cloud, 255, 255, 255);
	// 更新可视化工具中的点云数据
	if (viewer.contains("selected_point")) {
		viewer.updatePointCloud(selected_point_cloud, red_color, "selected_point");
	}
	else
	{
		viewer.addPointCloud(selected_point_cloud, red_color, "selected_point");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "selected_point");
	}
	// 如果没按计算键，则不计算
	if (!isDKeyPressed)return;

	float radius = 2.0f;
	float height = 60.0f;
	float leaf_size = 0.2f;
	pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>);
	pcl::IndicesPtr region_indices(new std::vector<int>);

	// 分割出圆柱区域
	//segmentRegionCylinder(cloud_input, cloud_cylinder, region_indices, selected_point, radius, height);
	//addCylinder(viewer, selected_point, radius, height);  // 圆柱区域可视化
	// 将区域内的点颜色改为绿色
	//for (size_t i = 0; i < region_indices->size(); ++i) {
	//	int idx = (*region_indices)[i];
	//	cloud_input->points[idx].r = 0;
	//	cloud_input->points[idx].g = 255;
	//	cloud_input->points[idx].b = 0;
	//}
	//viewer.updatePointCloud(cloud_input, "cloud1");
	//filterVoxelGrid(cloud_cylinder, cloud_voxel, leaf_size);
	// 区域拟合圆柱/平面
	//segmentCloud_Single(cloud_cylinder, region_indices);
	//segmentCloud(cloud_input,idx); // sac平面拟合
	//std::vector<pcl::PointIndices> cluster_indices;
	//euclideanClusterExtraction(cloud_input, cluster_indices); // 欧氏聚类
	//conditionalEuclidean(cloud_voxel, cloud_seg);
	//viewer.updatePointCloud(cloud_seg, "cloud1");
	//computeSelectedPointNeighborhood(cloud_input, idx);
	 //auto cluster = regionGrowing(cloud_input, idx, 0.1,0.5,0.5);

	getPlane(cloud_voxel, idx, leaf_size * 5.0f, 3.0f);

	isDKeyPressed = false; // 重置按键状态
}
// 初始化viewer
void initViewer(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<PointT>::Ptr& cloud, PointT selected_point)
{
	viewer.initCameraParameters();
	viewer.setBackgroundColor(0, 0, 0);
	viewer.setSize(1920,600);
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);
	viewer.setCameraPosition(centroid[0], centroid[1], centroid[2] + 10.0f, centroid[0], centroid[1], centroid[2], 0, 1, 0);
	// 注册选点事件回调
	viewer.registerPointPickingCallback(pointPickingCallback, (void*)&viewer);
	// 注册键盘事件回调
	viewer.registerKeyboardCallback(keyboardEventCallback, (void*)&viewer);
}
// 添加viewport (从1开始) count为视口数量
void addViewport(pcl::visualization::PCLVisualizer& viewer, int viewport =1, int count = 1, double r = 6.0/255.0, double g = 60.0/255.0, double b = 90.0/255.0)
{
	double x_min = (viewport - 1) * (1.0 / count);
	double x_max = viewport * (1.0 / count);
	viewer.createViewPort(x_min, 0.0, x_max, 1.0, viewport);
	viewer.setBackgroundColor(r, g, b, viewport);
	viewer.addCoordinateSystem(3, "coordinate", viewport);
	std::string x_label = "x_label_v" + std::to_string(viewport);
	std::string y_label = "y_label_v" + std::to_string(viewport);
	std::string z_label = "z_label_v" + std::to_string(viewport);
	viewer.addText3D("x", pcl::PointXYZ(11, 0, 0), 1.0, 1.0, 1.0, 1.0, x_label, viewport);
	viewer.addText3D("y", pcl::PointXYZ(0, 11, 0), 1.0, 1.0, 1.0, 1.0, y_label, viewport);
	viewer.addText3D("z", pcl::PointXYZ(0, 0, 11), 1.0, 1.0, 1.0, 1.0, z_label, viewport);
	// 标题
	viewer.addText("viewport " + std::to_string(viewport), 10, 10, std::to_string(viewport), viewport);
}

#pragma endregion
//// 开发api
//void getPlaneDev()
//{
//	PointT selected_point; 	//输入：一个三维点 
//
//	//readPcd("Cylinder.pcd", cloud_input); // Debug目录
//	//readPcd("../cloud/cat.pcd", cloud_input); // 隧道
//	//readPcd("../cloud/office_wall.pcd", cloud_input); // 办公室墙壁
//	readPcd("Cylinder.pcd", cloud_input); // 墩柱
//	//readPcd("../cloud/Scan Job 1 jb Export - Cloud.pcd", cloud_input); // 船厂
//
//	// 旋转点云
//	Eigen::Vector3f normal_planeXY(0.0f, 0.0f, 1.0f); // 旋转后的平面法向
//	//Eigen::Vector3f normal_incline(0.35, 0.22, 0.91); // 模拟非z法向 
//	Eigen::Vector3f normal_incline(0.09f, -0.16f, 0.98f); // 模拟非z法向 
//	Eigen::Matrix4f rotation = getRotationMatrix(normal_planeXY, normal_incline); // 求旋转矩阵
//	auto inv_rotation = rotation.transpose(); // 得逆旋转矩阵
//	//pcl::transformPointCloud(*cloud_input, *cloud_input, rotation); // 旋转
//
//	getVoxelCenters(cloud_input, leaf_size, cloud_voxel);
//	//filterVoxelGrid(cloud_input, cloud_voxel, leaf_size); // 体素化
//	// 求法线
//	//computeNormals(cloud_voxel, cloud_normals); // 计算法向
//
//	std::vector<pcl::PointIndices> clusters;
//	// 调用区域增长分割方法
//	//regionGrowingSegmentation(cloud_input, clusters);
//	//segmentCloud(cloud_input, point);
//
//	// PCL处理过程可视化
//
//	initViewer(viewer, cloud_voxel, selected_point);
//	int count = 3;
//	addViewport(viewer, 1, count);
//	addViewport(viewer, 2, count);
//	addViewport(viewer, 3, count);
//	//addViewport(viewer, 4, count);
//	//addViewport(viewer, 5, count);
//
//	addCloud(viewer, cloud_voxel, 1);
//	addCloud(viewer, cloud_voxel, 2);
//	addCloud(viewer, cloud_input, 3);
//	//addCloud(viewer, cloud_input, 4);
//
//	//viewer.spin();
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce(100);
//	}
//}

// 调试api
void getPlaneDebug()
{
	PointT selected_point; 	//输入：一个三维点 

	readPcd("Cylinder.pcd", cloud_input); // 墩柱
	//readPcd("../cloud/office_wall.pcd", cloud_input); // 办公室墙壁
	//readPcd("../cloud/Scan Job 1 jb Export - Cloud.pcd", cloud_input); // 船厂

	// 旋转点云
	//Eigen::Vector3f normal_planeXY(0.0f, 0.0f, 1.0f); // 旋转后的平面法向
	//Eigen::Vector3f normal_incline(0.35, 0.22, 0.91); // 模拟非z法向 
	//Eigen::Vector3f normal_incline(0.09f, -0.16f, 0.98f); // 模拟非z法向 
	//Eigen::Matrix4f rotation = getRotationMatrix(normal_planeXY, normal_incline); // 求旋转矩阵
	//auto inv_rotation = rotation.transpose(); // 得逆旋转矩阵
	//pcl::transformPointCloud(*cloud_input, *cloud_input, rotation); // 旋转

	getVoxelCenters(cloud_input, leaf_size, cloud_voxel);
	//filterVoxelGrid(cloud_input, cloud_voxel, 0.2); // 体素化
	// 求法线
	//computeNormals(cloud_voxel, cloud_normals); // 计算法向

	//std::vector<pcl::PointIndices> clusters;
	// 调用区域增长分割方法
	//regionGrowingSegmentation(cloud_input, clusters);
	//segmentCloud(cloud_input, point);

	// PCL处理过程可视化

	initViewer(viewer, cloud_voxel, selected_point);
	int count = 1;
	addViewport(viewer, 1, count);
	//addViewport(viewer, 2, count);
	//addViewport(viewer, 3, count);
	//addViewport(viewer, 4, count);
	//addViewport(viewer, 5, count);

	addCloud(viewer, cloud_input, 1);
	//addCloud(viewer, cloud_voxel, 2);
	//addCloud(viewer, cloud_input, 3);
	//addCloud(viewer, cloud_input, 4);

	//viewer.spin();
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
}
int main(int argc, char** argv)
{
	getPlaneDebug();
}
