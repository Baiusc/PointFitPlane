#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>//体素滤波
#include <pcl/features/normal_3d.h>//法线估计
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>//条件欧式聚类

typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;

void visualization2wd(pcl::PointCloud<PointTypeIO>::Ptr cloud1, pcl::PointCloud<PointTypeIO>::Ptr cloud2)
{
	//可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("条件欧式聚类分割"));
	//int v1, v2, v3, v4;
	//viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v1); //(Xmin,Ymin,Xmax,Ymax)设置窗口坐标
	//viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v2);
	//viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v3);
	//viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v4);

	int v1(0);
	MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	MView->setBackgroundColor(0, 0, 0, v1);
	pcl::visualization::PointCloudColorHandlerGenericField<PointTypeIO> fildColor(cloud1, "intensity"); // 按照intensity字段进行渲染
	MView->addText("before", 10, 10, "v1_text", v1);
	int v2(0);
	MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
	MView->setBackgroundColor(0, 0, 0, v2);
	pcl::visualization::PointCloudColorHandlerGenericField<PointTypeIO> fildColor2(cloud2, "intensity"); // 按照intensity字段进行渲染
	MView->addText("after", 10, 10, "v2_text", v2);
	MView->addPointCloud<PointTypeIO>(cloud1, fildColor, "sample cloud", v1);
	MView->addPointCloud<PointTypeIO>(cloud2, fildColor2, "cloud", v2);
	//MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
	//MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_boundary", v2);
	MView->addCoordinateSystem(0.1);
	//最后调用通过设置相机参数使用户从默认的角度和方向观察点云
	MView->initCameraParameters();
	MView->resetCamera();
	MView->spin();
	while (!MView->wasStopped())
	{
		MView->spinOnce(1000);
	/*	boost::this_thread::sleep(boost::posix_time::microseconds(1000));*/
	}
}

//条件欧式聚类分割
bool
enforceIntensitySimilarity(const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)//强度相似性
{
	if (fabs(point_a.intensity - point_b.intensity) < 5.0f)
		return (true);
	else
		return (false);
}

bool
enforceCurvatureSimilarity(const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)//曲率强度相似性
{
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(), point_b_normal = point_b.getNormalVector3fMap();
	if (fabs(point_a_normal.dot(point_b_normal)) < 0.05)//如果a点法线估计
		return (true);
	return (false);
}

bool
enforceCurvatureOrIntensitySimilarity(const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)//曲率强度相似性
{
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(), point_b_normal = point_b.getNormalVector3fMap();
	if (fabs(point_a.intensity - point_b.intensity) < 5.0f)//如果a点的密度-b点的密度<5
		return (true);
	if (fabs(point_a_normal.dot(point_b_normal)) < 0.05)//如果a点法线估计
		return (true);
	return (false);
}

bool
customRegionGrowing(const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)//区域生长
{
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(), point_b_normal = point_b.getNormalVector3fMap();
	if (squared_distance < 10000)
	{
		if (fabs(point_a.intensity - point_b.intensity) < 8.0f)//如果a点的密度-b点的密度<8
			return (true);
		if (fabs(point_a_normal.dot(point_b_normal)) < 0.06)//如果a点法线估计
			return (true);
	}
	else
	{
		if (fabs(point_a.intensity - point_b.intensity) < 3.0f)
			return (true);
	}
	return (false);
}

int
main(int argc, char** argv)
{
	// Data containers used
	pcl::PointCloud<PointTypeIO>::Ptr cloud_in(new pcl::PointCloud<PointTypeIO>), cloud_out(new pcl::PointCloud<PointTypeIO>);//创建cloud_in和cloud_out指针
	pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals(new pcl::PointCloud<PointTypeFull>);//创建法线估计
	pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters), small_clusters(new pcl::IndicesClusters), large_clusters(new pcl::IndicesClusters);//创建索引
	pcl::search::KdTree<PointTypeIO>::Ptr search_tree(new pcl::search::KdTree<PointTypeIO>);//创建搜索方式
	pcl::console::TicToc tt;

	// Load the input point cloud
	//加载点云数据
	std::cerr << "Loading...\n", tt.tic();
	//pcl::io::loadPCDFile("../cloud/Statues_4.pcd", *cloud_in);//输入点云
	pcl::io::loadPCDFile("../cloud/Statues_4.pcd", *cloud_in);//输入点云
	std::cerr << ">> Done: " << tt.toc() << " ms, " << cloud_in->points.size() << " points\n";//计算加载点云的时间，并在屏幕上打印输出

	// Downsample the cloud using a Voxel Grid class
	//对点云使用体素滤波
	std::cerr << "Downsampling...\n", tt.tic();//下采样
	pcl::VoxelGrid<PointTypeIO> vg;//创建体素滤波对象
	vg.setInputCloud(cloud_in);//设置输入点云
	vg.setLeafSize(80.0, 80.0, 80.0);//设置叶尺寸
	//vg.setLeafSize(0.02, 0.02, 0.02);//设置叶尺寸
	vg.setDownsampleAllData(true);//设置是否对所有点进行体素滤波
	vg.filter(*cloud_out);//执行滤波，并将结果保存在cloud_out中
	std::cerr << ">> Done: " << tt.toc() << " ms, " << cloud_out->points.size() << " points\n";//计算直通滤波所使用的时间

	// Set up a Normal Estimation class and merge data in cloud_with_normals
	//建立法线估计类，并将数据合并到cloud_with_normals
	std::cerr << "Computing normals...\n", tt.tic();
	pcl::copyPointCloud(*cloud_out, *cloud_with_normals);//复制点云
	pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne;//创建法线估计对象
	ne.setInputCloud(cloud_out);//设置输入点云
	ne.setSearchMethod(search_tree);//设置所搜方式
	ne.setRadiusSearch(300);//设置搜索半径大小
	//ne.setKSearch(50);//设置搜索半径大小
	ne.compute(*cloud_with_normals);//惊醒法线计算
	std::cerr << ">> Done: " << tt.toc() << " ms\n";//计算法线估计所使用的时间

	// Set up a Conditional Euclidean Clustering class
	//建立条件欧式聚类对象
	std::cerr << "Segmenting to clusters...\n", tt.tic();
	pcl::ConditionalEuclideanClustering<PointTypeFull> cec(true);//创建条件聚类分割对象，并进行初始化
	cec.setInputCloud(cloud_with_normals);//设置输入点云
	cec.setConditionFunction(&customRegionGrowing);//设置搜索函数
	cec.setClusterTolerance(500.0);//设置聚类参考点的所搜距离
	//cec.setClusterTolerance(0.2);//设置聚类参考点的所搜距离
	cec.setMinClusterSize(cloud_with_normals->points.size() / 1000);//设置过小聚类的标准
	cec.setMaxClusterSize(cloud_with_normals->points.size() / 5);//设置过大聚类的标准
	cec.segment(*clusters);//获取聚类的结果，分割结果保存在点云索引的向量中
	cec.getRemovedClusters(small_clusters, large_clusters);//获取无效尺寸的聚类
	std::cerr << ">> Done: " << tt.toc() << " ms\n";

	// Using the intensity channel for lazy visualization of the output
	//使用强度通道对输出进行延迟可视化
	for (int i = 0; i < small_clusters->size(); ++i)
		for (int j = 0; j < (*small_clusters)[i].indices.size(); ++j)
			cloud_out->points[(*small_clusters)[i].indices[j]].intensity = -2.0;
	for (int i = 0; i < large_clusters->size(); ++i)
		for (int j = 0; j < (*large_clusters)[i].indices.size(); ++j)
			cloud_out->points[(*large_clusters)[i].indices[j]].intensity = +10.0;
	for (int i = 0; i < clusters->size(); ++i)
	{
		int label = rand() % 8;
		for (int j = 0; j < (*clusters)[i].indices.size(); ++j)
			cloud_out->points[(*clusters)[i].indices[j]].intensity = label;
	}

	// Save the output point cloud
	std::cerr << "Saving...\n", tt.tic();
	pcl::io::savePCDFile("output.pcd", *cloud_out);//输出点云
	std::cerr << ">> Done: " << tt.toc() << " ms\n";//计算用时

	visualization2wd(cloud_in, cloud_out);

	return (0);
}
