#include "EfficientVolumeTopRepariXYZ.h"
namespace VolumeTopRepariXYZ
{
	// 定义全局变量来存储当前选中的点的坐标
	PointT selected_point;
	// 定义点云类型模板
	//typedef pcl::PointXYZRGB PointT;
	pcl::PointCloud<PointT>::Ptr cloud_src(new pcl::PointCloud<PointT>); //原点云
	pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients); //点法确定挖方底平面系数
	pcl::PointCloud<PointT>::Ptr cloud_offground(new pcl::PointCloud<PointT>); //非地面
	pcl::PointCloud<PointT>::Ptr cloud_top(new pcl::PointCloud<PointT>); //挖方顶面点云
	pcl::PointCloud<PointT>::Ptr voxel_top(new pcl::PointCloud<PointT>); //挖方顶面点云体素化
	pcl::PointCloud<pcl::PointXY>::Ptr top_repairX(new pcl::PointCloud<pcl::PointXY>); //二维修补X的结果
	pcl::PointCloud<pcl::PointXY>::Ptr top_repairY(new pcl::PointCloud<pcl::PointXY>); //二维修补Y的结果
	pcl::PointCloud<pcl::PointXY>::Ptr top_repairXY(new pcl::PointCloud<pcl::PointXY>); //二维修补XY的结果
	pcl::PointCloud<PointT>::Ptr voxel_bottom(new pcl::PointCloud<PointT>); //挖方顶面点云体素化
	pcl::PointCloud<PointT>::Ptr cloud_bottom(new pcl::PointCloud<PointT>); //挖方底面点云
	pcl::PointCloud<PointT>::Ptr grid_bottom(new pcl::PointCloud<PointT>); //网格体素化后的挖方底面点云
	pcl::PointCloud<PointT>::Ptr grid_bottom_repairX(new pcl::PointCloud<PointT>); //修补X的结果
	pcl::PointCloud<PointT>::Ptr grid_bottom_repairY(new pcl::PointCloud<PointT>); //修补Y的结果
	pcl::PointCloud<PointT>::Ptr grid_bottom_repair(new pcl::PointCloud<PointT>); //修补后的挖方底面网格点云
	pcl::PointCloud<PointT>::Ptr grid_top_repair(new pcl::PointCloud<PointT>); //修补后的挖方顶面网格点云


#pragma region PCL可视化
		// 鼠标单击事件回调函数
	void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* viewer_void) {
		std::cout << "[INFO] Point picking event occurred." << std::endl;
		// 检查是否获取到了选中的点
		if (event.getPointIndex() == -1)
			return;

		// 获取选中点的坐标
		float x, y, z;
		event.getPoint(x, y, z);
		selected_point.x = x;
		selected_point.y = y;
		selected_point.z = z;

		// 在终端输出选中点的坐标
		std::cout << "选点坐标：x=" << x << ", y=" << y << ", z=" << z << std::endl;
	}
	// 可视化三维点云
	void visualizePointCloud(pcl::PointCloud<PointT>::Ptr cloud, const std::string& window_name = "PCLviewer")
	{
		pcl::visualization::PCLVisualizer viewer(window_name); // 创建一个可视化窗口
		viewer.setBackgroundColor(0, 0, 0);
		pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(cloud, "z"); // 按Z轴值着色
		viewer.registerPointPickingCallback(pointPickingCallback, (void*)&viewer); // 注册点云选取回调函数
		viewer.addPointCloud<PointT>(cloud, color_handler, "point_cloud");  // 添加点云
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud"); // 点大小
		// 启动可视化循环
		while (!viewer.wasStopped()) {
			viewer.spinOnce(100);
		}
	}
	// 可视化2个二维xy点云
	void visualizePointCloud2d(pcl::PointCloud<pcl::PointXY>::Ptr cloud1, pcl::PointCloud<pcl::PointXY>::Ptr cloud2, const std::string& window_name = "PCLviewer_2d_2cloud")
	{
		// 创建3D点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_3d(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_3d(new pcl::PointCloud<pcl::PointXYZ>);

		// 为cloud1中的点分配z值
		for (const auto& point : cloud1->points) {
			cloud1_3d->push_back(pcl::PointXYZ(point.x, point.y, 0));
		}

		// 为cloud2中的点分配z值
		for (const auto& point : cloud2->points) {
			cloud2_3d->push_back(pcl::PointXYZ(point.x, point.y, 1));
		}

		// 创建可视化窗口
		pcl::visualization::PCLVisualizer viewer(window_name);
		viewer.setBackgroundColor(0, 0, 0);

		// 向查看器添加点云
		viewer.addPointCloud<pcl::PointXYZ>(cloud1_3d, "cloud1");
		viewer.addPointCloud<pcl::PointXYZ>(cloud2_3d, "cloud2");

		// 设置点大小
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud2");

		// 设置颜色
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud1");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "cloud2");

		// 启动可视化循环
		while (!viewer.wasStopped()) {
			viewer.spinOnce(100);
		}
	}
	void visualizePointCloud2d(pcl::PointCloud<pcl::PointXY>::Ptr cloud, const std::string& window_name = "PCLviewer_2d")
	{
		// 创建3D点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3d(new pcl::PointCloud<pcl::PointXYZ>);

		// 为cloud中的点分配z值
		for (const auto& point : cloud->points) {
			cloud_3d->push_back(pcl::PointXYZ(point.x, point.y, 0));
		}

		// 创建可视化窗口
		pcl::visualization::PCLVisualizer viewer(window_name);
		viewer.setBackgroundColor(0, 0, 0);

		// 向查看器添加点云
		viewer.addPointCloud<pcl::PointXYZ>(cloud_3d, "cloud");

		// 设置点大小
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud");

		// 启动可视化循环
		while (!viewer.wasStopped()) {
			viewer.spinOnce(100);
		}
	}








#pragma endregion

#pragma region 可复用的方法

	// 读pcd
	void readPcd(const std::string& filename, pcl::PointCloud<PointT>::Ptr& cloud)
	{
		pcl::PCDReader reader;
		reader.read(filename, *cloud);
		std::cout << "cloud_src has: " << cloud->points.size() << " data points." << std::endl;
	}
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
		pcl::search::KdTree<pcl::PointXY>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXY>);
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
		auto start = std::clock();
		// 创建滤波器对象
		pcl::ProjectInliers<PointT> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setInputCloud(cloud);
		proj.setModelCoefficients(plane_coefficients);
		proj.filter(*cloud_projected);
		auto end_1 = std::clock();
		std::cerr << "投影生成挖方底面，耗时：" << std::difftime(end_1, start) << "ms" << std::endl;
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
		auto start = std::clock();
		// 获取挖方底面点云的边界框
		PointT min_pt, max_pt;
		pcl::getMinMax3D(*cloud_bottom, min_pt, max_pt);
		auto end_1 = std::clock();
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
		auto end_2 = std::clock();

		// 创建一个八叉树对象
		pcl::octree::OctreePointCloudSearch<PointT> octree(leaf_size); // 设置体素分辨率
		// 设置输入点云数据
		octree.setInputCloud(cloud_bottom);
		// 构建八叉树索引
		octree.addPointsFromInputCloud();
		// 进行体素内搜索
		std::vector<int> nearest_indices;
		for (size_t i = 0; i < grid_cloud->size(); ++i) {
			const auto& point = (*grid_cloud)[i];
			if (octree.voxelSearch(point, nearest_indices) > 0) {
				// 网格点的底
				PointT bottom_point(point.x, point.y, point.z);
				grid_bottom->push_back(bottom_point);
			}
		}
		auto end_3 = std::clock();
		std::cerr << "获取挖方底面点云的边界框耗时：" << std::difftime(end_1, start) << "ms" << std::endl;
		std::cerr << "创建底面二维网格耗时：" << std::difftime(end_2, start) << "ms" << std::endl;
		std::cerr << "创建底面网格，octree体素搜索耗时：" << std::difftime(end_3, start) << "ms" << std::endl;

		std::cout << "\r\n网格大小: " << num_x << "列 × " << num_y << "行 " << std::endl;
		std::cout << "分辨率: " << leaf_size << "m" << std::endl;
		std::cout << "X-范围: " << min_pt.x << "m 到 " << max_pt.x << "m" << std::endl;
		std::cout << "Y-范围: " << min_pt.y << "m 到 " << max_pt.y << "m" << std::endl;
		std::cout << "Z-范围: " << min_pt.z << "m 到 " << max_pt.z << "m" << std::endl;
	}

	/// <summary>
	/// 在二维平面plane_coefficients上，对每一个X坐标，取Ymin和Ymax，在其间按leaf_size进行插值修补  
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
				for (float y = minY; y < maxY; y += interval) {
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
	/// 在二维平面plane_coefficients上，对每一个Y坐标，取Xmin和Xmax，在其间按leaf_size进行插值修补  
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
	/// 对每一个Y坐标，取Xmin和Xmax，在其间按leaf_size进行插X值修补，点z坐标以cloud内搜索得到的邻居作为参照
	/// </summary>
	/// <param name="cloud">输入点云</param>
	/// <param name="leaf_size">分辨率</param>
	/// <param name="cloud_x">输出：插X值修补后的点云</param>
	void repairX_3d(const pcl::PointCloud<PointT>::Ptr& cloud,
		float leaf_size,
		pcl::PointCloud<PointT>::Ptr& cloud_x)
	{
		// 按y坐标从小到大对点云进行排序
		auto compareYCoordinate = [](const PointT& p1, const PointT& p2)
		{
			return p1.y < p2.y;
		};
		std::sort(cloud->points.begin(), cloud->points.end(), compareYCoordinate);
		// 创建二维点云
		pcl::PointCloud<pcl::PointXY>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXY>);
		// 将点云的每个点的 XY 坐标提取到二维点云中
		cloud_2d->points.resize(cloud->size());
		for (size_t i = 0; i < cloud->size(); ++i) {
			cloud_2d->points[i].x = cloud->points[i].x;
			cloud_2d->points[i].y = cloud->points[i].y;
		}
		// 二维点云的kd树对象
		pcl::search::KdTree<pcl::PointXY>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXY>);
		int k = 1;
		float search_radius = leaf_size/2;
		std::vector<int> nearest_indices;
		std::vector<float> nearest_distances;
		kdtree->setInputCloud(cloud_2d); // 设置输入点云数据
		kdtree->setSortedResults(false); // 设置是否对搜索结果进行排序
		// for循环内的中间变量
		float current_y = cloud->points[0].y;
		float minX = cloud->points[0].x;
		float maxX = cloud->points[0].x;
		// 遍历按y排序后的点云
		for (std::size_t idx = 0; idx < cloud->points.size(); ++idx)
		{
			const PointT& point = cloud->points[idx];
			// 当y坐标发生变化时
			if (!(std::abs(point.y - current_y) < 0.00001))
			{
				// 根据leaf_size、minX、maxX，取等间距补点
				float interval = leaf_size;
				for (float x = minX; x < maxX; x += interval) {
					PointT point_add;
					point_add.x = x;  // x按interval等间距插值修补
					point_add.y = current_y; // y等于当前列的y值
					pcl::PointXY point_2d;
					point_2d.x = x;
					point_2d.y = current_y;
					// 在cloud中nearestKSearch搜索最近的1个点，取其z值
					kdtree->nearestKSearch(point_2d, k, nearest_indices, nearest_distances);
					point_add.z = cloud->points[nearest_indices[0]].z;
					cloud_x->points.push_back(point_add);
				}
				current_y = point.y;
				minX = point.x;
				maxX = point.x;
			}
			// 若y坐标没发生变化
			minX = std::min(minX, point.x); // 更新最值
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
					pcl::PointXY point_2d;
					point_2d.x = x;
					point_2d.y = current_y;
					// 在cloud中nearestKSearch搜索最近的1个点，取其z值
					kdtree->nearestKSearch(point_2d, k, nearest_indices, nearest_distances);
					point_add.z = cloud->points[nearest_indices[0]].z;
					cloud_x->points.push_back(point_add);
				}
			}
		}
	}


	void repairY_3d(const pcl::PointCloud<PointT>::Ptr& cloud,
		float leaf_size,
		pcl::PointCloud<PointT>::Ptr& cloud_y)
	{
		// 按照 x 坐标从小到大对点云进行排序
		auto compareXCoordinate = [](const PointT& p1, const PointT& p2)
		{
			return p1.x < p2.x;
		};
		std::sort(cloud->points.begin(), cloud->points.end(), compareXCoordinate);

		// 创建二维点云
		pcl::PointCloud<pcl::PointXY>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXY>);

		// 将点云的每个点的 XY 坐标提取到二维点云中
		cloud_2d->points.resize(cloud->size());
		for (size_t i = 0; i < cloud->size(); ++i) {
			cloud_2d->points[i].x = cloud->points[i].x;
			cloud_2d->points[i].y = cloud->points[i].y;
		}

		// 二维点云的kd树对象
		pcl::search::KdTree<pcl::PointXY>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXY>);
		int k = 1;
		float search_radius = leaf_size / 2;
		std::vector<int> nearest_indices;
		std::vector<float> nearest_distances;
		kdtree->setInputCloud(cloud_2d); // 设置输入点云数据
		kdtree->setSortedResults(false); // 设置是否对搜索结果进行排序

		// for循环内的中间变量
		float current_x = cloud->points[0].x;
		float minY = cloud->points[0].y;
		float maxY = cloud->points[0].y;

		// 遍历按照 x 排序后的点云
		for (std::size_t idx = 0; idx < cloud->points.size(); ++idx)
		{
			const PointT& point = cloud->points[idx];

			// 当 x 坐标发生变化时
			if (!(std::abs(point.x - current_x) < 0.00001))
			{
				// 根据 leaf_size、minY、maxY，取等间距补点
				float interval = leaf_size;
				for (float y = minY; y < maxY; y += interval) {
					PointT point_add;
					point_add.x = current_x;
					point_add.y = y;  // y 按 interval 等间距插值修补
					pcl::PointXY point_2d;
					point_2d.x = current_x;
					point_2d.y = y;
					// 在 cloud 中 nearestKSearch 搜索最近的 1 个点，取其 z 值
					kdtree->nearestKSearch(point_2d, k, nearest_indices, nearest_distances);
					point_add.z = cloud->points[nearest_indices[0]].z;
					cloud_y->points.push_back(point_add);
				}
				current_x = point.x;
				minY = point.y;
				maxY = point.y;
			}

			// 若 x 坐标没发生变化
			minY = std::min(minY, point.y); // 更新最值
			maxY = std::max(maxY, point.y);

			// 如果是最后一个 x 坐标
			if (idx == cloud->points.size() - 1)
			{
				// 对最后一个 x 坐标进行补点
				float interval = leaf_size;
				for (float y = minY; y < maxY; y += interval) {
					PointT point_add;
					point_add.x = current_x;
					point_add.y = y;
					pcl::PointXY point_2d;
					point_2d.x = current_x;
					point_2d.y = y;
					// 在 cloud 中 nearestKSearch 搜索最近的 1 个点，取其 z 值
					kdtree->nearestKSearch(point_2d, k, nearest_indices, nearest_distances);
					point_add.z = cloud->points[nearest_indices[0]].z;
					cloud_y->points.push_back(point_add);
				}
			}
		}
	}

	/// <summary>
	/// 先化为二维点云。然后对每一个Y坐标，取Xmin和Xmax，在其间按leaf_size进行插X值修补
	/// </summary>
	/// <param name="cloud">输入点云</param>
	/// <param name="leaf_size">分辨率</param>
	/// <param name="cloud_x">输出：插X值修补后的二维点云</param>
	void repairX_2d(const pcl::PointCloud<pcl::PointXY>::Ptr& cloud_2d,
		float leaf_size,
		pcl::PointCloud<pcl::PointXY>::Ptr& cloud_x)
	{
		// 按y坐标从小到大对二维点云进行排序
		auto compareYCoordinate = [](const pcl::PointXY& p1, const pcl::PointXY& p2)
		{
			return p1.y < p2.y;
		};
		std::sort(cloud_2d->points.begin(), cloud_2d->points.end(), compareYCoordinate);
		// 二维点云的kd树对象
		pcl::search::KdTree<pcl::PointXY>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXY>);
		int k = 1;
		float search_radius = leaf_size / 2;
		std::vector<int> nearest_indices;
		std::vector<float> nearest_distances;
		kdtree->setInputCloud(cloud_2d); // 设置输入点云数据
		kdtree->setSortedResults(false); // 设置是否对搜索结果进行排序
		// for循环内的中间变量
		float current_y = cloud_2d->points[0].y;
		float minX = cloud_2d->points[0].x;
		float maxX = cloud_2d->points[0].x;
		// 遍历按y排序后的点云
		for (std::size_t idx = 0; idx < cloud_2d->points.size(); ++idx)
		{
			const pcl::PointXY& point = cloud_2d->points[idx];
			// 当y坐标发生变化时
			if (!(std::abs(point.y - current_y) < 0.00001))
			{
				// 根据leaf_size、minX、maxX，取等间距补点
				float interval = leaf_size;
				for (float x = minX; x < maxX; x += interval) {
					pcl::PointXY point_add;
					point_add.x = x;  // x按interval等间距插值修补
					point_add.y = current_y; // y等于当前列的y值
					cloud_x->points.push_back(point_add);
				}
				current_y = point.y;
				minX = point.x;
				maxX = point.x;
			}
			// 若y坐标没发生变化
			minX = std::min(minX, point.x); // 更新最值
			maxX = std::max(maxX, point.x);
			// 如果是最后一个y坐标
			if (idx == cloud_2d->points.size() - 1)
			{
				// 对最后一个y坐标进行补点
				float interval = leaf_size;
				for (float x = minX; x < maxX; x += interval) {
					pcl::PointXY point_add;
					point_add.x = x;  // x按interval等间距插值修补
					point_add.y = current_y; // y等于当前列的y值
					cloud_x->points.push_back(point_add);
				}
			}
		}
	}
	void repairY_2d(const pcl::PointCloud<pcl::PointXY>::Ptr& cloud_2d,
		float leaf_size,
		pcl::PointCloud<pcl::PointXY>::Ptr& cloud_y)
	{
		// 按x坐标从小到大对二维点云进行排序
		auto compareXCoordinate = [](const pcl::PointXY& p1, const pcl::PointXY& p2)
		{
			return p1.x < p2.x;
		};
		std::sort(cloud_2d->points.begin(), cloud_2d->points.end(), compareXCoordinate);
		// 二维点云的kd树对象
		pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXY>);; // 创建一个 Kd 树对象
		//pcl::search::KdTree<pcl::PointXY>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXY>);
		int k = 1;
		float search_radius = leaf_size / 2;
		std::vector<int> nearest_indices;
		std::vector<float> nearest_distances;
		kdtree->setInputCloud(cloud_2d); // 设置输入点云数据
		kdtree->setSortedResults(false); // 设置是否对搜索结果进行排序
		// for循环内的中间变量
		float current_x = cloud_2d->points[0].x;
		float minY = cloud_2d->points[0].y;
		float maxY = cloud_2d->points[0].y;
		// 遍历按x排序后的点云
		for (std::size_t idx = 0; idx < cloud_2d->points.size(); ++idx)
		{
			const pcl::PointXY& point = cloud_2d->points[idx];
			// 当x坐标发生变化时
			if (!(std::abs(point.x - current_x) < 0.00001))
			{
				// 根据leaf_size、minY、maxY，取等间距补点
				float interval = leaf_size;
				for (float y = minY; y < maxY; y += interval) {
					pcl::PointXY point_add;
					point_add.x = current_x;  // x等于当前行的x值
					point_add.y = y; // y按interval等间距插值修补
					cloud_y->points.push_back(point_add);
				}
				current_x = point.x;
				minY = point.y;
				maxY = point.y;
			}
			// 若x坐标没发生变化
			minY = std::min(minY, point.y); // 更新最值
			maxY = std::max(maxY, point.y);
			// 如果是最后一个x坐标
			if (idx == cloud_2d->points.size() - 1)
			{
				// 对最后一个x坐标进行补点
				float interval = leaf_size;
				for (float y = minY; y < maxY; y += interval) {
					pcl::PointXY point_add;
					point_add.x = current_x;  // x等于当前行的x值
					point_add.y = y; // y按interval等间距插值修补
					cloud_y->points.push_back(point_add);
				}
			}
		}
	}
	// 修补z坐标
	void repairZ(const pcl::PointCloud<pcl::PointXY>::Ptr& repair_xy,
		const pcl::PointCloud<PointT>::Ptr& voxel_top,
		const pcl::PointCloud<pcl::PointXY>::Ptr& voxel_top_2d,
		float leaf_size,
		pcl::PointCloud<PointT>::Ptr& cloud_xyz)
	{
		pcl::search::KdTree<pcl::PointXY>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXY>);
		//pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXY>);; // 创建一个 Kd 树对象
		kdtree->setInputCloud(voxel_top_2d); // 设置输入点云数据
		float search_radius = leaf_size / 2; // 设置rs搜索半径
		int k = 1; // 设置nk搜索点数
		std::vector<int> rs_indices; // 用于存储最近邻点的索引
		std::vector<int> nk_indices(k); // 用于存储最近邻点的索引
		std::vector<float> rs_distances; // 用于存储最近邻点距离的向量
		std::vector<float> nk_distances(k); // 用于存储最近邻点距离的向量

		for (size_t i = 0; i < repair_xy->size(); ++i) {
			const auto& point = (*repair_xy)[i]; // xy值修补后的二维点
			PointT pt_repair; // z值修补后的三维点
			if (kdtree->radiusSearch(point, search_radius, rs_indices, rs_distances) > 0)
			{
				pt_repair.x = point.x;
				pt_repair.y = point.y;
				// 初始化最大z值和对应的索引
				float max_z_value = std::abs(voxel_top->points[rs_indices[0]].z);
				int max_z_index = rs_indices[0];

				// 遍历rs_indices找出z绝对值最大的点
				for (size_t i = 1; i < rs_indices.size(); ++i)
				{
					float current_z_value = std::abs(voxel_top->points[rs_indices[i]].z);
					if (current_z_value > max_z_value)
					{
						max_z_value = current_z_value;
						max_z_index = rs_indices[i];
					}
				}

				// 将z绝对值最大的点的z值赋给pt_repair.z
				pt_repair.z = voxel_top->points[max_z_index].z;
				//pt_repair.z = voxel_top->points[rs_indices[0]].z;
			}
			else
			{
				//kdtree->nearestKSearch(point, k, nk_indices, nk_distances);
				//pt_repair.x = point.x;
				//pt_repair.y = point.y;
				//pt_repair.z = voxel_top->points[nk_indices[0]].z;
			}
			cloud_xyz->push_back(pt_repair);


		}
	}

	void repairVoxelTop_3d(const pcl::PointCloud<PointT>::Ptr& grid,
		float leaf_size,
		pcl::PointCloud<PointT>::Ptr& grid_repair)
	{
		repairX_3d(grid, leaf_size, grid_bottom_repairX);
		repairY_3d(grid, leaf_size, grid_bottom_repairY);
		// 取cloud_x和cloud_y的交集
		getIntersection_NK(grid_bottom_repairX, grid_bottom_repairY, grid_repair);
		std::cout << "grid_bottom_repairX size: " << grid_bottom_repairX->size() << std::endl;
		std::cout << "grid_bottom_repairY size: " << grid_bottom_repairY->size() << std::endl;
		std::cout << "Intersection_NK size: " << grid_repair->size() << std::endl;
	}
	void repairVoxelTop_2d(const pcl::PointCloud<PointT>::Ptr& voxel_top,
		float leaf_size,
		pcl::PointCloud<PointT>::Ptr& top_repairXYZ)
	{
		// 创建二维点云
		pcl::PointCloud<pcl::PointXY>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXY>);
		// 将点云的每个点的 XY 坐标提取到二维点云中
		cloud_2d->points.resize(voxel_top->size());
		cloud_2d->width = voxel_top->size();
		cloud_2d->height = 1;
		cloud_2d->is_dense = true;
		for (size_t i = 0; i < voxel_top->size(); ++i) {
			cloud_2d->points[i].x = voxel_top->points[i].x;
			cloud_2d->points[i].y = voxel_top->points[i].y;
		}
		repairX_2d(cloud_2d, leaf_size, top_repairX);
		repairY_2d(cloud_2d, leaf_size, top_repairY);

		// 取交集
		getIntersection_2d(top_repairX, top_repairY, top_repairXY);
		// 在top_repairXY基础上修补z
		repairZ(top_repairXY, voxel_top, cloud_2d,leaf_size, top_repairXYZ);
		visualizePointCloud(top_repairXYZ);
		std::cout << "top_repairX size: " << top_repairX->size() << std::endl;
		std::cout << "top_repairY size: " << top_repairY->size() << std::endl;
		std::cout << "top_repairXY size: " << top_repairXY->size() << std::endl;
		std::cout << "top_repairXYZ size: " << top_repairXYZ->size() << std::endl;
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
		int k = 3;
		std::vector<int> nearest_indices;
		std::vector<float> nearest_distances;
		kdtree->setInputCloud(cloud_bottom); // 设置输入点云数据
		kdtree->setSortedResults(false); // 设置是否对搜索结果进行排序

		// 进行kdtree半径搜索
		float search_radius = leaf_size / 2; // 设置搜索半径
		for (size_t i = 0; i < grid_bottom->size(); ++i) {
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
				PointT top_point = calcTopPoint(point, plane_coefficients, max_distance);

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

		//// 创建一个八叉树对象
		//pcl::octree::OctreePointCloudSearch<PointT> octree(leaf_size); // 设置体素分辨率
		//// 设置输入点云数据
		//octree.setInputCloud(cloud_bottom);
		//// 构建八叉树索引
		//octree.addPointsFromInputCloud();
		//// 进行体素内搜索
		//std::vector<int> oc_nearest_indices;
		//std::vector<float> oc_sqr_dists;
		//for (size_t i = 0; i < grid_bottom->size(); ++i) {
		//	const auto& point = (*grid_bottom)[i];
		//	if (octree.voxelSearch(point, oc_nearest_indices) > 0) {
		//		// 计算(*cloud_top)[nearest_indices[i]]点到平面plane_coefficients的距离（有正负）绝对值最大的distance原始值
		//		float max_abs_distance = 0.0f;
		//		float max_distance = 0.0f;
		//		for (size_t i = 0; i < oc_nearest_indices.size(); i++) {
		//			float distance = calcPointToPlaneDistance((*cloud_top)[oc_nearest_indices[i]], plane_coefficients);
		//			if (abs(distance) > max_abs_distance) {
		//				max_abs_distance = abs(distance);
		//				max_distance = distance;
		//			}
		//		}
		//		// 顶面网格的对应点
		//		PointT top_point = calcTopPoint(point, plane_coefficients, max_distance);
		//		grid_top->push_back(top_point);
		//	}
		//	else
		//	{
		//		kdtree->nearestKSearch(point, k, nearest_indices, nearest_distances);
		//		float max_abs_distance = 0.0f;
		//		float max_distance = 0.0f;
		//		for (size_t i = 0; i < nearest_indices.size(); i++) {
		//			float distance = calcPointToPlaneDistance((*cloud_top)[nearest_indices[i]], plane_coefficients);
		//			if (abs(distance) > max_abs_distance) {
		//				max_abs_distance = abs(distance);
		//				max_distance = distance;
		//			}
		//		}
		//		// 顶面网格的对应点
		//		PointT top_point = calcTopPoint(point, plane_coefficients, max_distance);
		//		grid_top->push_back(top_point);
		//	}
		//}
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
		//renderer->AddActor(axes);

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
	/// 创建立方体群，并打开vtk窗口显示
	/// </summary>
	/// <param name="cloud_top"></param>
	/// <param name="cloud_bottom"></param>
	/// <param name="side_length"></param>
	/// <returns></returns>
	static void createCubes(const pcl::PointCloud<PointT>::Ptr cloud_top, const pcl::PointCloud<PointT>::Ptr cloud_bottom, const pcl::ModelCoefficients::Ptr plane_coefficients, float side_length) {
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
			float cube_height = calcPointToPlaneDistance(pt_top, plane_coefficients);
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

#pragma endregion


	void getVoxelCenters(pcl::PointCloud<PointT>::Ptr cloud_src, float leaf_size, pcl::PointCloud<PointT>::Ptr voxel_cloud) {

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

	}
	void visualizeVoxelCenters(pcl::PointCloud<PointT>::Ptr cloud_src,float leaf_size) {
		auto start = std::clock();
		// 创建一个八叉树对象
		pcl::octree::OctreePointCloud<PointT> octree(leaf_size); // 设置体素分辨率
		// 设置输入点云数据
		octree.setInputCloud(cloud_src);
		// 构建八叉树索引
		octree.addPointsFromInputCloud();
		auto end_1 = std::clock();
		// 获取被占用体素中心点
		std::vector<PointT, Eigen::aligned_allocator<PointT>> voxel_centers;
		octree.getOccupiedVoxelCenters(voxel_centers);
		auto end_2 = std::clock();
		// 创建一个点云对象
		pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);
		// 将体素中心点添加到点云中
		for (const auto& point : voxel_centers) {
			voxel_cloud->push_back(point);
		}

		auto end_3 = std::clock();
		std::cerr << "构建八叉树，耗时：" << std::difftime(end_1, start) << "ms" << std::endl;
		std::cerr << "获取被占用体素中心点，耗时：" << std::difftime(end_2, end_1) << "ms" << std::endl;
		std::cerr << "创建体素点云对象，耗时：" << std::difftime(end_3, end_2) << "ms" << std::endl;

		// 创建一个可视化窗口
		pcl::visualization::PCLVisualizer viewer("666");
		//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer.setBackgroundColor(0, 0, 0);

		// 添加点云
		pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(voxel_cloud, "z");
		viewer.addPointCloud<PointT>(voxel_cloud, color_handler, "voxel_cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "voxel_cloud");
		// 启动可视化循环
		while (!viewer.wasStopped()) {
			viewer.spinOnce(100);
		}
	}



	// 挖方体积计算入口
	VolumeResult getEfficientVolume(pcl::PointCloud<PointT>::Ptr cloud_src, Eigen::Vector3f normal, PointT point, float leaf_size)
	{
		VolumeResult result;
		normal = normal.normalized(); // 单位化平面法向量
		calcPlaneCoefficients(point, normal, plane_coefficients); //定义的平面的方程
		auto start = std::clock();
		// CSF地面分割（得到挖方的顶面）
		csf_ground_segmentation(cloud_src, cloud_top, cloud_offground);
		auto end_1 = std::clock();
		// 顶面octree体素化
		getVoxelCenters(cloud_top, leaf_size, voxel_top);
		//visualizePointCloud(voxel_top,"voxel_top");
		auto end_2 = std::clock();
		// 顶面修补
		repairVoxelTop_2d(voxel_top, leaf_size, grid_top_repair);
		//repairVoxelTop_3d(voxel_top, leaf_size, grid_top_repair);
		auto end_3 = std::clock();
		//visualizePointCloud(grid_top_repair,"grid_top_repair");
		// 投影生成挖方底面
		projectPointCloudToPlane(grid_top_repair, plane_coefficients, grid_bottom_repair); //2.7s
		//visualizePointCloud(grid_bottom_repair,"grid_bottom_repair");
		auto end_4 = std::clock();
		// 计算体积
		calcCubeClusterVolume(grid_bottom_repair, grid_top_repair, leaf_size);
		auto end_5 = std::clock();
		// 创建立方体群
		createCubes(grid_top_repair, grid_bottom_repair, plane_coefficients, leaf_size * 0.9);
		auto end_6 = std::clock();
		std::cerr << "CSF，耗时：" << std::difftime(end_1, start) << "ms" << std::endl;
		std::cerr << "顶面octree体素化，耗时：" << std::difftime(end_2, end_1) << "ms" << std::endl;
		std::cerr << "顶面3d修补：" << std::difftime(end_3, end_2) << "ms" << std::endl;
		std::cerr << "投影生成挖方底面，耗时：" << std::difftime(end_4, end_3) << "ms" << std::endl;
		std::cerr << "计算体积，耗时：" << std::difftime(end_5, end_4) << "ms" << std::endl;
		std::cerr << "创建立方体群，耗时：" << std::difftime(end_6, end_5) << "ms" << std::endl;
		return result;
	}




}

