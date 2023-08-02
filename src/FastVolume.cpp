#include "FastVolume.h"
namespace FastVolume
{
	__declspec(dllexport)
	// 定义全局变量来存储当前选中的点的坐标
	PointT selected_point;
	// 定义点云类型模板
	//typedef pcl::PointXYZRGB PointT;
	pcl::PointCloud<PointT>::Ptr cloud_src(new pcl::PointCloud<PointT>); //原点云
	pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients); //点法确定挖方底平面系数
	pcl::PointCloud<PointT>::Ptr cloud_offground(new pcl::PointCloud<PointT>); //非地面
	pcl::PointCloud<PointT>::Ptr cloud_top(new pcl::PointCloud<PointT>); //挖方顶面点云 （隧道地面）
	pcl::PointCloud<PointT>::Ptr top_rotated(new pcl::PointCloud<PointT>); // 旋转后的顶面
	pcl::PointCloud<PointT>::Ptr top_rotated_voxel(new pcl::PointCloud<PointT>); //旋转后体素化的顶面
	pcl::PointCloud<PointT>::Ptr top_rotated_repair(new pcl::PointCloud<PointT>); //旋转后修补的顶面
	pcl::PointCloud<PointT>::Ptr top_real_repair(new pcl::PointCloud<PointT>); //真实的修补顶面
	pcl::PointCloud<PointT>::Ptr bottom_rotated_repair(new pcl::PointCloud<PointT>); //旋转后修补的底面
	pcl::ModelCoefficients::Ptr rotated_plane_coefficients(new pcl::ModelCoefficients); //点法确定挖方底平面系数
	pcl::PointCloud<PointT>::Ptr bottom_real_repair(new pcl::PointCloud<PointT>); //真实的修补底面

#pragma region 读写操作和格式转换
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

	/// <summary>
	/// CSF地面分割
	/// </summary>
	/// <param name="pclCloud">pcl原点云</param>
	/// <param name="segCloud">pcl地面点云</param>
	void csf_ground_segmentation(const pcl::PointCloud<PointT>::Ptr pclCloud, const pcl::PointCloud<PointT>::Ptr cloud_ground, const pcl::PointCloud<PointT>::Ptr cloud_offground)
	{
		auto time_start = std::clock();
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
		cloud_ground->width = cloud_ground->size();
		for (const int& i : offGroundIndexes)
		{
			PointT pclPoint;
			pclPoint.x = csfCloud[i].x;
			pclPoint.y = csfCloud[i].y; // 将 csf 点云的 z 坐标赋值给 pcl 点云的 y 坐标
			pclPoint.z = csfCloud[i].z; // 将 csf 点云的 y 坐标取负并赋值给 pcl 点云的 z 坐标
			cloud_offground->points.push_back(pclPoint);
		}
		cloud_offground->width = cloud_offground->size();
		//pcl::io::savePCDFileBinary("../file/ground.pcd", *cloud_ground);
		//save2txt("../file/ground.txt", cloud_ground);

		auto time_end = std::clock();
		std::cerr << "\r\nCSF，耗时：" << std::difftime(time_end, time_start) << "ms" << std::endl;
	}

	/// <summary>
	/// 计算立方体群的体积，只打印结果不输出
	/// </summary>
	/// <param name="grid_bottom">底面</param>
	/// <param name="grid_top">顶面</param>
	/// <param name="plane_coefficients">底面方程系数</param>
	/// <param name="leaf_size">分辨率</param>
	void calcCubeClusterVolume(const pcl::PointCloud<PointT>::Ptr& grid_bottom,
		const pcl::PointCloud<PointT>::Ptr& grid_top,
		pcl::ModelCoefficients::Ptr plane_coefficients,
		float leaf_size,
		VolumeResult result)
	{
		auto time_start = std::clock();

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
		result.positive_volume = leaf_size * leaf_size * positive_height_sum; //挖方的体积
		result.negative_volume = leaf_size * leaf_size * std::abs(negative_height_sum); //填方的体积
		result.total_volume = result.positive_volume + result.negative_volume; //挖方加上填方的体积
		result.diff_volume = result.positive_volume - result.negative_volume; //挖方减去填方的体积
		result.positive_area = leaf_size * leaf_size * positive_point_count; //平面面积（正部分）
		result.negative_area = leaf_size * leaf_size * negative_point_count; //平面面积（负部分）
		result.total_area = leaf_size * leaf_size * total_point_count; //平面面积（全部）
		// 打印
		auto time_end = std::clock();
		std::cerr << "计算体积，耗时：" << std::difftime(time_end, time_start) << "ms" << std::endl;
		std::cout << "\r\n挖方与填方" << std::endl;;
		std::cout << "正的体积[挖方]：" << result.positive_volume << std::endl;
		std::cout << "负的体积[填方]：" << result.negative_volume << std::endl;
		std::cout << "挖方减去填方：" << result.diff_volume << std::endl;
		std::cout << "挖方加上填方：" << result.total_volume << std::endl;
		std::cout << "\r\n面积" << std::endl;;
		std::cout << "平面面积（正部分）：" << result.positive_area << std::endl;
		std::cout << "平面面积（负部分）：" << result.negative_area << std::endl;
		std::cout << "全部的平面面积：" << result.total_area << std::endl;


	}

	/// <summary>
	/// 先化为二维点云。然后对每一个Y坐标，取Xmin和Xmax，在其间按leaf_size进行插X值修补
	/// </summary>
	/// <param name="cloud">输入点云</param>
	/// <param name="leaf_size">分辨率</param>
	/// <param name="cloud_x">输出：插X值修补后的二维点云</param>
	void repairX_2d(const pcl::PointCloud<pcl::PointXY>::Ptr& cloud,
		float leaf_size,
		pcl::PointCloud<pcl::PointXY>::Ptr& cloud_x)
	{
		// 创建点云副本 (避免排序操作影响到本函数外)
		pcl::PointCloud<pcl::PointXY>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXY>(*cloud));
		auto compareYCoordinate = [](const pcl::PointXY& p1, const pcl::PointXY& p2)
		{
			return p1.y < p2.y; // 按y坐标从小到大对二维点云进行排序
		};
		std::sort(cloud_2d->points.begin(), cloud_2d->points.end(), compareYCoordinate);
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
	// 在二维中修补Y
	void repairY_2d(const pcl::PointCloud<pcl::PointXY>::Ptr& cloud,
		float leaf_size,
		pcl::PointCloud<pcl::PointXY>::Ptr& cloud_y)
	{
		// 创建点云副本 (避免排序操作影响到本函数外)
		pcl::PointCloud<pcl::PointXY>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXY>(*cloud));
		// 按x坐标从小到大对二维点云进行排序
		auto compareXCoordinate = [](const pcl::PointXY& p1, const pcl::PointXY& p2)
		{
			return p1.x < p2.x;
		};
		std::sort(cloud_2d->points.begin(), cloud_2d->points.end(), compareXCoordinate);
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


	// 对每个点，已知修补后的xy，生成并修补z坐标
	void repairZ(const pcl::PointCloud<pcl::PointXY>::Ptr& repair_xy,
		const pcl::PointCloud<PointT>::Ptr& voxel_top,
		const pcl::PointCloud<pcl::PointXY>::Ptr& voxel_top_2d,
		float leaf_size,
		pcl::PointCloud<PointT>::Ptr& cloud_xyz)
	{
		auto time_start = std::clock();
		pcl::search::KdTree<pcl::PointXY>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXY>);
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
			kdtree->nearestKSearch(point, k, nk_indices, nk_distances);
			pt_repair.x = point.x;
			pt_repair.y = point.y;
			pt_repair.z = voxel_top->points[nk_indices[0]].z;
			cloud_xyz->push_back(pt_repair);
		}

		auto end1 = std::clock();
		std::cout << "顶面修补z，耗时，耗时：" << std::difftime(end1, time_start) << "ms" << std::endl;
	}

	// 修补挖方
	void repairTop(const pcl::PointCloud<PointT>::Ptr& voxel_top,
		float leaf_size,
		pcl::PointCloud<PointT>::Ptr& top_repairXYZ)
	{
		// new二维点云
		pcl::PointCloud<pcl::PointXY>::Ptr top_repairX(new pcl::PointCloud<pcl::PointXY>);
		pcl::PointCloud<pcl::PointXY>::Ptr top_repairY(new pcl::PointCloud<pcl::PointXY>);
		pcl::PointCloud<pcl::PointXY>::Ptr top_repairXY(new pcl::PointCloud<pcl::PointXY>);
		pcl::PointCloud<pcl::PointXY>::Ptr voxel_top_2d(new pcl::PointCloud<pcl::PointXY>);
		// 将点云的每个点的 XY 坐标提取到二维点云中
		voxel_top_2d->points.resize(voxel_top->size());
		voxel_top_2d->width = voxel_top->size();
		voxel_top_2d->height = 1;
		voxel_top_2d->is_dense = true;
		for (size_t i = 0; i < voxel_top->size(); ++i) {
			voxel_top_2d->points[i].x = voxel_top->points[i].x;
			voxel_top_2d->points[i].y = voxel_top->points[i].y;
		}
		repairX_2d(voxel_top_2d, leaf_size, top_repairX);
		repairY_2d(voxel_top_2d, leaf_size, top_repairY);

		// 取交集 4s
		getIntersection_2d(top_repairX, top_repairY, top_repairXY);

		// 顶面修补 6s
		repairZ(top_repairXY, voxel_top, voxel_top_2d, leaf_size, top_repairXYZ);
	}

#pragma endregion

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
	// 初始化viewer
	void initViewer(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<PointT>::Ptr& cloud)
	{
		viewer.initCameraParameters();
		viewer.setBackgroundColor(0, 0, 0);
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cloud, centroid);
		viewer.setCameraPosition(centroid[0], centroid[1], centroid[2] + 10.0f, centroid[0], centroid[1], centroid[2], 0, 1, 0);
		viewer.registerPointPickingCallback(pointPickingCallback, (void*)&viewer);
	}
	// 添加viewport (从1开始)
	void addViewport(pcl::visualization::PCLVisualizer& viewer, int viewport = 1, double r = 0.0, double g = 0.0, double b = 0.0)
	{
		viewer.createViewPort((viewport - 1) * 0.25, 0.0, viewport * 0.25, 1.0, viewport); // 这种写法只能保证4个视口不重叠排列
		viewer.setBackgroundColor(r, g, b, viewport);
		viewer.addCoordinateSystem(10, "coordinate", viewport);
		std::string x_label = "x_label_v" + std::to_string(viewport);
		std::string y_label = "y_label_v" + std::to_string(viewport);
		std::string z_label = "z_label_v" + std::to_string(viewport);
		viewer.addText3D("x", pcl::PointXYZ(11, 0, 0), 1.0, 1.0, 1.0, 1.0, x_label, viewport);
		viewer.addText3D("y", pcl::PointXYZ(0, 11, 0), 1.0, 1.0, 1.0, 1.0, y_label, viewport);
		viewer.addText3D("z", pcl::PointXYZ(0, 0, 11), 1.0, 1.0, 1.0, 1.0, z_label, viewport);
		// 标题
		viewer.addText("viewport " + std::to_string(viewport), 10, 10, std::to_string(viewport), viewport);
	}
	// 添加点云 Z值着色
	void addCloud(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<PointT>::Ptr& cloud, int viewport)
	{
		pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(cloud, "z"); // 按Z轴值着色
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

#pragma region VTK可视化
	// 旋转polyData
	void rotateVtkData(vtkDataSet* data, const Eigen::Matrix4f& rotation )
	{
		// 创建一个vtkMatrix4x4对象，并用旋转矩阵填充它
		vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				matrix->SetElement(i, j, rotation(i, j));
			}
		}

		// 创建一个vtkTransform对象，并设置变换矩阵为matrix
		vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		transform->SetMatrix(matrix);

		// 创建一个vtkTransformPolyDataFilter对象，并设置输入为原始的polyData和变换为transform
		vtkSmartPointer<vtkTransformPolyDataFilter> filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
		filter->SetInputData(data);
		filter->SetTransform(transform);
		filter->Update();

		data = filter->GetOutput(); // 变换后的polyData
		//rotatedPolyData->DeepCopy(filter->GetOutput()); // 将变换后的polyData保存到输出参数rotatedPolyData中
	}
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
	/// <param name="data">vtkDataSet类的指针</param>
	static void myShow(vtkDataSet* data) {
		vtkDataSetMapper* mapper = vtkDataSetMapper::New();
		mapper->SetInputData(data);

		mapper->SetScalarRange(0, 7);

		vtkActor* actor = vtkActor::New();
		actor->SetMapper(mapper);
		actor->GetProperty()->SetColor(1.0, 0.0, 0.0);

		vtkCamera* camera = vtkCamera::New();
		camera->SetPosition(1, 1, 1);
		camera->SetFocalPoint(0, 0, 0);

		vtkRenderer* renderer = vtkRenderer::New();
		vtkAxesActor* axes = vtkAxesActor::New();
		// 设置x、y和z轴的长度
		//axes->SetTotalLength(10, 10, 10);
		//renderer->AddActor(axes);

		vtkRenderWindow* renWin = vtkRenderWindow::New();
		renWin->AddRenderer(renderer);

		vtkRenderWindowInteractor* iren = vtkRenderWindowInteractor::New();
		iren->SetRenderWindow(renWin);

		renderer->AddActor(actor);
		renderer->SetActiveCamera(camera);
		renderer->ResetCamera();
		renderer->SetBackground(0, 0, 0);

		renWin->SetSize(800, 600);
		renWin->Render();
		iren->Start();

		mapper->Delete();
		actor->Delete();
		camera->Delete();
		renderer->Delete();
		renWin->Delete();
		iren->Delete();
	}

	/// <summary>
	/// 创建立方体群，并打开vtk窗口显示
	/// </summary>
	/// <param name="cloud_top">每个立方体的顶面中心点</param>
	/// <param name="cloud_bottom">每个立方体的底面中心点</param>
	/// <param name="plane_coefficients">底平面方程</param>
	/// <param name="inv_rotation">逆旋转矩阵</param>
	/// <param name="side_length">每个立方体的边长</param>
	/// <returns></returns>
	static void createCubes(const pcl::PointCloud<PointT>::Ptr cloud_top, 
		const pcl::PointCloud<PointT>::Ptr cloud_bottom, 
		const pcl::ModelCoefficients::Ptr plane_coefficients, 
		Eigen::Matrix4f inv_rotation,
		float side_length) 
	{
		auto time_start = std::clock();
		// cloud_top 和 cloud_bottom 大小相同
		int cube_num = cloud_top->size();
		int top_num = cloud_bottom->size();
		// 如果 n 和 m 不相同，打印信息并返回 nullptr
		if (cube_num != top_num) {
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
		int vertex_unit = 8; // 一个立方体的顶点数为8
		int poly_unit = 6; // 一个立方体的面数为6
		// 预先分配内存
		points->Allocate(vertex_unit * cube_num); //分配内存，不初始化数据
		polys->SetNumberOfCells(poly_unit * cube_num); // 分配内存，不初始化数据
		scalars->SetNumberOfValues(vertex_unit * cube_num); //分配内存，不初始化数据
		//double max_height = 0.0; // 初始化最大值为0
		//double min_height = 0.0; // 初始化最小值为0
		for (int i = 0; i < cube_num; i++) {
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
			//for (int j = 0; j < vertex_unit; j++) {
			//	points->SetPoint(i * vertex_unit + j, std::get<0>(vertex[j]), std::get<1>(vertex[j]), std::get<2>(vertex[j]));
			//}
			for (auto v : vertex) {
				points->InsertNextPoint(std::get<0>(v), std::get<1>(v), std::get<2>(v));
			}
			// 添加顶点索引到 vtkCellArray
			int offset = vertex_unit * i;
			polys->InsertNextCell(mkVtkIdList({ 0 + offset, 1 + offset, 2 + offset, 3 + offset }));
			polys->InsertNextCell(mkVtkIdList({ 4 + offset, 5 + offset, 6 + offset, 7 + offset }));
			polys->InsertNextCell(mkVtkIdList({ 0 + offset, 1 + offset, 5 + offset, 4 + offset }));
			polys->InsertNextCell(mkVtkIdList({ 1 + offset, 2 + offset, 6 + offset, 5 + offset }));
			polys->InsertNextCell(mkVtkIdList({ 2 + offset, 3 + offset, 7 + offset, 6 + offset }));
			polys->InsertNextCell(mkVtkIdList({ 3 + offset, 0 + offset, 4 + offset, 7 + offset }));
			// 添加标量值
			for (int j = 0; j < vertex_unit; j++) {
				scalars->SetValue(i * vertex_unit + j, cube_height); // todo
				//scalars->InsertNextValue(cube_height);
			}
		}
		// 添加数据到 vtkPolyData
		polyData->SetPoints(points);
		polyData->SetPolys(polys);
	
		polyData->GetPointData()->SetScalars(scalars);
		auto time_end = std::clock();
		std::cerr << "创建立方体群，耗时：" << std::difftime(time_end, time_start) << "ms" << std::endl; 

		rotateVtkData(polyData,inv_rotation); // 旋转polyData
		myShow(polyData); // 调用 myShow 函数来显示 polyData。
		return;
	}


#pragma endregion
	void test_vector_destruction() {
		std::cout << "Testing destruction of std::vector<PointT, Eigen::aligned_allocator<PointT>>" << std::endl;
		{
			std::vector<PointT, Eigen::aligned_allocator<PointT>> vector1(10);
		}
		std::cout << "std::vector<PointT, Eigen::aligned_allocator<PointT>> destroyed" << std::endl;

		std::cout << "Testing destruction of std::vector<int>" << std::endl;
		{
			std::vector<int> vector2(20);
		}
		std::cout << "std::vector<int> destroyed" << std::endl;
	}
	// 快速挖方体积计算入口
	VolumeResult getFastVolume(pcl::PointCloud<PointT>::Ptr cloud_src, Eigen::Vector3f normal, PointT point, float leaf_size)
	{
		test_vector_destruction();
		VolumeResult result;
		// CSF地面分割（得到挖方的顶面）
		csf_ground_segmentation(cloud_src, cloud_top, cloud_offground);

		normal = normal.normalized(); // 单位化平面法向量
		calcPlaneCoefficients(point, normal, plane_coefficients); //求平面方程

		// 旋转点云
		Eigen::Vector3f normal_planeXY(0.0f, 0.0f, 1.0f); // 旋转后的平面法向
		Eigen::Matrix4f rotation = getRotationMatrix(normal, normal_planeXY); // 求旋转矩阵
		auto inv_rotation = rotation.transpose(); // 得逆旋转矩阵
		pcl::transformPointCloud(*cloud_top, *top_rotated, rotation); // 旋转顶面

		// 挖方顶面octree体素化
		getVoxelCenters(top_rotated, leaf_size, top_rotated_voxel);

		// 修补
		repairTop(top_rotated_voxel, leaf_size, top_rotated_repair);

		// 逆旋转生成真实的顶面
		pcl::transformPointCloud(*top_rotated_repair, *top_real_repair, inv_rotation);
		// 投影还原真实底面
		projectPointCloudToPlane(top_real_repair, plane_coefficients, bottom_real_repair);
		// 旋转生成转正的底面
		pcl::transformPointCloud(*bottom_real_repair, *bottom_rotated_repair, rotation);
		//求转正的底面的平面方程
		calcPlaneCoefficients(bottom_rotated_repair->points[0], normal_planeXY, rotated_plane_coefficients); 
		// 计算体积  (用真实的点)
		//calcCubeClusterVolume(bottom_real_repair, top_real_repair, plane_coefficients, leaf_size);
		// 计算体积  (用转正后的点)
		calcCubeClusterVolume(bottom_rotated_repair, top_rotated_repair, rotated_plane_coefficients, leaf_size, result);

		// PCL处理过程可视化
		pcl::visualization::PCLVisualizer viewer("MutiViewer");
		initViewer(viewer, cloud_top);
		addViewport(viewer, 1);
		addViewport(viewer, 2);
		addViewport(viewer, 3);
		addViewport(viewer, 4);

		addCloud(viewer, cloud_top, 0.1, 1);
		addCloud(viewer, cloud_offground, 0.6, 1);

		addCloud(viewer, top_rotated_voxel, 0.4, 2);

		addCloud(viewer, top_rotated_repair, 0.2, 3);
		addCloud(viewer, bottom_rotated_repair, 0.7, 3);

		addCloud(viewer, top_real_repair, 0.3, 4);
		addCloud(viewer, bottom_real_repair, 0.8, 4);

		viewer.spinOnce();
		createCubes(top_rotated_repair, bottom_rotated_repair, rotated_plane_coefficients, inv_rotation, leaf_size * 0.9); // 创建立方体群 7.8s



		return result;
	}

}

