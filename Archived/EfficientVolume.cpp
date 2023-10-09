#include "EfficientVolume.h"
namespace VolumeEfficent
{
	pcl::visualization::PCLVisualizer viewer("MutiViewer");
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
	pcl::PointCloud<PointT>::Ptr top_repair_inv(new pcl::PointCloud<PointT>); //修补后的挖方顶面网格点云
	pcl::PointCloud<PointT>::Ptr bottom_repair_inv(new pcl::PointCloud<PointT>); //修补后的挖方顶面网格点云



#pragma region 无任何函数依赖的纯数学方法
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
	void addViewport(pcl::visualization::PCLVisualizer& viewer, int viewport = 1, double r=0.0, double g=0.0, double b=0.0 )
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
		pcl::visualization::PointCloudColorHandlerCustom<PointT> color(cloud, r*255.0, g*255.0, b*255.0);
		std::string cloud_id = "cloud" + std::to_string(viewport) + std::to_string(hue);
		viewer.addPointCloud(cloud, color, cloud_id, viewport);
	}

	void showClouds(pcl::PointCloud<PointT>::Ptr& cloud, int viewport = 0)
	{
		pcl::visualization::PCLVisualizer viewer("MutiViewer");
		static bool first_time = true;
		if (first_time)
		{
			viewer.initCameraParameters(); // 初始化
			viewer.setBackgroundColor(0, 0, 0); // 背景
			// 初始相机位置
			Eigen::Vector4f centroid;
			pcl::compute3DCentroid(*cloud, centroid);
			viewer.setCameraPosition(centroid[0], centroid[1], centroid[2] + 10.0, centroid[0], centroid[1], centroid[2], 0, 1, 0);
			// 选点Callback注册
			viewer.registerPointPickingCallback(pointPickingCallback, (void*)&viewer); // 注册点云选取回调函数
			first_time = false;
		}
		// 创建新视口
		viewer.createViewPort(viewport * 0.5 - 0.5, 0.0, viewport * 0.5, 1, viewport);
		// 标题
		viewer.addText("viewport " + viewport, 10, 10);
		// 添加坐标轴及标签
		viewer.addCoordinateSystem(10, "coordinate", viewport);
		std::string x_label = "x_label_v" + std::to_string(viewport + 1);
		std::string y_label = "y_label_v" + std::to_string(viewport + 1);
		std::string z_label = "z_label_v" + std::to_string(viewport + 1);
		viewer.addText3D("x", pcl::PointXYZ(11, 0, 0), 1.0, 1.0, 1.0, 1.0, x_label, viewport);
		viewer.addText3D("y", pcl::PointXYZ(0, 11, 0), 1.0, 1.0, 1.0, 1.0, y_label, viewport);
		viewer.addText3D("z", pcl::PointXYZ(0, 11, 11), 1.0, 1.0, 1.0, 1.0, z_label, viewport);
		std::string cloud_id = "cloud" + std::to_string(viewport + 1); // 点云id
		pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(cloud, "z"); // 按Z轴值着色
		viewer.addPointCloud<PointT>(cloud, color_handler, cloud_id,viewport);  // 添加点云
	}




	void visualizePointClouds(pcl::PointCloud<PointT>::Ptr& cloud1, pcl::PointCloud<PointT>::Ptr& cloud2)
	{
		pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
		viewer.initCameraParameters();

		int v1(0);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.addCoordinateSystem(10, "coordinate",v1);
		viewer.addText3D("x", pcl::PointXYZ(11, 0, 0), 1.0, 1.0, 1.0, 1.0, "x_label_v1");
		viewer.addText3D("y", pcl::PointXYZ(0, 11, 0), 1.0, 1.0, 1.0, 1.0, "y_label_v1");
		viewer.addText3D("z", pcl::PointXYZ(0, 0, 11), 1.0, 1.0, 1.0, 1.0, "z_label_v1");
		// 计算点云的质心
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cloud1, centroid);
		// 设置相机位置和方向
		viewer.setCameraPosition(centroid[0], centroid[1], centroid[2] + 10.0, centroid[0], centroid[1], centroid[2], 0, 1, 0);
		viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v1);
		viewer.addText("Cloud before transforming", 10, 10, "v1 test", v1);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> color(cloud1, 0, 255, 0);
		viewer.addPointCloud(cloud1, color, "cloud", v1);

		int v2(0);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.addCoordinateSystem(10, "coordinate", v2);
		viewer.addText3D("x", pcl::PointXYZ(11, 0, 0), 1.0, 1.0, 1.0, 1.0, "x_label_v2");
		viewer.addText3D("y", pcl::PointXYZ(0, 11, 0), 1.0, 1.0, 1.0, 1.0, "y_label_v2");
		viewer.addText3D("z", pcl::PointXYZ(0, 0, 11), 1.0, 1.0, 1.0, 1.0, "z_label_v2");
		viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v2);
		viewer.addText("Cloud after transforming", 10, 10, "v2 test", v2);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> color_transformed(cloud2, 0, 255, 0);
		viewer.addPointCloud(cloud2, color_transformed, "cloud_transformed", v2);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
		}
	}
	// 可视化三维点云
	void visualizePointCloud(pcl::PointCloud<PointT>::Ptr cloud, const std::string& window_name )
	{
		pcl::visualization::PCLVisualizer viewer(window_name); // 创建一个可视化窗口
		viewer.setBackgroundColor(0, 0, 0);
		// 添加轴标签
		viewer.addCoordinateSystem(5.0, Eigen::Affine3f::Identity(), "coordinate");
		viewer.addText3D("x", pcl::PointXYZ(11, 0, 0), 1.0, 1.0, 1.0, 1.0, "x_label");
		viewer.addText3D("y", pcl::PointXYZ(0, 11, 0), 1.0, 1.0, 1.0, 1.0, "y_label");
		viewer.addText3D("z", pcl::PointXYZ(0, 0, 11), 1.0, 1.0, 1.0, 1.0, "z_label");
		//重置相机视角
		viewer.resetCamera();
		// 计算点云的质心
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cloud, centroid);
		// 设置相机的位置和方向
		viewer.setCameraPosition(centroid[0] - 30.0, centroid[1] - 50.0, centroid[2] + 10.0,
			centroid[0], centroid[1], centroid[2], 0.0, 0.0, 1.0);
		pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(cloud, "z"); // 按Z轴值着色
		viewer.registerPointPickingCallback(pointPickingCallback, (void*)&viewer); // 注册点云选取回调函数
		viewer.addPointCloud<PointT>(cloud, color_handler, "point_cloud");  // 添加点云
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud"); // 点大小

		viewer.spin();
		
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
	// 二维点云化为z值统一的三维点云
	void convertTo3d_constZ(const pcl::PointCloud<pcl::PointXY>::Ptr& inputCloud, pcl::PointCloud<PointT>::Ptr& outputCloud ,float zValue =0.0f)
	{
		// 清空输出点云
		outputCloud->clear();

		// 遍历输入点云中的每个点
		for (const auto& point : inputCloud->points)
		{
			// 创建一个三维点对象并设置坐标
			PointT newPoint;
			newPoint.x = point.x;
			newPoint.y = point.y;
			newPoint.z = zValue;

			// 将新点添加到输出点云中
			outputCloud->push_back(newPoint);
		}
	}
	// 已知两个平面p1,p2的法向量分别为n1和n2，求p1到p2的旋转矩阵
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

	// 已知一个平面的方程，其旋转后与xy平面重合。求齐次变换矩阵
	Eigen::Matrix4f getRotationMatrix(const pcl::ModelCoefficients::Ptr& plane_coefficients) {
		// 假设 plane_coefficients->values = [A, B, C, D]，表示平面方程为 Ax + By + Cz + D = 0
		Eigen::Vector3f plane_normal(plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2]);

		// 计算法向量与 z 轴之间的夹角
		float cos_angle = plane_normal.dot(Eigen::Vector3f::UnitZ()) / plane_normal.norm();
		float angle_y = acos(cos_angle);

		// 计算法向量在 xy 平面上的投影与 x 轴之间的夹角
		Eigen::Vector2f normal_xy(plane_normal.x(), plane_normal.y());
		float angle_z = atan2(normal_xy.y(), normal_xy.x());

		// 构造绕 y 轴和 z 轴旋转的旋转矩阵
		Eigen::AngleAxisf rotation_y(angle_y, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf rotation_z(-angle_z, Eigen::Vector3f::UnitZ());

		Eigen::Matrix3f rotation_yz = (rotation_y * rotation_z).matrix();
		Eigen::Matrix4f transform;
		// 将旋转矩阵扩展为齐次变换矩阵
		transform.setIdentity();
		transform.block<3, 3>(0, 0) = rotation_yz;
		return transform;
	}


	//// 已知一个平面的方程，其旋转后与xy平面重合。求旋转矩阵
	//Eigen::Matrix4f getRotationMatrix(const pcl::ModelCoefficients::Ptr& plane_coefficients) {
	//	// 假设 plane_coefficients->values = [A, B, C, D]，表示平面方程为 Ax + By + Cz + D = 0
	//	Eigen::Vector3f plane_normal(plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2]);

	//	// 计算法向量与 z 轴之间的夹角
	//	float cos_angle = plane_normal.dot(Eigen::Vector3f::UnitZ()) / plane_normal.norm();
	//	float angle = acos(cos_angle);

	//	// 构造绕 y 轴旋转的旋转矩阵
	//	Eigen::AngleAxisf rotation_y(angle, Eigen::Vector3f::UnitY());

	//	// 构造绕 x 轴旋转 90 度的旋转矩阵
	//	Eigen::AngleAxisf rotation_x(M_PI / 2, Eigen::Vector3f::UnitX());

	//	Eigen::Matrix3f rotation_xy = (rotation_x * rotation_y).matrix();
	//	Eigen::Matrix4f transform;
	//	// 将旋转矩阵扩展为齐次变换矩阵
	//	transform.setIdentity();
	//	transform.block<3, 3>(0, 0) = rotation_xy;
	//	return transform;
	//}

	// 已知旋转矩阵，求旋转后的点云
	void rotatePointCloud(
		const pcl::PointCloud<PointT>::Ptr& cloud,
		const Eigen::Vector3f& direction,
		Eigen::Matrix4f& transform,
		pcl::PointCloud<PointT>::Ptr& rotated_cloud) {
		// 计算 direction 与 x 轴之间的夹角
		float angle_y = atan2(direction.z(), direction.x());
		float angle_z = atan2(direction.y(), direction.x());

		// 构造绕 y 轴和 z 轴的旋转矩阵
		Eigen::AngleAxisf rotation_y(-angle_y, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf rotation_z(-angle_z, Eigen::Vector3f::UnitZ());
		Eigen::Matrix3f rotation_matrix = (rotation_z * rotation_y).matrix();

		// 将旋转矩阵扩展为齐次变换矩阵
		transform.setIdentity();
		transform.block<3, 3>(0, 0) = rotation_matrix;

		// 旋转点云
		pcl::transformPointCloud(*cloud, *rotated_cloud, transform);
	}



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
				if (pointNKNSquaredDistance[0] < 0.0001)//如果搜索到的第一个点距为0，那么这个点为重合点（因为用A搜B中的点，不存在包含自身的情况）
				{
					cloud_xy->points.push_back(cloud_x->points[i]);
				}
			}
		}

		auto end1 = std::clock();
		std::cerr << "取交集，耗时：" << std::difftime(end1, time_start) << "ms" << std::endl;
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
		pcl::io::savePCDFileBinary("../file/ground.pcd", *cloud_ground);
		//save2txt("../file/ground.txt", cloud_ground);

		auto time_end = std::clock();
		std::cerr << "\r\nCSF，耗时：" << std::difftime(time_end, time_start) << "ms" << std::endl;
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


	void projectPointCloud(
		const pcl::PointCloud<pcl::PointXY>::Ptr& cloud,
		const pcl::ModelCoefficients::Ptr& coefficients,
		pcl::PointCloud<PointT>::Ptr& result) {
		// 假设 coefficients->values = [A, B, C, D]，表示平面方程为 Ax + By + Cz + D = 0
		Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
		Eigen::Vector3f plane_origin(0, 0, -coefficients->values[3] / coefficients->values[2]);

		result->clear();

		for (const auto& p : *cloud) {
			Eigen::Vector3f point(p.x, p.y, 0);
			float distance = (point - plane_origin).dot(plane_normal);
			Eigen::Vector3f projected_point = point - distance * plane_normal;
			result->push_back(PointT(projected_point.x(), projected_point.y(), projected_point.z()));
		}
	}

	// 二维点云投影到平面
	void projectPointCloudToPlane(pcl::PointCloud<pcl::PointXY>::Ptr& cloud_2d, pcl::ModelCoefficients::Ptr& plane_coefficients, pcl::PointCloud<PointT>::Ptr& cloud_projected)
	{
		auto time_start = std::clock();
		// 将二维点云转换为三维点云
		pcl::PointCloud<PointT>::Ptr cloud_3d(new pcl::PointCloud<PointT>);
		cloud_3d->resize(cloud_2d->size());
		for (size_t i = 0; i < cloud_2d->size(); ++i) {
			(*cloud_3d)[i].x = (*cloud_2d)[i].x;
			(*cloud_3d)[i].y = (*cloud_2d)[i].y;
			(*cloud_3d)[i].z = 0.0; // 将z坐标设置为0
		}

		// 创建滤波器对象
		pcl::ProjectInliers<PointT> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setInputCloud(cloud_3d);
		proj.setModelCoefficients(plane_coefficients);
		proj.filter(*cloud_projected);
		auto time_end = std::clock();
		std::cerr << "二维投影生成挖方底面，耗时：" << std::difftime(time_end, time_start) << "ms" << std::endl;
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
		float positive_volume = leaf_size * leaf_size * positive_height_sum; //挖方的体积
		float negative_volume = leaf_size * leaf_size * std::abs(negative_height_sum); //填方的体积
		float total_volume = positive_volume + negative_volume; //挖方加上填方的体积
		float diff_volume = positive_volume - negative_volume; //挖方减去填方的体积
		float positive_area = leaf_size * leaf_size * positive_point_count; //平面面积（正部分）
		float negative_area = leaf_size * leaf_size * negative_point_count; //平面面积（负部分）
		float total_area = leaf_size * leaf_size * total_point_count; //平面面积（全部）
		// 打印
		auto time_end = std::clock();
		std::cerr << "计算体积，耗时：" << std::difftime(time_end, time_start) << "ms" << std::endl;
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
	void repairZ_c(const pcl::PointCloud<pcl::PointXY>::Ptr& repair_xy,
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

			if (kdtree->radiusSearch(point, search_radius, rs_indices, rs_distances) > 0)
			{

				PointT pt_repair; // z值修补后的三维点
				pt_repair.x = point.x;
				pt_repair.y = point.y;
				// 初始化最大z值和对应的索引
				float max_z_value = std::abs(voxel_top->points[rs_indices[0]].z);
				int max_z_index = rs_indices[0];
				// 遍历rs_indices找出z绝对值最大的点
				for (size_t j = 0; j < rs_indices.size(); ++j)
				{
					auto pt = voxel_top->points[rs_indices[j]];
					float current_z_value = std::abs(voxel_top->points[rs_indices[j]].z);
					if (current_z_value > max_z_value)
					{
						max_z_value = current_z_value;
						max_z_index = rs_indices[j];
					}
				}

				// 将z绝对值最大的点的z值赋给pt_repair.z
				pt_repair.z = voxel_top->points[max_z_index].z;
				//pt_repair.z = voxel_top->points[rs_indices[0]].z;
				cloud_xyz->push_back(pt_repair);
			}
			else
			{
				PointT pt_repair; // z值修补后的三维点
				kdtree->nearestKSearch(point, k, nk_indices, nk_distances);
				pt_repair.x = point.x;
				pt_repair.y = point.y;
				pt_repair.z = voxel_top->points[nk_indices[0]].z;
				cloud_xyz->push_back(pt_repair);
			}

		}

		auto end1 = std::clock();
		std::cout << "rs+nk，耗时：" << std::difftime(end1, time_start) << "ms" << std::endl;
	}
	void convert3DTo2D(const pcl::PointCloud<PointT>::Ptr& cloud_3d, pcl::PointCloud<pcl::PointXY>::Ptr& cloud_2d)
	{
		cloud_2d->resize(cloud_3d->size());
		for (size_t i = 0; i < cloud_3d->size(); ++i) {
			(*cloud_2d)[i].x = (*cloud_3d)[i].x;
			(*cloud_2d)[i].y = (*cloud_3d)[i].y;
		}
	}
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
	

	void repairVoxelTop_3d(const pcl::PointCloud<PointT>::Ptr& grid,
		float leaf_size,
		pcl::PointCloud<PointT>::Ptr& grid_repair)
	{
		repairX_3d(grid, leaf_size, grid_bottom_repairX);
		repairY_3d(grid, leaf_size, grid_bottom_repairY);
		// 取cloud_x和cloud_y的交集
		getIntersection_NK(grid_bottom_repairX, grid_bottom_repairY, grid_repair);
		//std::cout << "grid_bottom_repairX size: " << grid_bottom_repairX->size() << std::endl;
		//std::cout << "grid_bottom_repairY size: " << grid_bottom_repairY->size() << std::endl;
		//std::cout << "Intersection_NK size: " << grid_repair->size() << std::endl;
	}
	// 修补挖方
	void repairTop(const pcl::PointCloud<PointT>::Ptr& voxel_top,
		float leaf_size,
		pcl::PointCloud<PointT>::Ptr& top_repairXYZ)
	{
		// 创建二维点云
		pcl::PointCloud<pcl::PointXY>::Ptr bottom_repair_2d(new pcl::PointCloud<pcl::PointXY>);
		// 创建二维点云
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

		// 取交集 4.1s
		getIntersection_2d(top_repairX, top_repairY, top_repairXY);

		// 顶面修补 6.4s
		repairZ(top_repairXY, voxel_top, voxel_top_2d, leaf_size, top_repairXYZ);

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
		auto time_start = std::clock();
		repairX(grid, plane_coefficients, leaf_size, grid_bottom_repairX);
		repairY(grid, plane_coefficients, leaf_size, grid_bottom_repairY);
		// 取cloud_x和cloud_y的交集
		getIntersection_NK(grid_bottom_repairX, grid_bottom_repairY, grid_repair);
		//std::cout << "grid_bottom_repairX size: " << grid_bottom_repairX->size() << std::endl;
		//std::cout << "grid_bottom_repairY size: " << grid_bottom_repairY->size() << std::endl;
		//std::cout << "Intersection_NK size: " << grid_repair->size() << std::endl;

		auto time_end = std::clock();
		std::cerr << "底面修补，耗时：" << std::difftime(time_end, time_start) << "ms" << std::endl;
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
		float leaf_size) 
	{
		auto time_start = std::clock();
		pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
		//pcl::KdTreeFLANN<PointT>::Ptr kdtree(new pcl::KdTreeFLANN<PointT>);; // 创建一个 Kd 树对象
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
		auto time_end = std::clock();
		std::cerr << "顶面生成，耗时：" << std::difftime(time_end, time_start) << "ms" << std::endl;
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
	/// 创建立方体群，并打开vtk窗口显示
	/// </summary>
	/// <param name="cloud_top"></param>
	/// <param name="cloud_bottom"></param>
	/// <param name="side_length"></param>
	/// <returns></returns>
	static void createCubes(const pcl::PointCloud<PointT>::Ptr cloud_top, const pcl::PointCloud<PointT>::Ptr cloud_bottom, const pcl::ModelCoefficients::Ptr plane_coefficients, float side_length) {
		
		auto time_start = std::clock();
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
				scalars->InsertNextValue(cube_height);
			}
		}
		// 添加数据到 vtkPolyData
		polyData->SetPoints(points);
		polyData->SetPolys(polys);
		polyData->GetPointData()->SetScalars(scalars);
		auto time_end = std::clock();
		std::cerr << "创建立方体群，耗时：" << std::difftime(time_end, time_start) << "ms" << std::endl;
		// 调用 myShow 函数来显示 polyData。
		myShow(polyData);
		return;
	}

#pragma endregion


	void getVoxelCenters(pcl::PointCloud<PointT>::Ptr cloud_src, float leaf_size, pcl::PointCloud<PointT>::Ptr voxel_cloud) {
		
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
	VolumeResult getEfficientVolume_old(pcl::PointCloud<PointT>::Ptr cloud_src, Eigen::Vector3f normal, PointT point, float leaf_size)
	{
		VolumeResult result;
		normal = normal.normalized(); // 单位化平面法向量
		calcPlaneCoefficients(point, normal, plane_coefficients); //定义的平面的方程
		// CSF地面分割（得到挖方的顶面）
		csf_ground_segmentation(cloud_src, cloud_top, cloud_offground);
		// 投影生成挖方底面
		projectPointCloudToPlane(cloud_top, plane_coefficients, cloud_bottom); //2.7s
		// 挖方底面体素化
		getVoxelCenters(cloud_bottom, leaf_size, voxel_bottom);
		// 底面修补
		repairGridBottom(voxel_bottom, plane_coefficients, leaf_size, grid_bottom_repair);
		// 顶面生成
		createGridTop(cloud_bottom, cloud_top, grid_bottom_repair, grid_top_repair, leaf_size);
		// 计算体积
		calcCubeClusterVolume(grid_bottom_repair, grid_top_repair, leaf_size);
		// 创建立方体群
		createCubes(grid_top_repair, grid_bottom_repair, plane_coefficients, leaf_size * 0.9);
		return result;
	}
	// 挖方体积计算入口 不修补
	VolumeResult getEfficientVolume_NoRepair(pcl::PointCloud<PointT>::Ptr cloud_src, Eigen::Vector3f normal, PointT point, float leaf_size)
	{
		VolumeResult result;
		normal = normal.normalized(); // 单位化平面法向量
		calcPlaneCoefficients(point, normal, plane_coefficients); //定义的平面的方程

		// CSF地面分割（得到挖方的顶面）
		csf_ground_segmentation(cloud_src, cloud_top, cloud_offground);

		// 顶面octree体素化
		getVoxelCenters(cloud_top, leaf_size, voxel_top);
		//visualizePointCloud(voxel_top,"voxel_top");

		// 投影生成挖方底面
		projectPointCloudToPlane(voxel_top, plane_coefficients, voxel_bottom); //2.7s
		//visualizePointCloud(grid_bottom_repair,"grid_bottom_repair");

		// 计算体积
		calcCubeClusterVolume(voxel_top, voxel_bottom, leaf_size);

		// 创建立方体群
		createCubes(voxel_top, voxel_bottom, plane_coefficients, leaf_size * 0.9);


		return result;
	}
	// 挖方体积计算入口
	VolumeResult getEfficientVolume(pcl::PointCloud<PointT>::Ptr cloud_src, Eigen::Vector3f normal, PointT point, float leaf_size)
	{

		// CSF地面分割（得到挖方的顶面）
		csf_ground_segmentation(cloud_src, cloud_top, cloud_offground);

		VolumeResult result;
		normal = normal.normalized(); // 单位化平面法向量

		pcl::PointCloud<PointT>::Ptr cloud_top_rotated(new pcl::PointCloud<PointT>);

		calcPlaneCoefficients(point, normal, plane_coefficients); //定义的平面的方程

		Eigen::Vector3f normal_planeXY(0.0f, 0.0f, 1.0f);
		// 求旋转矩阵
		Eigen::Matrix4f transform = getRotationMatrix(normal, normal_planeXY);

		auto inv_transform = transform.transpose();

		// 旋转点云
		pcl::transformPointCloud(*cloud_top, *cloud_top_rotated, transform);

		// 顶面octree体素化
		getVoxelCenters(cloud_top_rotated, leaf_size, voxel_top);

		// 修补
		repairTop(voxel_top, leaf_size, grid_top_repair);

		// 逆旋转点云
		pcl::transformPointCloud(*grid_top_repair, *top_repair_inv, inv_transform);

		// 投影生成挖方底面
		projectPointCloudToPlane(top_repair_inv, plane_coefficients, bottom_repair_inv);

		// 计算体积
		calcCubeClusterVolume(bottom_repair_inv, top_repair_inv, leaf_size);

		// 可视化
		initViewer(viewer, cloud_top);
		addViewport(viewer, 1);
		addViewport(viewer, 2);
		addViewport(viewer, 3);
		addViewport(viewer, 4);

		addCloud(viewer, cloud_top, 0.1, 1);
		addCloud(viewer, cloud_offground, 0.9, 1);

		addCloud(viewer, voxel_top, 0.3, 2);

		addCloud(viewer, grid_top_repair, 0.5, 3);

		addCloud(viewer, top_repair_inv, 0.2, 4);
		addCloud(viewer, bottom_repair_inv, 0.8, 4);
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();  // 更新可视化窗口
		}
		// 创建立方体群
		createCubes(top_repair_inv, bottom_repair_inv, plane_coefficients, leaf_size * 0.9);

		return result;
	}




}

