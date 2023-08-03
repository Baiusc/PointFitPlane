#include "MyFunc.h"
namespace MyFunc
{
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
		PointT selected_point;
		selected_point.x = x;
		selected_point.y = y;
		selected_point.z = z;

		// 在终端输出选中点的坐标
		std::cout << "选点坐标：x=" << x << ", y=" << y << ", z=" << z << std::endl;
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
	void addViewport(pcl::visualization::PCLVisualizer& viewer, int viewport, int count, double r , double g , double b )
	{
		double x_min = (viewport - 1) * (1.0 / count);
		double x_max = viewport * (1.0 / count);
		viewer.createViewPort(x_min, 0.0, x_max, 1.0, viewport);
		//viewer.createViewPort((viewport - 1) * 0.25, 0.0, viewport * 0.25, 1.0, viewport); // 这种写法只能保证4个视口不重叠排列
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
	// 添加点云 Z值着色
	void addCloud(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<PointT>::Ptr& cloud, int viewport,std::string axis)
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
	// Add point cloud without changing its color
	void addCloud(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<PointT>::Ptr& cloud, int viewport)
	{
		std::string cloud_id = "cloud" + std::to_string(viewport);
		viewer.addPointCloud(cloud, cloud_id, viewport);
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

}

