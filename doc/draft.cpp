void getPlaneBySacPoint(const pcl::PointCloud<PointT>::Ptr cloud, PointT point, pcl::ModelCoefficients::Ptr coeff)
{
	std::cerr << "\n邻域迭代SAC...\n", tt.tic();

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
