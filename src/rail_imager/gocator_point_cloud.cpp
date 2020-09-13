#include "gocator_point_cloud.h"
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl\filters\passthrough.h>
#include <pcl\common\common.h>
using namespace pcl;
using namespace gocator;
using namespace boost::filesystem;
using namespace boost::iostreams;

GocatorPointCloud::GocatorPointCloud()
{
	m_s_LaserInput.ps_Buffer = m_Points;
}

int gocator::GocatorPointCloud::SetPath(const std::string& path)
{
	m_filePath = path;
	return 0;
}

int GocatorPointCloud::LoadFile(const std::string& name, std::vector<PointCloud<PointXYZ>::Ptr>& pointClouds)
{
	m_fileName = name;
	mapped_file_source file; // 当前打开的 2D 激光数据文件

	uint64_t ullLen;
	uint64_t ullPos = 0;

	file.open(m_filePath + "/" + name);
	ullLen = file.size();
	while ((ullLen - ullPos) >= 20)
	{
		memcpy(&m_s_LaserInput.s_Hdr, file.data() + ullPos, 20); // 读取帧数据头部
		ullPos += 20;

		memcpy(m_s_LaserInput.ps_Buffer, file.data() + ullPos, m_s_LaserInput.s_Hdr.uiValidPointCount * sizeof(PROFILE_POINT)); // 读取帧数据
		ullPos += (uint64_t)m_s_LaserInput.s_Hdr.uiValidPointCount * sizeof(PROFILE_POINT);

		PointCloud<PointXYZ> cloud;
		cloud.width = m_s_LaserInput.s_Hdr.uiValidPointCount;
		cloud.height = 1;
		cloud.resize(cloud.width);
		for (size_t i = 0; i < cloud.width; i++)
		{
			cloud.points[i].x = m_s_LaserInput.ps_Buffer[i].x;
			cloud.points[i].y = -m_s_LaserInput.ps_Buffer[i].z;
			cloud.points[i].z = 0.0;
		}
		pointClouds.push_back(cloud.makeShared());
	}

	file.close();				  // 关闭文件

	return 0;
}

std::string gocator::GocatorPointCloud::GetFilePathName()
{
	return m_filePath + "/" + m_fileName;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr gocator::GocatorPointCloud::passThroughFilter1D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string field, const float low, const float high, const bool remove_inside)
{
	if (low > high)
	{
		std::cout << "Warning! Min is greater than max!\n";
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;

	pass.setInputCloud(cloud);
	pass.setFilterFieldName(field);
	pass.setFilterLimits(low, high);
	pass.setFilterLimitsNegative(remove_inside); // true: 返回选区内的点，false：返回选区外的点
	pass.filter(*cloud_filtered);
	return cloud_filtered;
}

/// @brief 从点云生成 OpenCV Mat，可以使用 cv::imwrite 方便的将 cv::Mat 保存为图像。
/// @param cloud  点云
/// @param dimensionToRemove 需要删除的数据维度，默认为“z”，即将点云投影到 XoY平面
/// @param stepSize1 第一维网格构建补偿，默认为 1，生成图像尺寸与点云坐标一致
/// @param stepSize2 第二维网格构建补偿，默认为 1，生成图像尺寸与点云坐标一致
/// @return OpenCV 的图像矩阵
cv::Mat gocator::GocatorPointCloud::PointCloud2Mat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string dimensionToRemove, float stepSize1, float stepSize2)
{
	pcl::PointXYZ cloudMin, cloudMax; ///< 点云各维度的最大值与最小值
	pcl::getMinMax3D(*cloud, cloudMin, cloudMax); ///< 获取最大最小值

	std::string dimen1, dimen2;
	float dimen1Max = 0.0, dimen1Min = 0.0, dimen2Min = 0.0, dimen2Max = 0.0;
	if (dimensionToRemove == "x")
	{
		dimen1 = "y";
		dimen2 = "z";
		dimen1Min = cloudMin.y;
		dimen1Max = cloudMax.y;
		dimen2Min = cloudMin.z;
		dimen2Max = cloudMax.z;
	}
	else if (dimensionToRemove == "y")
	{
		dimen1 = "x";
		dimen2 = "z";
		dimen1Min = cloudMin.x;
		dimen1Max = cloudMax.x;
		dimen2Min = cloudMin.z;
		dimen2Max = cloudMax.z;
	}
	else if (dimensionToRemove == "z")
	{
		dimen1 = "x";
		dimen2 = "y";
		dimen1Min = cloudMin.x;
		dimen1Max = cloudMax.x;
		dimen2Min = cloudMin.y;
		dimen2Max = cloudMax.y;
	}

	std::vector<std::vector<int>> pointCountGrid; ///< 网格内的点数

	//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> grid;

	for (float i = dimen1Min; i < dimen1Max; i += stepSize1)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr slice = passThroughFilter1D(cloud, dimen1, i, i + stepSize1); ///< 按照网格步长对点云进行切片

		std::vector<int> slicePointCount; ///< 当前网格切片中，各个网格单元包含的点数

		for (float j = dimen2Min; j < dimen2Max; j += stepSize2)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cell = passThroughFilter1D(slice, dimen2, j, j + stepSize2); ///< 将网格切片分割为网格单元

			int gridSize = grid_cell->size();
			slicePointCount.push_back(gridSize); ///< 保存网格单元点数
		}
		pointCountGrid.push_back(slicePointCount);
	}

	cv::Mat mat(static_cast<int>(pointCountGrid.at(0).size()), static_cast<int>(pointCountGrid.size()), CV_8UC1);
	mat = cv::Scalar(0); // mat 中的元素设置为 0

	for (int i = 0; i < mat.rows; ++i)
	{
		for (int j = 0; j < mat.cols; ++j)
		{
			mat.at<uchar>(i, j) = pointCountGrid.at(j).at(i) ? 255 : 0;
		}
	}

	return mat;
}
