#include "organized.h"

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughFilter1D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string field, const float low, const float high, const bool remove_inside = false)
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
cv::Mat PointCloud2Mat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string dimensionToRemove = "z", float stepSize1 = 1.0f, float stepSize2 = 1.0f)
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

	cv::Mat mat(static_cast<int>(pointCountGrid.size()), static_cast<int>(pointCountGrid.at(0).size()), CV_8UC1);
	mat = cv::Scalar(0); // mat 中的元素设置为 0

	for (int i = 0; i < mat.rows; ++i)
	{
		for (int j = 0; j < mat.cols; ++j)
		{
			mat.at<uchar>(i, j) = pointCountGrid.at(i).at(j) ? 255 : 0;
		}
	}

	return mat;
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::string filename = "D:/git_repos/dataset/PointCloudLibrary/data/tutorials/ism_test_cat.pcd";
	if (pcl::io::loadPCDFile(filename, *point_cloud) == -1)
	{
		std::cout << "Was not able to open file \"" << filename << "\".\n";
		return -1;
	}
	else
	{
		//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
		//viewer.showCloud(point_cloud);
		//while (!viewer.wasStopped())
		//{
		//}
		cv::Mat mat = PointCloud2Mat(point_cloud);
		cv::imwrite("a.png", mat);
	}
	return 0;
}
