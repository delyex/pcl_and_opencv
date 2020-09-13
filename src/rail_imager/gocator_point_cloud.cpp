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
	mapped_file_source file; // ��ǰ�򿪵� 2D ���������ļ�

	uint64_t ullLen;
	uint64_t ullPos = 0;

	file.open(m_filePath + "/" + name);
	ullLen = file.size();
	while ((ullLen - ullPos) >= 20)
	{
		memcpy(&m_s_LaserInput.s_Hdr, file.data() + ullPos, 20); // ��ȡ֡����ͷ��
		ullPos += 20;

		memcpy(m_s_LaserInput.ps_Buffer, file.data() + ullPos, m_s_LaserInput.s_Hdr.uiValidPointCount * sizeof(PROFILE_POINT)); // ��ȡ֡����
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

	file.close();				  // �ر��ļ�

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
	pass.setFilterLimitsNegative(remove_inside); // true: ����ѡ���ڵĵ㣬false������ѡ����ĵ�
	pass.filter(*cloud_filtered);
	return cloud_filtered;
}

/// @brief �ӵ������� OpenCV Mat������ʹ�� cv::imwrite ����Ľ� cv::Mat ����Ϊͼ��
/// @param cloud  ����
/// @param dimensionToRemove ��Ҫɾ��������ά�ȣ�Ĭ��Ϊ��z������������ͶӰ�� XoYƽ��
/// @param stepSize1 ��һά���񹹽�������Ĭ��Ϊ 1������ͼ��ߴ����������һ��
/// @param stepSize2 �ڶ�ά���񹹽�������Ĭ��Ϊ 1������ͼ��ߴ����������һ��
/// @return OpenCV ��ͼ�����
cv::Mat gocator::GocatorPointCloud::PointCloud2Mat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string dimensionToRemove, float stepSize1, float stepSize2)
{
	pcl::PointXYZ cloudMin, cloudMax; ///< ���Ƹ�ά�ȵ����ֵ����Сֵ
	pcl::getMinMax3D(*cloud, cloudMin, cloudMax); ///< ��ȡ�����Сֵ

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

	std::vector<std::vector<int>> pointCountGrid; ///< �����ڵĵ���

	//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> grid;

	for (float i = dimen1Min; i < dimen1Max; i += stepSize1)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr slice = passThroughFilter1D(cloud, dimen1, i, i + stepSize1); ///< �������񲽳��Ե��ƽ�����Ƭ

		std::vector<int> slicePointCount; ///< ��ǰ������Ƭ�У���������Ԫ�����ĵ���

		for (float j = dimen2Min; j < dimen2Max; j += stepSize2)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cell = passThroughFilter1D(slice, dimen2, j, j + stepSize2); ///< ��������Ƭ�ָ�Ϊ����Ԫ

			int gridSize = grid_cell->size();
			slicePointCount.push_back(gridSize); ///< ��������Ԫ����
		}
		pointCountGrid.push_back(slicePointCount);
	}

	cv::Mat mat(static_cast<int>(pointCountGrid.at(0).size()), static_cast<int>(pointCountGrid.size()), CV_8UC1);
	mat = cv::Scalar(0); // mat �е�Ԫ������Ϊ 0

	for (int i = 0; i < mat.rows; ++i)
	{
		for (int j = 0; j < mat.cols; ++j)
		{
			mat.at<uchar>(i, j) = pointCountGrid.at(j).at(i) ? 255 : 0;
		}
	}

	return mat;
}
