
#pragma once
#include <iostream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2\core\base.hpp>
#include <opencv2\core\mat.hpp>
namespace gocator
{
	/**
	 * @brief 点云数据集信息
	 *
	 */
	struct PROFILE_POINT_SET_HEADER
	{
		unsigned long long ullTimestamp;  ///< 时间戳，单位：ns
		unsigned long long ullFrameIndex; ///< 帧号，用来同步
		unsigned int uiValidPointCount;   ///< 当前帧点数
		unsigned int uiLaserSn;           ///< 激光器编号，0~3

		/// operator '=' reload
		void operator=(const PROFILE_POINT_SET_HEADER& pt_hdr)
		{
			ullTimestamp = pt_hdr.ullTimestamp;
			ullFrameIndex = pt_hdr.ullFrameIndex;
			uiValidPointCount = pt_hdr.uiValidPointCount;
			uiLaserSn = pt_hdr.uiLaserSn;
		}
	};
	struct PROFILE_POINT
	{
		double x; ///< 扫描平面横向坐标，单位：mm
		double z; ///< 扫描平面纵向坐标，单位：mm

		/// Constructor
		PROFILE_POINT() : x(0), z(0) {}
		PROFILE_POINT(double px, double py) : x(px), z(py) {}

		/// Copy constructor.
		PROFILE_POINT(const PROFILE_POINT& pt)
		{
			x = pt.x;
			z = pt.z;
		}

		/// Operator '=' reload
		void operator=(const PROFILE_POINT& pt)
		{
			x = pt.x;
			z = pt.z;
		}

		/// Operator '-' reload
		PROFILE_POINT operator-(const PROFILE_POINT& pt) const
		{
			return PROFILE_POINT(x - pt.x, z - pt.z);
		}

		/// Operator '+' reload
		PROFILE_POINT operator+(const PROFILE_POINT& pt) const
		{
			return PROFILE_POINT(x + pt.x, z + pt.z);
		}

		/// Operator '/' reload
		PROFILE_POINT operator/(const double& div) const
		{
			return PROFILE_POINT(x / div, z / div);
		}
	};

	/// 激光扫描仪数据输入
	struct LASER2D_FRAME_IN
	{
		PROFILE_POINT_SET_HEADER s_Hdr; ///< 数据帧头部
		/**
		* \brief 输出激光扫描数据点指针
		* \remarks 跨进程时不能使用
		*/
		PROFILE_POINT* ps_Buffer;
	};

	class GocatorPointCloud
	{
	private:
		std::string m_filePath;
		std::string m_fileName;
		LASER2D_FRAME_IN m_s_LaserInput;					///< 读取 2D 激光扫描文件数据变量
		PROFILE_POINT m_Points[1024];
	public:
		GocatorPointCloud();

		int SetPath(const std::string& path);
		int LoadFile(const std::string& name, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pointClouds);
		std::string GetFilePathName();
		pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughFilter1D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string field, const float low, const float high, const bool remove_inside = false);
		cv::Mat PointCloud2Mat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string dimensionToRemove = "z", float stepSize1 = 1.0f, float stepSize2 = 1.0f);
	};
}