// point_cloud2png.cpp : Defines the entry point for the application.
//

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/io/png_io.h>

#include "point_cloud2png.h"

using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointXYZ> pointcloud;
	for (float x = -0.5f; x <= 0.5f; x += 0.01f)
	{
		for (float y = -0.5f; y <= 0.5f; y += 0.01f)
		{
			pcl::PointXYZ point;
			point.x = x;
			point.y = y;
			point.z = 2.0 * y;
			pointcloud.push_back(point);
		}
	}
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(pointcloud.makeShared());
	while (!viewer.wasStopped())
	{
	}

	pointcloud.width = (uint32_t)pointcloud.points.size();
	pointcloud.height = 1;

	float angularResolution = (float)(0.1f * M_PI / 180.0); ///> 角分辨率 1°，相邻像素表示的光束相差 1°。
	float maxAngleWidth = (float)(90.0f * M_PI / 180.0); ///> 
	float maxAngleHeight = (float)(40.0f * M_PI / 180.0); ///>  模拟的传感器能够看到 360° 影像
	Eigen::Affine3f sensorPos = (Eigen::Affine3f)Eigen::Translation3f(0, 0, -1); ///> 虚拟传感器的位置：（0，0，0）
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME; ///> x朝向右，y向下，z轴是向前的,另一种选择是激光框架，x面向前方，y向左，z向上。
	float noiseLevel = 0.0f; ///> 对于噪声，噪声是0，范围图像是使用普通的z缓冲区创建的
	float minRange = 0.0f; ///> minRange >0 ,所有更近的点都将被忽略
	int boarderSize = 0; ///> 边界大小>0,边界将会在裁剪时留下一个未被观察到的点的边界。

	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(pointcloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPos, coordinate_frame, noiseLevel, minRange, boarderSize);
	float* ranges = rangeImage.getRangesArray();
	unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, rangeImage.width, rangeImage.height);

	pcl::io::saveRgbPNGFile("image.png", rgb_image, rangeImage.width, rangeImage.height);

	cout << rangeImage << endl;

	cout << "Done!" << endl;
	return 0;
}
