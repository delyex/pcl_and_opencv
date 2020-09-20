#include "rail_imager.h"
#include "gocator_point_cloud.h"

#include <vector>

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace cv;
using namespace gocator;

bool g_bEndOfLaserFiles = false;
int main()
{
	GocatorPointCloud cloud;
	string path = "D:/大轨检数据/上海地铁/0620/下行第三轮";
	string name = "GOCATOR1_20200620_120548.dat";
	vector<PointCloud<PointXYZ>::Ptr> clouds;

	cloud.SetPath(path);
	TicToc tt;
	cloud.LoadFile(name, clouds); // 将文件中的点云信息一次性全部读取到 clouds 中
	for (size_t i = 0; i < clouds.size(); i++)
	{
		tt.tic();
		cv::Mat mat = cloud.PointCloud2Mat(clouds[i]); // 讲 PCL 点云转换为 OpenCV::Mat
		std::string screenshots_dir = path + "/screenshots/";
		boost::filesystem::create_directory(screenshots_dir);

		std::stringstream ss;
		ss.str("");
		ss << screenshots_dir << "Screenshot_" << setw(6) << setfill('0') << i << ".png";
		cout << ss.str() << endl;

		cv::imwrite(ss.str(), mat);

		cout << "Convert " << ss.str() << " in " << tt.toc() << " ms." << endl;
		//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
		//viewer.showCloud(clouds[i].makeShared());
		//while (!viewer.wasStopped())
		//{
		//}
	}

	return 0;
}

