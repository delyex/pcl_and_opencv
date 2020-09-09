# pcl_and_opencv
PCL 和 OpenCV 联合应用。
## 1. 程序说明
* `range_image_visualization`
  
  使用 PCL 的 RangeImage 类将点云保存为图像，这个要考虑到相机模型和位置，点云图像会有畸变。
* `imager`

  使用 PCL 自带的函数将点云保存为 png 文件.
* `organized_point_cloud`
  
  将点云投影到某一个平面上，转换为 OpenCV 的 Mat 变量，然后用 OpenCV 保存图像，**没有畸变**。
