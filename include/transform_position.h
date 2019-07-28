#ifndef ROBOT_WRITE_TRANSFORM_POSITION_H
#define ROBOT_WRITE_TRANSFORM_POSITION_H

#include <iostream>
#include <boost/format.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace cv;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class transformImg{
public:
    transformImg(int object_label);
    ~transformImg(){}

    // 图像处理，输入图片地址与类别，输出轮廓点。
    void processing(string &path, vector<vector<Point>> &points);

    // 计算xyz的坐标值,输入轮廓点，输出xyz坐标点。
    void get_position(vector<vector<Point>> &points, vector<PointCloud> &output_point);
    void load_xml();

private:
    int label;

    // 内参与外参。
    double x, y, z;
    double cx, cy, fx, fy;
    Eigen::AngleAxisd rotation_vector;      // 外参旋转向量。
    Eigen::Isometry3d T;                    // 初始化变换矩阵。

};

#endif //ROBOT_WRITE_TRANSFORM_POSITION_H
