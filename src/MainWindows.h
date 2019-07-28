#ifndef ROBOT_WRITE_MAINWINDOWS_H
#define ROBOT_WRITE_MAINWINDOWS_H

#include <QtWidgets/QWidget>
#include <QtWidgets>
#include <QPushButton>
#include <QLabel>
#include <QLayout>

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <opencv2/opencv.hpp>

#include <torch/script.h>
#include <torch/torch.h>

using namespace cv;
using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class windows: public QWidget{
    Q_OBJECT

public:
    windows(QWidget *parents = nullptr);
    virtual ~windows(){}

    // UI
    void detUI();
    void robotUI();

    // 图片识别模块。
    void det_module();

    // 机器人写字模块。
    void robot_module();

private:
    int object_label = -1;           // 识别出来的数字类别。

    // UI
    QPushButton* detImgButton;
    QPushButton* robotButton;
    QLabel* detText;
    QLabel* robotText;
    QVBoxLayout* layout;
    QHBoxLayout* layout1;
    QHBoxLayout* layout2;
};

#endif //ROBOT_WRITE_MAINWINDOWS_H
