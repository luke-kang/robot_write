#include "transform_position.h"

transformImg::transformImg(int object_label): label(object_label) {
}

void transformImg::processing(string &path, vector<vector<Point>> &points) {
    boost::format fmt( "%s/%d.%s" );
    Mat img = imread((fmt%path %label %"jpg").str());
    cv::resize(img, img, Size(120, 160));

    // 预处理。
    Mat gray, bin;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    threshold(gray, bin, 0, 255, THRESH_BINARY | THRESH_OTSU);

    // 画出轮廓。
    vector<vector<Point>> temp_point;
    vector<Vec4i> h;
    findContours(bin, temp_point, h, RETR_TREE, CHAIN_APPROX_SIMPLE);

    Mat mask = Mat::zeros(img.size(), img.type());
    for (int i = 0; i < temp_point.size(); ++i) {
        double areas = contourArea(temp_point[i]);

        if ((areas > 18000) || (areas < 100)) continue;

        drawContours(mask, temp_point, i, Scalar(255, 0, 0));
        points.push_back(temp_point[i]);
    }

    imshow("test", mask);
    waitKey(0);
}

// 获得xyz。
void transformImg::get_position(vector<vector<Point>> &points, vector<PointCloud> &output_point) {
    // 得到内参与外参。
    load_xml();

    // 开始计算。
    int xOffset = 750;
    int yOffset = 500;
    for (int i = 0; i < points.size(); ++i) {

        PointCloud tempCloudPoint;
        for (int j = 0; j < points[i].size(); ++j) {

            // 加入偏移量。
            int u = xOffset + points[i][j].x;
            int v = points[i][j].y + yOffset;

            // TODO: 深度值给定，后续可以通过相机驱动得到。
            int d = 800;

            // 获得像素坐标与相机坐标的位置关系。
            Eigen::Vector3d point;
            point[2] = double(d) / 1000.0;
            point[0] = (u - cx) * point[2] / fx;
            point[1] = (v - cy) * point[2] / fy;

            // 获得相机坐标与基坐标的位置关系，左乘。
            Eigen::Vector3d pointWorld = T * point;

            PointT temp_point;
            temp_point.x = pointWorld[0];
            temp_point.y = pointWorld[1];
            temp_point.z = pointWorld[2];

            tempCloudPoint.push_back(temp_point);
        }

        output_point.push_back(tempCloudPoint);
    }
}

// 读取内外参xml文件。
void transformImg::load_xml(){
    string inter_path = "../../data/params/3DCameraInCailResult.xml";
    string out_path = "../../data/params/3DCameraExCailResult.xml";

    // 读取内参。
    FileStorage fs;
    if (!fs.open(inter_path, FileStorage::READ)){
        cout << "内参读取错误！" << endl;
        return;
    }
    vector<double> inter_data;
    fs["cameraMatrix"]["data"] >> inter_data;

    fx = inter_data[0];
    fy = inter_data[4];
    cx = inter_data[2];
    cy = inter_data[5];

    // 读取外参矩阵。
    FileStorage fs_2;
    if (!fs_2.open(out_path, FileStorage::READ)){
        cout << "读取外参错误！" << endl;
        return;
    }

    double angle = fs_2["Angle"];
    double axis_x = fs_2["AxisX"];
    double axis_y = fs_2["AxisY"];
    double axis_z =  fs_2["AxisZ"];
    Eigen::AngleAxisd temp_matrix(angle, Eigen::Vector3d(axis_x, axis_y, axis_z));

    rotation_vector = temp_matrix.matrix();

    // 读取外参平移位置，后续需要转换成米为单位。
    x = fs_2["TranslationX"];
    y = fs_2["TranslationY"];
    z = fs_2["TranslationZ"];

    // 旋转向量转旋转矩阵求出外参。
    Eigen::Isometry3d temp_T(rotation_vector);
    temp_T.pretranslate(Eigen::Vector3d(x, y, z) / 1000.0);
    T = temp_T.matrix();
}
