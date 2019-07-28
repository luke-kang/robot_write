#ifndef ROBOT_WRITE_DISCRIMINANT_IMG_H
#define ROBOT_WRITE_DISCRIMINANT_IMG_H

#include <sstream>
#include <cstdlib>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

using namespace std;
using namespace cv;

class discriminantImg{
public:
    discriminantImg();
    ~discriminantImg(){};

    // 得到画出的图片。
    void get_img(Mat &img);

    // 图片预处理成标准格式，带入cnn模型中。
    void preprocessing(Mat &input, Mat &output);


private:
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev;
    libfreenect2::PacketPipeline *pipeline;

};

#endif //ROBOT_WRITE_DISCRIMINANT_IMG_H
