#include "discriminant_img.h"

discriminantImg::discriminantImg(): dev(nullptr), pipeline(nullptr) {
    if(freenect2.enumerateDevices() == 0){
        std::cout << "no device connected!" << std::endl;
    }

    std::string serial = freenect2.getDefaultDeviceSerialNumber();

    cout<<"The serial number is :"<<serial<<endl;
    //! [discovery]

    pipeline = new libfreenect2::OpenGLPacketPipeline();

    dev = freenect2.openDevice(serial, pipeline);
}

void discriminantImg::get_img(Mat &img) {
    //! [listeners]
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
    libfreenect2::FrameMap frames;
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    //! [listeners]

    //! [start]
    dev->start();
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
    //! [start]
//    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
//    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);

    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
//    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    // 开始获取。
    img = Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);

    dev->stop();
    dev->close();
}

// 图片预处理成标准图片输出。
void discriminantImg::preprocessing(Mat &input, Mat &output) {
    // 灰度化。
    Mat gray;
    cvtColor(input, gray, COLOR_BGR2GRAY);

    // 边缘检测。
    Mat canny_gray;
    Canny(gray, canny_gray, 100, 200);

    // 寻找轮廓。
    vector<vector<Point>> points;
    vector<Vec4i> h;
    findContours(canny_gray, points, h, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // 得到矩形框。
    Rect rectPoint;
    for (int i = 0; i < points.size(); ++i) {

        double areas = contourArea(points[i]);

        if ((areas < 80000) || (areas > 90000)) continue;

        rectPoint = boundingRect(points[i]);
    }

    // TODO: 两种方式：

    // 得到ROI区域。
    int scale = 50;
    Rect rect = Rect(rectPoint.x + scale, rectPoint.y + scale,
            rectPoint.width - scale*2, rectPoint.height - scale*3);
    Mat mask = gray(rect);

    // 形态学操作，腐蚀。
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    erode(mask, mask, element, Point(-1, -1), 1);

    // 设置自定义阈值进行二值化处理。
    bitwise_not(mask, mask);
    for (int i = 0; i < mask.rows; ++i) {
        for (int j = 0; j < mask.cols; ++j) {
            if (mask.at<uchar>(i, j) < 30) continue;

            mask.at<uchar>(i, j) = 255;
        }
    }

    // 翻转操作。
    flip(mask, mask, 1);

    // 缩放图片与归一化。
    resize(mask, mask, cv::Size(), 28.0f/mask.cols, 28.0f/mask.rows);
    mask.convertTo(mask, CV_32FC1, 1.0f/255.0f);

    output = mask.clone();

//    imshow("test", mask);
//    waitKey(0);
}

