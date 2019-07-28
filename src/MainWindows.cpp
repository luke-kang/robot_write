#include "MainWindows.h"
#include "discriminant_img.h"
#include "transform_position.h"
#include "Aubo_driver.h"

#define DE2RA M_PI/180

windows::windows(QWidget *parents): QWidget(parents){
    this->setWindowTitle("机器人写字");
    this->setFixedSize(200, 150);

    layout = new QVBoxLayout(this);

    // 添加组件。
    detUI();
    robotUI();

    layout->addLayout(layout1);
    layout->addLayout(layout2);
    this->setLayout(layout);

}

void windows::detUI() {
    layout1 = new QHBoxLayout;
    detImgButton = new QPushButton("识别数字");
    detText = new QLabel("未识别");

    layout1->addWidget(detImgButton);
    layout1->addWidget(detText);

    // 添加槽函数。
    connect(detImgButton, &QPushButton::clicked, this, &windows::det_module);

}

void windows::robotUI() {
    layout2 = new QHBoxLayout;
    robotButton = new QPushButton("开始写字");
    robotText = new QLabel("未写字");

    layout2->addWidget(robotButton);
    layout2->addWidget(robotText);

    // 添加槽函数。
    connect(robotButton, &QPushButton::clicked, this, &windows::robot_module);

}

// 图片识别模块。
void windows::det_module() {
    detText->setText("正在识别。。。");

    discriminantImg get_label;
    // 1.kinect相机驱动拍照获得图片。
    Mat img = imread("./pingyi.jpg");

//    Mat img;
//    get_label.get_img(img);

    // 2.预处理。
    Mat input_img;
    get_label.preprocessing(img, input_img);

    // 3.带入深度学习模型进行预测。
    // 加载模型。
    shared_ptr<torch::jit::script::Module> model = torch::jit::load("../../data/model.pt");

    // 创建输入图片。
    torch::jit::IValue img_tensor = torch::from_blob(input_img.data, {1, 28, 28, 1}).permute({0, 3, 1, 2}).to(torch::kFloat);

    // 输出预测结果。
    auto output = at::softmax(model->forward({img_tensor}).toTensor(), 1);
    auto pred = at::argmax(output, 1, true);
    auto label = pred.item<int64_t>();

    detText->setText("识别完成!");
    cout << "识别的类别为：" << label << endl;

    object_label = label;

}

// 机器人写字模块。
void windows::robot_module() {
    if (object_label == -1){
        robotText->setText("没有识别到图片！");
        return;
    }

    robotText->setText("正在写字。。。");

    // 1.定义对象：输入识别出的类别，输出xyz信息。
    transformImg get_position(object_label);

    string path = "../../data/count/";
    vector<vector<Point>> points;
    get_position.processing(path, points);

    // 得到xyz。
    vector<PointCloud> result_point;
    get_position.get_position(points, result_point);

    cout << "已得到轮廓个数：" << result_point.size() << endl;

    // 2.机器人驱动写字。
    const char* paddr = "192.168.1.101";
    int port = 8899;
    const char* username = "wang_creator";
    const char* password = "123";
    driver driver_demo(paddr, port, username, password);
    driver_demo.init_robot();

    // 开始移动。
    double ZConst = 0.2160;
    double temp_x = 0;
    double temp_y = 0;
    for (int i = 0; i < result_point.size(); ++i) {
        for (int j = 0; j < result_point.at(i).size(); ++j) {
            double pos[6] = {result_point.at(i)[j].x, result_point.at(i)[j].y, ZConst, 180 * DE2RA, 0, -90 * DE2RA};
            driver_demo.movel(pos);

            temp_x = result_point.at(i)[j].x;
            temp_y = result_point.at(i)[j].y;
        }

        double pos[6] = {temp_x, temp_y, ZConst + 0.05, 180 * DE2RA, 0, -90 * DE2RA};
        driver_demo.movel(pos);
    }

    robotText->setText("写字完成！");
}
