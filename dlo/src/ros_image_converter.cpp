#include "ros_image_converter.h"
#include "dlo.h"

ImageConverter::ImageConverter()
{
    strImgFloder = "/home/lance/Workspaces/hxz_ws/pic_buffer/";
    crop_num = 0;
    bw_got = 0;
    classlist = crosslist = {};
    // camera_sub = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::cameraCb, this);  // 持续订阅相机rgb图像
    // bw_sub = it_.subscribe("bw_topic", 1000, &ImageConverter::bwCb, this);  // 订阅vgg16网络返回的轮廓图
    // bw_sub = nh_.subscribe("bw_topic", 1, &ImageConverter::bwCb, this);  // 订阅vgg16网络返回的轮廓图
    // crop_pub = it_.advertise("crop_topic", 1000);  // 发布旋转裁剪图
    crop_pub = nh_.advertise<std_msgs::String>("crop_topic", 1000);  // 发布旋转裁剪图
    // rgb_pub = it_.advertise("rgb_topic", 1);  // 发布相机获取到的rgb图像
    rgb_pub = nh_.advertise<std_msgs::String>("rgb_topic", 1000);  // 发布相机获取到的rgb图像
    // boxes_sub = nh_.subscribe("boxes_topic", 1000, &ImageConverter::boxesCb, this);  // 订阅yolo v3网络返回的端点交叉点识别结果boxes信息
    // crop_class_sub = nh_.subscribe("crop_class_topic", 1000, &ImageConverter::cropclassCb, this);  // 订阅二分类网络返回的交叉点分类结果
    getready = 1;
    bw_img = imgSkeleton = yolo_img = imgTraversal = camera_img = imread(strImgFloder + "init/black.png");
}

ImageConverter::~ImageConverter()
{
// cv::destroyWindow("OPENCV_WINDOW");
}
  
void ImageConverter::show_img(String strWindowName, Mat &imgShow, bool x255){
    if(x255)
        imgShow*=255;
    namedWindow(strWindowName, WINDOW_AUTOSIZE);  // === 显示图片 ===
    imshow(strWindowName, imgShow);
}

void ImageConverter::move_windows(){
    moveWindow("camera_img", 2000, 400);
    moveWindow("bw_img", 2000, 1000);
    moveWindow("Darknet", 0, 320);
    moveWindow("Traversal", 855, 320);
    moveWindow("visual_img", 0, 880);
    moveWindow("Result", 855, 880);
    waitKey();
}

