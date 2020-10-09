#include "dlo.h"
#include "dlo_global.h"
#include "ros_image_converter.h"
// #include "kuka_moveit.h"
#include "kukafri_hw/servo.h"

vector<int64> g_vnClassList, g_vnCrossList;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Main_Node");
  ros::NodeHandle nh;
  CImageConverter ic;
  CIiwaServo sv;
  /*  KukaMoveit test */
  // CKukaMoveit km_main;
  // ros::AsyncSpinner spinner(4);
  // spinner.start();
  // km_main.GoHome(D_GROUP);

  /*  IiwaServo test  */
  sv.MoveLeftToHome(10);
  sv.MoveRightToHome(10);
  ros::Duration(10).sleep();
  // sv.MoveLeftToJoint(0, 0, 0, 0, 0, 0, 0);
  // sv.MoveRightToJoint(0, 0, 0, 0, 0, 0, 0);
  // ros::Duration(10).sleep();
  cout << "\t\t=====  Now Start  =====\n";

  ros::Subscriber camera_sub = nh.subscribe("/camera/color/image_raw", 1, &CImageConverter::CallbackCameraImgGet, &ic);  // 持续订阅相机rgb图像
  ros::Subscriber bw_sub = nh.subscribe("bw_topic", 1, &CImageConverter::CallbackBinaryImgGet, &ic);  // 订阅vgg16网络返回的轮廓图
  ros::Subscriber boxes_sub = nh.subscribe("boxes_topic", 1000, &CImageConverter::CallbackBoxesGet, &ic);  // 订阅yolo v3网络返回的端点交叉点识别结果boxes信息
  ros::Subscriber crop_class_sub = nh.subscribe("crop_class_topic", 1000, &CImageConverter::CallbackCropClassGet, &ic);  // 订阅二分类网络返回的交叉点分类结果
  
  ros::Rate loop_rate(10);
  while(ros::ok()){
    // ROS_INFO_STREAM("Main thread [" << boost::this_thread::get_id() << "]");
    // namedWindow("m_imgCamera");
    // imshow("m_imgCamera", ic.m_imgCamera);
    // startWindowThread();
    // waitKey();
    if(ic.flagCameraImgReady){
      ic.ProcessShowCameraView();
      ic.flagCameraImgReady = 0;
      if(ic.flagReady4Next)
        ic.Init();
    }
    if(ic.flagBinaryImgReady){
      ic.ProcessSkeleton();
      ic.flagSkeletonReady = 1;
      ic.flagBinaryImgReady = 0;
    }
    if(ic.flagBoxesReady && ic.flagSkeletonReady){
      ic.ProcessTraversal();
      ic.flagSkeletonReady = 0;
      ic.flagBoxesReady = 0;
    }
    if(ic.flagCropClassReady){
      ic.ProcessStrategy();
      ic.flagCropClassReady = 0;
      ic.flagReady4Next = 1;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
	return 0;
}

void CImageConverter::CallbackCameraImgGet(const sensor_msgs::ImageConstPtr& msg)
  {
    // ROS_INFO_STREAM("CallbackCameraImgGet thread [" << boost::this_thread::get_id() << "]");
    SensorMsgs2CvMat(msg, m_imgCamera);
    flagCameraImgReady = 1;
  }

void CImageConverter::CallbackBinaryImgGet(const std_msgs::String& msg)
  {
    // ROS_INFO_STREAM("CallbackBinaryImgGet thread [" << boost::this_thread::get_id() << "]");
    flagBinaryImgReady = 1;
  }

void CImageConverter::CallbackBoxesGet(const dlo::BoundingBoxes::ConstPtr& boxes)
  {
    // ROS_INFO_STREAM("CallbackBoxesGet thread [" << boost::this_thread::get_id() << "]");
    m_boxes = boxes;
    flagBoxesReady = 1;
  }

void CImageConverter::CallbackCropClassGet(const std_msgs::Int64::ConstPtr& cropclass)
  {
    // ROS_INFO_STREAM("CallbackCropClassGet thread [" << boost::this_thread::get_id() << "]");
    g_vnClassList.push_back(cropclass->data); // 二分类返回的分类结果
    if(m_vstrCropDir.size() == g_vnClassList.size())
      flagCropClassReady = 1;
  }