#include "dlo.h"
#include "ros_image_converter.h"
#include "kuka_moveit.h"
// #include "gripper_control.h"


vector<int64> g_vnClassList, g_vnCrossList;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Main_Node");
  ros::NodeHandle nh;
  CKukaMoveit kmMain;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  kmMain.GoHome(D_GROUP);
  // CGripperControl gcMain;
  // gcMain.Gripper_Close('R');
  // gcMain.Gripper_Close('L');
  // gcMain.Gripper_Open('D');

  CImageConverter ic;
  ros::Subscriber camera_sub = nh.subscribe("/camera/color/image_raw", 1, &CImageConverter::CallbackCameraImgGet, &ic);  // 持续订阅相机rgb图像
  ros::Subscriber bw_sub = nh.subscribe("bw_topic", 1, &CImageConverter::CallbackBinaryImgGet, &ic);  // 订阅vgg16网络返回的轮廓图
  ros::Subscriber boxes_sub = nh.subscribe("boxes_topic", 1000, &CImageConverter::CallbackBoxesGet, &ic);  // 订阅yolo v3网络返回的端点交叉点识别结果boxes信息
  ros::Subscriber crop_class_sub = nh.subscribe("crop_class_topic", 1000, &CImageConverter::CallbackCropClassGet, &ic);  // 订阅二分类网络返回的交叉点分类结果
  ros::Rate loop_rate(10);
  ros::AsyncSpinner s(4);
  s.start();

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
    if(m_vstrCropDir.size() == g_vnClassList.size()){
      flagCropClassReady = 1;
      
      // else if(checkstate == 1){
      //   cout << endl << "checkstate == 1, trying m_imgBinaryO" << endl;
      //   classlist = {};
      //   crosslist = {};
      //   m_vstrCropDir = {};
      //   m_nCropNum = 0;
      //   // Mat imgo = imread("/home/lance/Workspaces/hxz_ws/pic_buffer/O.png");
      //   // imgo*=256;
      //   // m_imgSkeleton = skeleton(imgo, s_img_address, 1);
      //   m_imgSkeleton = skeleton(m_imgBinaryO, s_img_address, 3);
      //   m_vstrCropDir = traversal(s_img_address, m_imgSrc, m_boxesCopy);
      //   for(int c=0; c<m_vstrCropDir.size(); ++c)
      //   {
      //     string crossnum = m_vstrCropDir[c].substr(m_vstrCropDir[c].length()-5, 1); // 文件名中的交叉点序号 -- yolo网络决定
      //     crosslist.push_back(stoi(crossnum));  // 交叉点序号, stoi()string转int
      //     Mat crop_img = imread(m_vstrCropDir[c]);
      //     if(crop_img.empty()){cout << "open crop_img error in function CallbackBoxesGet" << endl;}
      //     sensor_msgs::ImagePtr crop_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", crop_img).toImageMsg();
      //     m_pubCrop.publish(crop_msg);
      //   }
      // }

    }
    // else if(m_nCropNum<m_vstrCropDir.size())
    //   {
    //     ++m_nCropNum;
    //     string crossnum = m_vstrCropDir[m_nCropNum].substr(m_vstrCropDir[m_nCropNum].length()-5, 1); // 文件名中的交叉点序号 -- yolo网络决定
    //     crosslist.push_back(stoi(crossnum));  // 交叉点序号, stoi()string转int
    //     Mat crop_img = imread(m_vstrCropDir[m_nCropNum]);
    //     sensor_msgs::ImagePtr crop_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", crop_img).toImageMsg();
    //     m_pubCrop.publish(crop_msg);
    // }
  }