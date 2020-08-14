#include "dlo.h"
#include "ros_image_converter.h"
#include "move_kuka.h"


vector<int64> classlist, crosslist;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Main_Node");
  ros::NodeHandle n;
  KukaMoveit kmMain;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  kmMain.GoHome(D_GROUP);


  // kmMain.MoveToLeftPose(Point(485-80, 391-80), 1, {2.093, 0, 0});
  // KukaMoveit kuka;
  // cout<<"1\n";
  // kuka.MoveToLeftPose(0.3, 0.2, 0.6, {0, 0, 0, 1});
  // kuka.MoveLdxdydz(0.1, 0.1);
  // while(1){
  // kuka.MoveOneLeftJointIncrease(5, 0.3);
  // kuka.MoveOneRightJointIncrease(4, 0.3);
  // kuka.MoveOneLeftJointIncrease(5, -0.3);
  // kuka.MoveOneRightJointIncrease(4, -0.3);}


  ImageConverter ic;
  ros::Subscriber camera_sub = n.subscribe("/camera/color/image_raw", 1, &ImageConverter::cameraCb, &ic);  // 持续订阅相机rgb图像
  ros::Subscriber bw_sub = n.subscribe("bw_topic", 1, &ImageConverter::bwCb, &ic);  // 订阅vgg16网络返回的轮廓图
  ros::Subscriber boxes_sub = n.subscribe("boxes_topic", 1000, &ImageConverter::boxesCb, &ic);  // 订阅yolo v3网络返回的端点交叉点识别结果boxes信息
  ros::Subscriber crop_class_sub = n.subscribe("crop_class_topic", 1000, &ImageConverter::cropclassCb, &ic);  // 订阅二分类网络返回的交叉点分类结果
  
  ros::Rate loop_rate(10);
  ros::AsyncSpinner s(4);
  s.start();

  while(ros::ok()){
    ROS_INFO_STREAM("Main thread [" << boost::this_thread::get_id() << "]");
    // namedWindow("imgCamera");
    // imshow("imgCamera", ic.camera_img);
    // startWindowThread();
    // waitKey();
    ros::spinOnce();
    loop_rate.sleep();
  }
	return 0;
}

void ImageConverter::sensorMsgs_to_cvMat(const sensor_msgs::ImageConstPtr& msg, Mat& dst){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    dst = cv_ptr->image;
}

void ImageConverter::cameraCb(const sensor_msgs::ImageConstPtr& msg)
  {
    // ROS_INFO_STREAM("CameraCb thread [" << boost::this_thread::get_id() << "]");
    sensorMsgs_to_cvMat(msg, camera_img);
    if(getready)
      init();
  }

void ImageConverter::init()
  {
    cout << "NOW IN init()" << endl;
    start = clock(); // time
    classlist = {};
    crosslist = {};
    crop_dir = {};
    crop_num = 0;
    bw_got = 0;
    getready = 0;
    // Update GUI Window
    rgb_img = camera_img;
    // cv::Rect m_select = cv::Rect(0, 160, 640, 320);
		// Mat imgDst = rgb_img(m_select);
    // imwrite(strImgFloder + "R.png" ,imgDst);
    // rgb_img = imread(strImgFloder + "R.png");
    copyMakeBorder(rgb_img, rgb_img, 80, 80, 80, 80, BORDER_CONSTANT, Scalar(85, 120, 68));
    imwrite(strImgFloder + "R.png" ,rgb_img);
    std_msgs::String signal;
    signal.data = "1";
    rgb_pub.publish(signal);
    cout << "Published rgbimg successed!\n";
  }


void ImageConverter::bwCb(const std_msgs::String& msg)
  {
    ROS_INFO_STREAM("BwCb thread [" << boost::this_thread::get_id() << "]");
    if(!bw_got)
    {
      bw_got = 1;
      bw_img = imread(strImgFloder + "G.png");
      
      imgSkeleton = skeleton(bw_img, strImgFloder + "S.png", 3);
      
    }
    else
    {
      bw_imgo = imread(strImgFloder + "O");
      bw_imgo*=256;
    }
  }

void ImageConverter::boxesCb(const dlo::BoundingBoxes::ConstPtr& boxes)
  {
    ROS_INFO_STREAM("BoxesCb thread [" << boost::this_thread::get_id() << "]");
    boxes_copy = boxes;
    yolo_img = imread(strImgFloder + "D.png");
    

    crop_dir = traversal(strImgFloder + "S.png", rgb_img, boxes); // 裁剪图路径列表 -- 按遍历顺序
    imgTraversal = imread(strImgFloder + "T.png");
    

    for(int c=0; c<crop_dir.size(); ++c)
    {
      string crossnum = crop_dir[c].substr(crop_dir[c].length()-5, 1); // 文件名中的交叉点序号 -- yolo网络决定
      crosslist.push_back(stoi(crossnum));  // 交叉点序号, stoi()string转int
      std_msgs::String msgsCropDir;
      msgsCropDir.data = crop_dir[c];
      crop_pub.publish(msgsCropDir);
      // cout << "croppub\n";
    }
  }

void ImageConverter::cropclassCb(const std_msgs::Int64::ConstPtr& cropclass)
  {
    ROS_INFO_STREAM("CropclassCb thread [" << boost::this_thread::get_id() << "]");
    classlist.push_back(cropclass->data); // 二分类返回的分类结果
    if(crop_dir.size() == classlist.size()){
      cout << "NOW IN cropclassCb()/if" << endl;
      end = clock(); // time
      double endtime = (double)(end - start) / CLOCKS_PER_SEC; // time
      cout << endtime << endl; // time
      visualization(classlist, crosslist);  // === 可视化 ===
      int checkstate = strategy(classlist, crosslist);  // === 拆解策略 ===
      // show_img("bw_img", bw_img);
      // show_img("imgSkeleton", imgSkeleton);
      // show_img("yolo_img", yolo_img);
      // show_img("Traversal", imgTraversal);
      // move_windows();
      if(!checkstate){
        getready = 1;
        cout << "get ready for next\n";
      }
      else{
        getready = 1;
        cout << "get ready for next\n";
      }
      // else if(checkstate == 1){
      //   cout << endl << "checkstate == 1, trying bw_imgo" << endl;
      //   classlist = {};
      //   crosslist = {};
      //   crop_dir = {};
      //   crop_num = 0;
      //   // Mat imgo = imread("/home/lance/Workspaces/hxz_ws/pic_buffer/O.png");
      //   // imgo*=256;
      //   // imgSkeleton = skeleton(imgo, s_img_address, 1);
      //   imgSkeleton = skeleton(bw_imgo, s_img_address, 3);
      //   crop_dir = traversal(s_img_address, rgb_img, boxes_copy);
      //   for(int c=0; c<crop_dir.size(); ++c)
      //   {
      //     string crossnum = crop_dir[c].substr(crop_dir[c].length()-5, 1); // 文件名中的交叉点序号 -- yolo网络决定
      //     crosslist.push_back(stoi(crossnum));  // 交叉点序号, stoi()string转int
      //     Mat crop_img = imread(crop_dir[c]);
      //     if(crop_img.empty()){cout << "open crop_img error in function boxesCb" << endl;}
      //     sensor_msgs::ImagePtr crop_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", crop_img).toImageMsg();
      //     crop_pub.publish(crop_msg);
      //   }
      // }

    }
    // else if(crop_num<crop_dir.size())
    //   {
    //     ++crop_num;
    //     string crossnum = crop_dir[crop_num].substr(crop_dir[crop_num].length()-5, 1); // 文件名中的交叉点序号 -- yolo网络决定
    //     crosslist.push_back(stoi(crossnum));  // 交叉点序号, stoi()string转int
    //     Mat crop_img = imread(crop_dir[crop_num]);
    //     sensor_msgs::ImagePtr crop_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", crop_img).toImageMsg();
    //     crop_pub.publish(crop_msg);
    // }
  }