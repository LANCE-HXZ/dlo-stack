#include "ros_image_converter.h"
#include "img_processing.h"
#include "dlo.h"

/*  构造函数    */
CImageConverter::CImageConverter()
{
    IMG_FLODER = "pic_buffer/";
    m_nCropNum = 0;
    m_pubCrop = m_nh.advertise<std_msgs::String>("crop_topic", 1000);  // 发布旋转裁剪图信号
    m_pubSrc = m_nh.advertise<std_msgs::String>("rgb_topic", 1000);  // 发布已经获取到的rgb图像信号
    m_imgBinary = m_imgSkeleton = m_imgYolo = m_imgTraversal = m_imgCamera = imread(IMG_FLODER + "init/black.png");      //  图片变量初始化
    flagCameraImgReady = flagBinaryImgReady = flagBoxesReady = flagCropClassReady = flagSkeletonReady = 0;
    m_bBinaryImgGet = 0;
    flagReady4Next = 1;       //  是否准备好处理下一张图片
}

CImageConverter::~CImageConverter()
{
// cv::destroyWindow("OPENCV_WINDOW");
}

/*  每次执行完整流程之前进行参数初始化  */
void CImageConverter::Init()
  {
    cout << "NOW IN Init()" << endl;
    g_vnClassList = g_vnCrossList = {}; m_vstrCropDir = {};
    m_nCropNum = 0; flagBinaryImgReady = flagReady4Next = 0;
    m_imgSrc = m_imgCamera;
    MakeConstantBorder(m_imgSrc, m_imgSrc, EDGE);
    imwrite(IMG_FLODER + "1_R.png", m_imgSrc);
    imshow("R", m_imgSrc);
    waitKey();
    std_msgs::String signal;
    signal.data = "1";
    m_pubSrc.publish(signal);
    cout << "Published rgbimg successed!\n";
  }

/*  显示相机图片和初始化参数  */
void CImageConverter::ProcessShowCameraView(){
    if(flagReady4Next)
      Init();
}

/*  在二值轮廓图准备好后进入细化流程  */
void CImageConverter::ProcessSkeleton(){
    if(!m_bBinaryImgGet)
    {
      m_bBinaryImgGet = 1;
      m_imgBinary = imread(IMG_FLODER + "3_O.png");
      rgb2binary(m_imgBinary, m_imgBinary);
      imwrite(IMG_FLODER+"4_B.png", m_imgBinary);
      m_imgBinary = removeSinglePoint(m_imgBinary, 3, 15);
      imwrite(IMG_FLODER+"4_B1_RemoveOutlier.png", m_imgBinary);
      m_imgSkeleton = skeleton(m_imgBinary, IMG_FLODER + "5_S.png", 3);
    }
    else
    {
      m_imgBinaryO = imread(IMG_FLODER + "3_O");
      m_imgBinaryO *= 256;
    }
}

/*  在细化图和目标识别boxes准备好后进入遍历流程  */
void CImageConverter::ProcessTraversal(){
    m_boxesCopy = m_boxes;
    m_imgYolo = imread(IMG_FLODER + "2_D.png");
    
    m_vstrCropDir = traversal(IMG_FLODER + "5_S.png", m_imgSrc, m_boxes); // 裁剪图路径列表 -- 按遍历顺序
    m_imgTraversal = imread(IMG_FLODER + "6_T.png");
    
    for(int c=0; c<m_vstrCropDir.size(); ++c)
    {
      string crossnum = m_vstrCropDir[c].substr(m_vstrCropDir[c].length()-5, 1); // 文件名中的交叉点序号 -- yolo网络决定
      g_vnCrossList.push_back(stoi(crossnum));  // 交叉点序号, stoi()string转int
      std_msgs::String msgsCropDir;
      msgsCropDir.data = m_vstrCropDir[c];
      m_pubCrop.publish(msgsCropDir);
    }
}

/*  在交叉点类型识别完整后进入拆解策略和执行流程  */
void CImageConverter::ProcessStrategy(){
    visualization();  // === 可视化 ===
      int checkstate = strategy();  // === 拆解策略 ===
      // ShowImg("m_imgBinary", m_imgBinary);
      // ShowImg("m_imgSkeleton", m_imgSkeleton);
      // ShowImg("m_imgYolo", m_imgYolo);
      // ShowImg("Traversal", m_imgTraversal);
      // MoveWindows();
      if(!checkstate){
        flagReady4Next = 1;
        cout << "get ready for next\n";
      }
      else{
        flagReady4Next = 1;
        cout << "get ready for next\n";
      }
}

/*  ros订阅者通过cv_bridge接收的图片话题消息需要转换为cv格式
    输入: msg(订阅者接收的图片msg), dst(输出图) */
void CImageConverter::SensorMsgs2CvMat(const sensor_msgs::ImageConstPtr& msg, Mat& dst){
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

/*  为图片四边增加单色的边框, 增加了图片的宽和高
    输入: m_imgSrc(原图), imgDst(输出图), nEdge(增加的边框宽度), color(边框颜色, 默认为绿布色)    */
void CImageConverter::MakeConstantBorder(Mat& imgSrc, Mat& imgDst, int nEdge, Scalar color){
    copyMakeBorder(imgSrc, imgDst, nEdge, nEdge, nEdge, nEdge, BORDER_CONSTANT, color);
}
  
/*  imshow打印图片
    输入: 窗口命名, 需要展示的图片, x255(为true时表示输入图片的像素值最大为1, 需要进行*=255操作)    */
void CImageConverter::ShowImg(String strWindowName, Mat &imgShow, bool x255){
    if(x255)
        imgShow *= 255;
    namedWindow(strWindowName, WINDOW_AUTOSIZE);
    imshow(strWindowName, imgShow);
}

/*  移动显示窗口的位置  */
void CImageConverter::MoveWindows(){
    moveWindow("m_imgCamera", 2000, 400);
    moveWindow("m_imgBinary", 2000, 1000);
    moveWindow("Darknet", 0, 320);
    moveWindow("Traversal", 855, 320);
    moveWindow("visual_img", 0, 880);
    moveWindow("Result", 855, 880);
    waitKey();
}

