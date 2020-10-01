#include "ros_image_converter.h"
#include "dlo.h"
#include "dlo_global.h"
#include "manipulation.h"


/*  构造函数    */
CImageConverter::CImageConverter()
{
    srand((int)time(0));
    CStrategy sttg;  // === 拆解策略 ===
    IMG_FLODER = "pic_buffer/";
    m_nCropNum = 0;
    m_pubCrop = m_nh.advertise<std_msgs::String>("crop_topic", 1000);  // 发布旋转裁剪图信号
    m_pubSrc = m_nh.advertise<std_msgs::String>("rgb_topic", 1000);  // 发布已经获取到的rgb图像信号
    m_imgBinary = m_imgSkeleton = m_imgYolo = m_imgTraversal = m_imgCamera;      //  图片变量初始化
    flagCameraImgReady = flagBinaryImgReady = flagBoxesReady = flagCropClassReady = flagSkeletonReady = 0;
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
    // imshow("R", m_imgSrc);
    // waitKey();
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
    m_imgBinary = readImg(IMG_FLODER + "3_O.png");

    rgb2binary(m_imgBinary, m_imgBinary);
    imwrite(IMG_FLODER+"4_B.png", m_imgBinary);

    pre_dilate(m_imgBinary, 3, 2); // 膨胀去除黑离群点
    cv::imwrite(IMG_FLODER + "4_B2_dilate.png", m_imgBinary);

    pre_erode(m_imgBinary, 3, 6); // 腐蚀去除白离群点
    cv::imwrite(IMG_FLODER + "4_B3_erode.png", m_imgBinary);

    skeleton(m_imgBinary, IMG_FLODER + "5_S.png", 3);

    m_imgSkeleton = readImg(IMG_FLODER + "5_S.png");
    m_imgSkeleton = removeSinglePoint(m_imgSkeleton, 30, 30);
    m_imgSkeleton = removeSinglePoint(m_imgSkeleton, 60, 60);
    m_imgSkeleton = removeSinglePoint(m_imgSkeleton, 40, 60);
    m_imgSkeleton = removeSinglePoint(m_imgSkeleton, 90, 90);
    m_imgSkeleton = removeSinglePoint(m_imgSkeleton, 10, 10);
    imwrite(IMG_FLODER+"5_S.png", m_imgSkeleton);
}

/*  在细化图和目标识别boxes准备好后进入遍历流程  */
void CImageConverter::ProcessTraversal(){
    m_boxesCopy = m_boxes;
    m_imgYolo = readImg(IMG_FLODER + "2_D.png");
    
    m_vstrCropDir = traversal(IMG_FLODER + "5_S.png", m_imgSrc, m_boxes); // 裁剪图路径列表 -- 按遍历顺序
    m_imgTraversal = readImg(IMG_FLODER + "6_T.png");
    
    for(int c=0; c<m_vstrCropDir.size(); ++c)
    {
      string crossnum = m_vstrCropDir[c].substr(m_vstrCropDir[c].length()-6, 2); // 文件名中的交叉点序号 -- yolo网络决定
      g_vnCrossList.push_back(stoi(crossnum));  // 交叉点序号, stoi()string转int, 首个非零位前的零会自动省略
      std_msgs::String msgsCropDir;
      msgsCropDir.data = m_vstrCropDir[c];
      m_pubCrop.publish(msgsCropDir);
    }
}

/*  在交叉点类型识别完整后进入拆解策略和执行流程  */
void CImageConverter::ProcessStrategy(){
    visualization();  // === 可视化 ===
    SOperation oprt = sttg.strategy();
    manipulation(oprt);

    /*  将对应交叉点识别结果成对相反出现的交叉点框分别保存到0/和1/训练集文件夹  */
    cout << "SAVING CROSS: \n";
    for(int i = 0; i < cross.size(); ++i){
		if(c0[i] == -1 || c1[i] == -1)	continue;
        cout << i << "\t";
		Mat imgTrainCross0 = imread(m_vstrCropDir[c0[i]]);
        Mat imgTrainCross1 = imread(m_vstrCropDir[c1[i]]);
        imwrite(IMG_FLODER+"0/"+to_string(rand())+".png", imgTrainCross0);
        imwrite(IMG_FLODER+"1/"+to_string(rand())+".png", imgTrainCross1);
    }
    waitKey();
    // ShowImg("m_imgBinary", m_imgBinary);
    // ShowImg("m_imgSkeleton", m_imgSkeleton);
    // ShowImg("m_imgYolo", m_imgYolo);
    // ShowImg("Traversal", m_imgTraversal);
    // MoveWindows();
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

