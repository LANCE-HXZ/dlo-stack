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
    m_imgCamera = cv::Mat::zeros(640, 800, CV_8UC3);      //  图片变量初始化
    flagCameraImgReady = flagBinaryImgReady = flagBoxesReady = flagCropClassReady = flagSkeletonReady = 0;
    flagReady4Next = 1;       //  是否准备好处理下一张图片
    m_imgAll = Mat::zeros(1440, 2560, CV_8UC3);
}

CImageConverter::~CImageConverter()
{
// cv::destroyWindow("OPENCV_WINDOW");
}

/*  每次执行完整流程之前进行参数初始化  */
void CImageConverter::Init()
  {
    cout << "NOW IN Init()" << endl;
    ROUND = nameWithTime();
    cout << ROUND << endl;
    g_vnClassList = g_vnCrossList = {}; m_vstrCropDir = {};
    m_nCropNum = 0; flagBinaryImgReady = flagReady4Next = 0;
    m_imgSrc = m_imgCamera;
    MakeConstantBorder(m_imgSrc, m_imgSrc, EDGE);
    imwrite(IMG_FLODER + "1_R.png", m_imgSrc);
    ShowAll(m_imgSrc, 0);
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
    Mat imgBinary = readImg(IMG_FLODER + "3_O.png");
    ShowAll(imgBinary, 8);
    Mat imgY = readImg(IMG_FLODER + "3_Y.png");
    ShowAll(imgY, 4);
    rgb2binary(imgBinary, imgBinary);
    imwrite(IMG_FLODER + "4_B.png", imgBinary);
    ShowAll(imgBinary, 2);
    // imgBinary = readImg(IMG_FLODER + "t.png");    //  直接测试图像细化
    // resize(imgBinary, imgBinary, Size(640,320));
    pre_dilate(imgBinary, 4, 1); // 膨胀去除黑离群点
    imwrite(IMG_FLODER + "4_B2_dilate.png", imgBinary);
    ShowAll(imgBinary, 1);
    pre_erode(imgBinary, 3, 5); // 腐蚀去除白离群点
    imwrite(IMG_FLODER + "4_B3_erode.png", imgBinary);
    ShowAll(imgBinary, 5);
    skeleton(imgBinary, IMG_FLODER + "5_S.png", 3);
    Mat imgSkeleton = readImg(IMG_FLODER + "5_S.png");
    ShowAll(imgSkeleton, 9);
    imgSkeleton = removeSinglePoint(imgSkeleton, 30, 30);
    imgSkeleton = removeSinglePoint(imgSkeleton, 60, 60);
    imgSkeleton = removeSinglePoint(imgSkeleton, 40, 40);
    imgSkeleton = removeSinglePoint(imgSkeleton, 90, 90);
    imgSkeleton = removeSinglePoint(imgSkeleton, 10, 10);
    imwrite(IMG_FLODER+"5_S2.png", imgSkeleton);
    // ShowAll(imgSkeleton, 6);
}

/*  在细化图和目标识别boxes准备好后进入遍历流程  */
void CImageConverter::ProcessTraversal(){
    //  新的端点检测
    Mat imgSkeleton = readImg(IMG_FLODER + "5_S2.png");
    // fix_cross_error(imgSkeleton);
    // imgSkeleton*=255;
    // imshow("imgSkeleton", imgSkeleton);
    // waitKey();
    //  端点检测
    vector<Point> endpoints, crossings;
    endPointAndintersectionPointDetection(imgSkeleton, endpoints, crossings);
    //画出端点
    for (vector<Point>::iterator i = endpoints.begin(); i != endpoints.end(); ++i)
    {
        circle(imgSkeleton, Point(i->x, i->y), 3, Scalar(0, 255, 0), 0);
    }
    // imwrite(IMG_FLODER+"5_Se.png", imgSkeleton);
    // for (vector<Point>::iterator i = crossings.begin(); i != crossings.end(); ++i)
    // {
    //     circle(imgSkeleton, Point(i->x, i->y), 5, Scalar(0, 255, 255), -1);
    // }
    // cout << "11111111111\n";
    ShowAll(imgSkeleton, 6);
    // cout << "222222222222\n";

    m_boxesCopy = m_boxes;
    Mat imgYolo = readImg(IMG_FLODER + "2_D.png");
    ShowAll(imgYolo, 10);
    
    m_vstrCropDir = traversal(IMG_FLODER + "5_S2.png", m_imgSrc, m_boxes, endpoints); // 裁剪图路径列表 -- 按遍历顺序
    Mat imgTraversal = readImg(IMG_FLODER + "6_T.png");
    ShowAll(imgTraversal, 3);
    if(m_vstrCropDir.size()==0){    //  遍历后无交叉点
        flagCropClassReady = 1;
        return;
    }

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
    Mat imgVisualization = readImg(IMG_FLODER + "7_V.png");
    ShowAll(imgVisualization, 7);
    Mat imgResult = readImg(IMG_FLODER + "8_Result.png");
    ShowAll(imgResult, 11);
    manipulation(oprt);

    /*  将对应交叉点识别结果成对相反出现的交叉点框分别保存到0/和1/训练集文件夹  */
    cout << "\n\nSAVING CORRECT CROSSINGS: \n";
    vector<bool> vbSave(cross.size()*2, 1);
    for(int i = 0; i < cross.size(); ++i){
		if(c0[i] == -1 || c1[i] == -1)	continue;
        cout << i << "-" << c0[i] << "-" << c1[i] << "\t\t";
		Mat imgTrainCross0 = imread(m_vstrCropDir[c0[i]]);
        Mat imgTrainCross1 = imread(m_vstrCropDir[c1[i]]);
        imwrite(IMG_FLODER+"0/"+ROUND+"_"+to_string(i)+".png", imgTrainCross0);
        imwrite(IMG_FLODER+"1/"+ROUND+"_"+to_string(i)+".png", imgTrainCross1);
        vbSave[c0[i]] = vbSave[c1[i]] = 0;
    }
    cout << "\nSAVING ERROR CROSSINGS: \n";
    for(int j = 0; j < vbSave.size(); ++j){
        if(vbSave[j]){
            cout << g_vnCrossList[j] << "_" << g_vnClassList[j] << "\t\t";
            Mat imgTrainCross = imread(m_vstrCropDir[j]);
            imwrite(IMG_FLODER+"2/"+ROUND+"_"+to_string(g_vnCrossList[j])+"_"+to_string(g_vnClassList[j])+"_"+to_string(j)+".png", imgTrainCross);
        }
    }
    cout << endl;
    exit(0);
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

void CImageConverter::ShowAll(Mat imgIn, int n){
    int c = n%4, r = n/4, x = c*640, y = r*480;
    CvRect rect = cvRect(x, y, imgIn.cols-2*EDGE, imgIn.rows-2*EDGE);
    imgIn.colRange(EDGE, imgIn.cols-EDGE).rowRange(EDGE, imgIn.rows-EDGE).copyTo(m_imgAll(rect));
    imwrite(IMG_FLODER + "0_All.png", m_imgAll);
    imwrite(IMG_FLODER + "S/" + ROUND + ".png", m_imgAll);
}
