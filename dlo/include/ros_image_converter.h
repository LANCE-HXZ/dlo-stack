#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

// darknet_ros_msgs
#include "dlo/BoundingBoxes.h"
#include "dlo/BoundingBox.h"
#include "strategy.h"

#include "ros/ros.h"
#include <boost/thread.hpp>
#include "image_transport/image_transport.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <string>

using namespace cv;
using namespace std;

class CImageConverter
{
  private:
    ros::NodeHandle m_nh;
    ros::Publisher m_pubCrop;
    ros::Publisher m_pubSrc;
    string IMG_FLODER, ROUND;
    CStrategy sttg;  // === 拆解策略 ===
  
  public:
    bool flagReady4Next;
    int m_nCropNum;
    bool flagCameraImgReady, flagBinaryImgReady, flagBoxesReady, flagCropClassReady, flagSkeletonReady;
    
    vector<string> m_vstrCropDir;
    Mat m_imgSrc, m_imgBinary, m_imgBinaryO, m_imgSkeleton, m_imgCamera, m_imgTraversal, m_imgYolo;
    dlo::BoundingBoxes::ConstPtr m_boxes, m_boxesCopy;

    CImageConverter();
    ~CImageConverter();

    void SensorMsgs2CvMat(const sensor_msgs::ImageConstPtr& msg, Mat& dst);
    void MakeConstantBorder(Mat& imgSrc, Mat& imgDst, int nEdge, Scalar color = Scalar(85, 120, 68));
    void ShowImg(String strWindowName, Mat &imgShow, bool x255 = 0);
    void ShowAll(Mat &imgD, Mat &imgB, Mat &imgS, Mat &imgT, Mat &imgV, Mat &imgRS, Mat &imgC1, Mat &imgC2);
    void MoveWindows();

    void CallbackCameraImgGet(const sensor_msgs::ImageConstPtr& msg);
    void CallbackBinaryImgGet(const std_msgs::String& msg);
    void CallbackBoxesGet(const dlo::BoundingBoxes::ConstPtr& boxes);
    void CallbackCropClassGet(const std_msgs::Int64::ConstPtr& cropclass);
    void Init();
    void ProcessShowCameraView();
    void ProcessSkeleton();
    void ProcessTraversal();
    void ProcessStrategy();
};