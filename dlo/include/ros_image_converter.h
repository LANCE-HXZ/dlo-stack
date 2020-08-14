#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

// darknet_ros_msgs
#include "dlo/BoundingBoxes.h"
#include "dlo/BoundingBox.h"

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

class ImageConverter
{
  private:
    ros::NodeHandle nh_;
    // ros::Subscriber boxes_sub;
    // ros::Subscriber crop_class_sub;
    ros::Publisher crop_pub;
    ros::Publisher rgb_pub;
  
  public:
    bool bw_got, getready;
    int crop_num;
    string strImgFloder;
    vector<string> crop_dir;
    Mat rgb_img, bw_img, bw_imgo, imgSkeleton, camera_img, imgTraversal, yolo_img;
    clock_t now, start, end; // time
    dlo::BoundingBoxes::ConstPtr boxes_copy;

    ImageConverter();
    ~ImageConverter();

    void cameraCb(const sensor_msgs::ImageConstPtr& msg);
    void init();
    // void bwCb(const sensor_msgs::ImageConstPtr& msg);
    void bwCb(const std_msgs::String& msg);
    void boxesCb(const dlo::BoundingBoxes::ConstPtr& boxes);
    void cropclassCb(const std_msgs::Int64::ConstPtr& cropclass);
    void show_img(String strWindowName, Mat &imgShow, bool x255 = 0);
    void move_windows();
    void sensorMsgs_to_cvMat(const sensor_msgs::ImageConstPtr& msg, Mat& dst);

};