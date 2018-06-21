#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp> 
#include <math.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

void chatterCallback(const sensor_msgs::ImageConstPtr& msg)//一旦收到传感器信息msg,执行回调函数
{
   
   
    Mat im_raw;
/*CvImagePtr cv_bridge::toCvCopy(const sensor_msgs::ImageConstPtr & source,
 const std::string & encoding = std::string())*/
 //函数原型rosimage
/*class CvImage
{
    sensor_msgs::ImagePtr toImageMsg()const;
    void toImageMsg(sensor_msgs::Image& ros image)const;
};*/
    //这里主要是为了把ROS的图像消息里面的数据提取出来,边长 cv::MAT
     cv_bridge::CvImagePtr cv_ptr;//opencv格式，cv_bridge是链接ros图像与opencv图像的桥梁
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);//msg转换成opencv格式，方便处理
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

    
    if(cv_ptr->image.channels()==1)
    {
       cv_ptr->image.copyTo(im_raw);
       //ushort *data=im_raw.ptr<ushort>
       //uint16_t d=im_raw.at<uint16_t>(672/2,376/2);
       //float d=im_raw.at<float>(im_raw.cols/2,im_raw.rows/2);
       float d=im_raw.at<float>(im_raw.rows/1.3,im_raw.cols/2);
       std::cout<<d<<std::endl;
       imshow("sz",im_raw);
       waitKey(3);
    }
}
int main(int argc, char **argv){
    //ROS节点初始化
  ros::init(argc, argv, "depth_value");
  ros::NodeHandle nh;
  //订阅图像消息 /usb_cam/image_raw or /rgb/image_raw
  ros::Subscriber sub = nh.subscribe("/depth/depth_registered", 1, &chatterCallback);
  ros::spin();
  return 0;
}
