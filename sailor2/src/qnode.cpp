/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/sailor2/qnode.hpp"
//#include "data"

#include <geometry_msgs/TransformStamped.h>//发布位姿的
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <ARToolKitPlus/TrackerSingleMarker.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace sailor2 {

/*****************************************************************************
** Implementation
*****************************************************************************/
void chatterCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv::Mat im_gray;
    //cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr;//opencv格式，cv_bridge是链接ros图像与opencv图像的桥梁
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//msg转换成opencv格式，方便处理
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //ROS_ASSERT(cv_ptr->image.channels()==3 || cv_ptr->image.channels()==1);

    cv_ptr->image.copyTo(im_raw);
}


QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {
    ros::init(init_argc,init_argv,"sailor2");
    ros::NodeHandle n;
    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    //image_sub=n.subscribe("/left/image_rect_color",1,&chatterCallback);
    image_sub=n.subscribe("/darknet_ros/detection_image",1,&chatterCallback);

}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();

    }
	wait();
}






}  // namespace sailor2
