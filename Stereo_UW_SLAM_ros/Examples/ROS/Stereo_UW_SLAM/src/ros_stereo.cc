/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include "../../../include/Converter.h"
#include <sys/time.h>

using namespace std;
ofstream f;



class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    geometry_msgs::PoseStamped pose_msg;
    nav_msgs::Path path_msg;

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    
    f.open("/home/nvidia/Trajectory_ROS.txt");
    f << fixed;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;
//    while (ros::ok())
//    {
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/left/image_rect_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/right/image_rect_color", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);

    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
    ros::Time current_time;
    current_time=ros::Time::now();
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("PoseStamped", 1);

    igb.pose_msg.header.stamp=current_time;
    igb.path_msg.header.stamp=current_time;
    igb.pose_msg.header.frame_id="PoseStamped";
    igb.path_msg.header.frame_id="PoseStamped";

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("Path", 1);
    while (ros::ok())
    {
      pose_pub.publish(igb.pose_msg);
      path_pub.publish(igb.path_msg);
      ros::spinOnce();
    }

//    }
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    cv::Mat T;
    
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        T=mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
        cv::Mat Rwc = T.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*T.rowRange(0,3).col(3);
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
        timeval time;//get the day time 
        gettimeofday( &time, NULL );
        double stamped=time.tv_sec+(double)(time.tv_usec)/1000000;
        //cout<<stamped<<endl;
        //f << stamped<<" "<<setprecision(6) << twc.at<float>(0) << " " << twc.at<float>(1)  << " " << twc.at<float>(2) <<endl;
        f << stamped<<" "<<setprecision(6) << twc.at<float>(0) << " " << twc.at<float>(1)  << " " << twc.at<float>(2) << " "  << q[0] << " " << q[1] << " "  << q[2] << " " << q[3] << endl;
	//f <<setprecision(6) << twc.at<float>(0) << " " << twc.at<float>(1)  << " " << twc.at<float>(2) << " "  << q[0] << " " << q[1] << " "  << q[2] << " " << q[3] << endl;
        pose_msg.header.stamp=ros::Time::now();
        pose_msg.pose.position.x=twc.at<float>(2);
        pose_msg.pose.position.y=-twc.at<float>(0);
        pose_msg.pose.position.z=twc.at<float>(1);
        pose_msg.pose.orientation.x=q[2];
        pose_msg.pose.orientation.y=q[0];
        pose_msg.pose.orientation.z=q[1];
        pose_msg.pose.orientation.w=q[3];
        path_msg.poses.push_back(pose_msg);
        //路径
    }
    else
    {
        T=mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
        cv::Mat Rwc = T.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*T.rowRange(0,3).col(3);
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
	timeval time;//get the day time 
        gettimeofday( &time, NULL );
	double stamped=time.tv_sec+(double)(time.tv_usec)/1000000;
        //cout<<stamped<<endl;
	//f << stamped<<" "<<setprecision(6) << twc.at<float>(0) << " " << twc.at<float>(1)  << " " << twc.at<float>(2) <<endl;
        f << stamped<<" "<<setprecision(6) << twc.at<float>(0) << " " << twc.at<float>(1)  << " " << twc.at<float>(2) << " "  << q[0] << " " << q[1] << " "  << q[2] << " " << q[3] << endl;
        pose_msg.header.frame_id="PoseStamped";
        pose_msg.header.stamp=ros::Time::now();
        pose_msg.pose.position.x=twc.at<float>(2);
        pose_msg.pose.position.y=twc.at<float>(0);
        pose_msg.pose.position.z=twc.at<float>(1);
        pose_msg.pose.orientation.x=q[2];
        pose_msg.pose.orientation.y=q[0];
        pose_msg.pose.orientation.z=q[1];
        pose_msg.pose.orientation.w=q[3];
        //路径
        path_msg.poses.push_back(pose_msg);
    }

}


