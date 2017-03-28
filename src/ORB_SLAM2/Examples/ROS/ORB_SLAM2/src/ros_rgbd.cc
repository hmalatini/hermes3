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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/TransformStamped.h"
#include "../../../include/System.h"
#include "../../../include/KeyFrame.h"
#include "../../../include/Converter.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;

    tf::TransformBroadcaster br;

    //For use with rtabmap_ros
    ros::Publisher odom_pub;
    ros::Rate* r = new ros::Rate(30.0);
    ros::Time current_time;
    ros::Time last_time;
    ros::Publisher getOdomPub(void){return odom_pub;};
    void setOdomPub(ros::Publisher o){odom_pub=o;};
    ros::Time getCurrentTime(void){return current_time;};
    void setCurrentTime(ros::Time ct){current_time=ct;};
    ros::Time getLastTime(void){return last_time;};
    void setLastTime(ros::Time lt){last_time=lt;};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    ros::Publisher odom_publisher;

    odom_publisher = nh.advertise<nav_msgs::Odometry>("odom", 20);

    igb.setOdomPub(odom_publisher);

    igb.setCurrentTime(ros::Time::now());
    igb.setLastTime(ros::Time::now());

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    ORB_SLAM2::KeyFrame* pointKeyFrame = mpSLAM->GetLastKeyFrame();

    static cv::Mat rotation;
    static vector<float> quaternion;
    static cv::Mat translation;

    if(!(pointKeyFrame->isBad())){
        rotation = pointKeyFrame->GetRotation().t();
        quaternion = ORB_SLAM2::Converter::toQuaternion(rotation);
        translation = pointKeyFrame->GetCameraCenter();

        /*printf("Rotacion: %f %f %f %f\n", quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
        printf("Translacion: %f %f %f\n", translation.at<float>(0), translation.at<float>(1), translation.at<float>(2));
        printf("---------------------------------------------------------------");*/
    } else {
        return;
    }

    //Last position
    static float last_pos_x = 0;
    static float last_pos_y = 0;
    static float last_pos_angular = 0;

    ros::Time current_time = ros::Time::now();

    //Variables for odometry
    double dt = (current_time - last_time).toSec();
    double vel_lineal_x = 0;
    double vel_lineal_y = 0;
    double vel_angular_z = 0;
    if(dt != 0){
        vel_lineal_x = ((translation.at<float>(0) * 5) - last_pos_x) / dt;
        vel_lineal_y = ((translation.at<float>(2) * 5) - last_pos_y) / dt;
        vel_angular_z = ((quaternion[1] * 2.3) - last_pos_angular) / dt;
    }

    //TF MESSAGE
    tf::Transform transform;
    //transform.setOrigin( tf::Vector3(translation.at<float>(0) * 5, translation.at<float>(2) * 5, -1 * (translation.at<float>(1) * 5)));
    transform.setOrigin( tf::Vector3(translation.at<float>(0) * 5, translation.at<float>(2) * 5, 0));

    tf::Quaternion q;
    //q.setRPY(quaternion[0] * 2.3, quaternion[2] * 2.3, -1 * quaternion[1] * 2.3);
    q.setRPY(0, 0, -1 * quaternion[1] * 2.3);
    transform.setRotation(q);

    ImageGrabber::br.sendTransform(tf::StampedTransform(transform, current_time, "odom", "base_link"));

    //Publicamos mensajes odometricos para algoritmo rtabmap_ros
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = translation.at<float>(0) * 5;
    odom.pose.pose.position.y = translation.at<float>(2) * 5;
    odom.pose.pose.position.z = 0.0;

    odom.twist.twist.linear.x = vel_lineal_x;
    odom.twist.twist.linear.y = vel_lineal_y;
    odom.twist.twist.angular.z = vel_angular_z;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -1 * quaternion[1] * 2.3);
    odom.pose.pose.orientation = odom_quat;

    ImageGrabber::odom_pub.publish(odom);

    ImageGrabber::last_time = ImageGrabber::current_time;
    last_pos_x = translation.at<float>(0) * 5;
    last_pos_y = translation.at<float>(2) * 5;
    last_pos_angular = quaternion[1] * 2.3;

    ImageGrabber::r->sleep();
}