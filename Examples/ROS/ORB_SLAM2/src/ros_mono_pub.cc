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

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl_ros/point_cloud.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

ros::Publisher cloud_pub_vp;
sensor_msgs::PointCloud2 cloud_ros_vp;
pcl::PointCloud<pcl::PointXYZ> cloud_pcl_vp;
ros::Publisher cloud_pub_vpRef;
sensor_msgs::PointCloud2 cloud_ros_vpRef;
pcl::PointCloud<pcl::PointXYZ> cloud_pcl_vpRef;

ros::Publisher kf_pub;
geometry_msgs::PoseArray kf_pt_array;
//geometry_msgs::Pose camera_pose;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_Pub");
    ros::start();

    /*point cloud library*/
    cloud_pcl_vp.width  = 5000;
    cloud_pcl_vp.height = 1;
    cloud_pcl_vp.points.resize(cloud_pcl_vp.width * cloud_pcl_vp.height);
    cloud_pcl_vpRef.width  = 10000;
    cloud_pcl_vpRef.height = 1;
    cloud_pcl_vpRef.points.resize(cloud_pcl_vpRef.width * cloud_pcl_vpRef.height);

    ros::NodeHandle nh;
    cloud_pub_vp = nh.advertise<sensor_msgs::PointCloud2>("cloud_vp", 1000);
    cloud_pub_vpRef = nh.advertise<sensor_msgs::PointCloud2>("cloud_vpRef", 1000);
    kf_pub = nh.advertise<geometry_msgs::PoseArray>("kf", 1000);

    /*tf*/
    tf2_ros::StaticTransformBroadcaster tb;
    geometry_msgs::TransformStamped ts;

    //time
	ts.header.stamp = ros::Time::now();
	ts.header.frame_id = "start_point";
    ts.child_frame_id = "base_link";

    tf2::Vector3 translation;
	translation.setValue(0, 0, 0);
	ts.transform.translation.x = translation.x();
	ts.transform.translation.y = translation.y();
	ts.transform.translation.z = translation.z();
	
	tf2::Quaternion rotation;
	rotation.setRPY(-1.57, 0, 0);
	ts.transform.rotation.x = rotation.x();
	ts.transform.rotation.y = rotation.y();
	ts.transform.rotation.z = rotation.z();
	ts.transform.rotation.w = rotation.w();
	
	tb.sendTransform(ts);

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    //ros::Subscriber sub = nodeHandler.subscribe("/webcam/image_raw", 1000, &ImageGrabber::GrabImage,&igb);
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    ORB_SLAM2::Map* mpMap = mpSLAM->Read_Map();
    
    if(!Tcw.empty()){   
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

        geometry_msgs::Pose camera_pose;

        camera_pose.position.x = twc.at<float>(0);
        camera_pose.position.y = twc.at<float>(1);
        camera_pose.position.z = twc.at<float>(2);

        camera_pose.orientation.x = q[0];
        camera_pose.orientation.y = q[1];
        camera_pose.orientation.z = q[2];
        camera_pose.orientation.w = q[3];

        kf_pt_array.poses.push_back(camera_pose);
        kf_pt_array.header.frame_id = "base_link";
        kf_pt_array.header.stamp = ros::Time::now();
        kf_pub.publish(kf_pt_array);
    }
    
    const vector<ORB_SLAM2::MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<ORB_SLAM2::MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<ORB_SLAM2::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
    
    cloud_pcl_vp.width  = vpMPs.size();
    cloud_pcl_vp.height = 1;
    cloud_pcl_vp.points.resize(vpMPs.size());
    
    
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        cloud_pcl_vp.points[i].x = pos.at<float>(0);
        cloud_pcl_vp.points[i].y = pos.at<float>(1);
        cloud_pcl_vp.points[i].z = pos.at<float>(2);
    }

    pcl::toROSMsg(cloud_pcl_vp, cloud_ros_vp);  
    cloud_ros_vp.header.frame_id = "base_link";
    cloud_ros_vp.header.stamp = ros::Time::now();
    cloud_pub_vp.publish(cloud_ros_vp);
    
    /*
    cloud_pcl_vpRef.width  = spRefMPs.end();
    cloud_pcl_vpRef.height = 1;
    cloud_pcl_vpRef.points.resize(spRefMPs.end());
    */
    int i = 0;
    for(set<ORB_SLAM2::MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        cloud_pcl_vpRef.points[i].x = pos.at<float>(0);
        cloud_pcl_vpRef.points[i].y = pos.at<float>(1);
        cloud_pcl_vpRef.points[i].z = pos.at<float>(2);
        i++;
    }
    pcl::toROSMsg(cloud_pcl_vpRef, cloud_ros_vpRef);
    cloud_ros_vpRef.header.frame_id = "base_link";
    cloud_ros_vpRef.header.stamp = ros::Time::now();
    cloud_pub_vpRef.publish(cloud_ros_vpRef);
}