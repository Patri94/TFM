#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <functional>
#include <signal.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <list>
#include <numeric>
#include <vector>
#include <string>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <XmlRpcException.h>
#include <simulation/c_groundtruthtf.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/ccalib/omnidir.hpp>

using namespace std;

GroundTruth::GroundTruth ( const ros::NodeHandle& nh1,const ros::NodeHandle& nh_private1)
        :nh1_(nh1),nh_private1_(nh_private1)
{
    gtruth_subs=nh1_.subscribe<nav_msgs::Odometry> ("Doris/ground_truth/state",1,&GroundTruth::gTruthCallback,this);
}

void GroundTruth::gTruthCallback (const nav_msgs::Odometry::ConstPtr& msg){

        geometry_msgs::TransformStamped g_truth;

        g_truth.header.frame_id= "world";
        g_truth.child_frame_id= "Doris/cuerpo";
        g_truth.transform.translation.x=msg->pose.pose.position.x;
        g_truth.transform.translation.y=msg->pose.pose.position.y;
        g_truth.transform.translation.z=msg->pose.pose.position.z;
        g_truth.transform.rotation.x=msg->pose.pose.orientation.x;
        g_truth.transform.rotation.y=msg->pose.pose.orientation.y;
        g_truth.transform.rotation.z=msg->pose.pose.orientation.z;
        g_truth.transform.rotation.w=msg->pose.pose.orientation.w;
        g_truth.header.stamp=ros::Time::now();
        this->br.sendTransform(g_truth);


}
