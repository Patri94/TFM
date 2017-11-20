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
#include <detector/c_detector.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <list>
#include <numeric>
#include <vector>
#include <string>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <XmlRpcException.h>
#include <detector/marker.h>
#include <detector/messagedet.h>
#include <curls/curl.h>
#include <Aria.h>


#define BLOCK_SIZE_FOR_ADAPTIVE_THRESHOLD 75
#define CELL_MARKER_SIZE 7
#define CURVE_SIZE 4

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace XmlRpc;

cRealProofs::cRealProofs (const ros::Nodehandle& nh, const ros::NodeHandle& nh_private)
    :nh_(nh),nh_private_(nh_private)
{
    //Publishing needed topics into master

    this->pub_scan=nh_.advertise<sensor_msgs::LaserScan> ("Doris/scan",1,true);
    this->pub_odom=nh_advertise<nav_msgs::Odometry> ("Doris/odom",i,true);

    image_transport::ImageTransport it(nh_);
    this->pub_image=it.advertise("Doris/cam",1,true);

    //Reading URL camera
    curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, cameraUrl.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &cRealProofs::writeData);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &cameraStream);

    //Reading laser
    laserConnector= new ArLaserConnector(rn->getArgumentParser(),rn->getRobot());
    if(!laserConnector->connectLasers(false, false, true)){
                    printf("Could not connect to configured lasers.\n");
    }

    laser= rn->getRobot()->findLaser(1);
    laserDataScan=NULL;

}

size_t cRealProofs::write_data(char *ptr, size_t size, size_t nmemb, void *userdata) {
        std::ostringstream *stream = (std::ostringstream*)userdata;
        size_t count = size * nmemb;
        stream->write(ptr, count);
        return count;
}


void cRealProofs::readImage(void){
    data.clear();
    cameraStream.str("");
    cameraStream.clear();

    res=curl_easy_perform(curl);
    std::string strCameraStream = cameraStream.str();
    std::copy(strCameraStream.begin(), strCameraStream.end(), std::back_inserter(data));

    if(data.size() > 0){
                    cv::Mat data_mat = cv::Mat(data);
                    frame = cv::Mat(cv::imdecode(data_mat, 1));
                    data_mat.release();
                    this->image=cv_bridge::CvImage(std_msgs::Header(),"bgr8",frame).toImageMsg();
                    pub_image.publish(this->image);
    }


}

void cRealProofs::readScan(void){
   if (laser !=NULL){
       laser->lockDevice();
       const std::list<ArSensorReading*> *currentReadings = new std::list<ArSensorReading*>(*laser->getRawReadings());

   if(laserDataScan == NULL){
               laserDataScan = new LaserScan();
           } else {
                     laserDataScan->clear();
   }
   for(std::list<ArSensorReading*>::const_iterator it = currentReadings->begin(); it != currentReadings->end(); ++it){
                           laserDataScan->addLaserScanData((float)(*it)->getRange() / 1000, (float)(*it)->getExtraInt());
                   }

           laser->unlockDevice();
           this->scan_msg.header.stamp=ros::Time::now();
           this->scan_msg.header.frame_id="Doris/laser_link";
           this->scan_msg.angle_min=laserDataScan->getAngleMin();
           this->scan_msg.angle_max=laserDataScan->getAngleMax();
           this->scan_msg.angle_increment=laserDataScan->getIncrement();
           this->scan_msg.ranges=laserDataScan->getRanges();
           this->scan_msg.intensities=laserDataScan->getRanges();

           //Cambiar con las especificaciones del laser.
           this->scan_msg.range_min=0.08;
           this->scan_msg.range_max=10;

           this->pub_scan.publish(scan_msg);




   delete currentReadings;
    }
}

void cRealProofs::Odometry(void){
   this->odom_pose= new ArPose(robot->getPose());

    x = myRawPose->getX() + deltaDistance * std::cos(myRawPose->getThRad() + (deltaDegrees / 2.0));
    y = myRawPose->getY() + deltaDistance * std::sin(myRawPose->getThRad() + (deltaDegrees / 2.0));
    th = myRawPose->getThRad() + deltaDegrees;

    tf::Quaternion Quat;
    tf::Matrix3x3 Mat;
    geometry_msgs::Quaternion QuatMs;
    Mat.setRPY(0,0,th);
    Mat.getRotation(Quat);
    tf::quaternionTFToMsg (Quat,QuatMs);
    this->odom_msg.frame_id="Doris/odom";
    this->odom_msg.child_frame_id="Doris/cuerpo";
    this->odom_msg.pose.pose.orientation=QuatMs;
    this->odom_msg.pose.pose.position.x=x;
    this->odom_msg.pose.pose.position.y=y;
    this->odom_msg.pose.pose.position.z=0;
    this->odom_msg.twist.twist.linear.x=robot->getVel();
    this->odom_msg.twist.twist.linear.z=robot->getrotVel();

    //Covariances
    float pose_cov [36] ;
    float twist_cov [36] ;

    for (int i=0; i<36 ;i++){
        pose_cov[i]=0;
        twist_cov[i]=0;
        if(i==0 || i==7){
            pose_cov[i]=0.00001;
        }
        if(i==14 || i==21 || i==28){
            pose_cov[i]=1000000000000.0;
        }
        if (i==36){
            pose_cov[i]=0.001;
        }
    }

    this->odom_msg.twist.covariance=twist_cov;
    this->odom_msg.pose.covariance=pose_cov;

    this->odom_pub.publish(this->odom_msg);



}

void RobotNode::computePositionFromEncoders(){
    double currentDistance = robot->getOdometerDistance();
    double currentRads = RNUtils::deg2Rad(robot->getOdometerDegrees());

    double currentVel = robot->getVel();
    double currentRotVel = robot->getRotVel();

    //RNUtils::printLn("odometer: {d: %lf, th: %lf}, vel: {lin: %lf, rot: %lf}", currentDistance, currentRads, robot->getVel(), robot->getRotVel());

    if(isFirstFakeEstimation){
        prevDistance = currentDistance;
        prevRads = currentRads;
        prevVel = currentVel;
        prevRotVel = currentRotVel;
        isFirstFakeEstimation = false;
    }

    deltaDistance = currentDistance - prevDistance;
    deltaDegrees = currentRads - prevRads;

    if(robot->getVel() < 0){
        deltaDistance *= -1.0;
    }

    if(currentRotVel < 0){
        if(deltaDegrees > 0){
            deltaDegrees *= -1.0;
        }
    }

    publishOdometry();
    //RNUtils::printLn("pose-Robot: {%f, %f, %f}, pose-Raw: {%f, %f, %f}, delta: {%f, %f}", robot->getPose().getX(), robot->getPose().getY(), robot->getPose().getThRad(), myRawPose->getX(), myRawPose->getY(), myRawPose->getThRad(), deltaDistance, deltaDegrees);
    prevVel = currentVel;
    prevRotVel = currentRotVel;
    prevDistance = currentDistance;
    prevRads = currentRads;

    robot->resetTripOdometer();

}

