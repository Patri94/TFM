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

class cRealProofs{
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher  pub_odom, pub_cam, pub_scan;

    //Camera
    CURL* curl;
    CURLcode res;
    std::vector<char> data;
    std::ostringdtream cameraStream;
    static const std::string cameraUrl;
    cv::Mat frame;
    sensor_msgs::ImagePtr image;

    //Laser
    LaserScan *laserDataScan;
    ArLaserConnector *laserConnector;
    ArLaser *laser;

    //Odometry
    ArRobotConnector *connector;
    ArRobot *robot;
    ArPose *myRawPose;
    double prevDistance;
    double prevRads;
    double prevVel;
    double prevRotVel;

};
