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
#include <math.h>
#include <functional>
#include <iostream>
#include <list>
#include <numeric>
#include <vector>
#include <string>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <curl/curl.h>
#include <limits>
#include <tf/tf.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>

class cRealProofs{
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher  pub_odom,  pub_scan;
    image_transport::Publisher pub_cam;

    //Socket
    int socket_;
    int domain;
    int type;
    int protocol;
    int port;
    struct sockaddr addr;
    unsigned short sa_family_; //AF_xxx
    char sa_data_[14]; //protocol adress



    //Camera
    CURL* curl;
    CURLcode res;
    std::vector<char> data;
    std::ostringstream cameraStream;
    static const std::string cameraUrl;
    cv::Mat frame;
    sensor_msgs::ImagePtr image;

    //Laser
    sensor_msgs::LaserScan scan_msg;

    //Odometry
    nav_msgs::Odometry odom_msg;

    //Constructor and Destructor
    cRealProofs(const ros::NodeHandle& nh,  const ros::NodeHandle& nh_private);
    cRealProofs(): cRealProofs(ros::NodeHandle(), ros::NodeHandle("~") ){}



    //functions
    size_t write_data(char *ptr, size_t size, size_t nmemb, void *userdata);
    void readImage(void);
    void readFromSocket(void);
};
