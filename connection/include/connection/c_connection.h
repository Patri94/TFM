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
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

class cConnection{

public:
ros::NodeHandle nh_;
ros::NodeHandle nh_private_;
ros::Publisher laser_pub, odom_pub;
image_transport::Publisher image_pub;

//Socket
int socket_,socket_tcp;
int domain;
int type;
int protocol;
int port;
struct sockaddr_in addr;
struct sockaddr_in addr_udp;
unsigned short sa_family_; //AF_xxx
char sa_data_[14]; //protocol adress

//Camera
CURL* curl;
CURLcode res;
std::vector<char> data;
std::ostringstream cameraStream;
//const std::string cameraUrl;
cv::Mat frame;
sensor_msgs::ImagePtr image_msg;

//TF
 //laser
tf2_ros::StaticTransformBroadcaster static_broadcaster;
geometry_msgs::TransformStamped static_laserTransform;
geometry_msgs::TransformStamped static_cameraTransform;
sensor_msgs::LaserScan scan;

 //odometry
tf2_ros::TransformBroadcaster br;
geometry_msgs::TransformStamped odom;


cConnection(const ros::NodeHandle& nh,  const ros::NodeHandle& nh_private);
cConnection(): cConnection(ros::NodeHandle(), ros::NodeHandle("~") ){}
//~cConnection();

void readFromSocket();
void decoMessage(char message [], int size);
static size_t write_data(char *ptr, size_t size, size_t nmemb, void *userdata);
void readImage(void);
void siginthandler(int param);
};
