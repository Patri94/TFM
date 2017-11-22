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
#include <math.h>
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
#include <tf/tf.h>
#include <curl/curl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <real_proofs/c_real.h>

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace XmlRpc;

#define PI 3.141592

cRealProofs::cRealProofs( const ros::NodeHandle& nh,const ros::NodeHandle& nh_private)
    :nh_(nh),nh_private_(nh_private)
{
    //int port;
   // const char address;
    nh_.getParam("Doris_Connection/domain",this->domain);
    nh_.getParam("Doris_Connection/type",this->type);
    nh_.getParam("Doris_Connection/protocol",this->protocol);

    std::string data;
    int family;
    nh_.getParam("Doris_Connection/address",data);
    nh_.getParam("Doris_Connection/port",this->port);
    nh_.getParam("Doris_Connection/family",family);

    this->addr.sa_family=family;

    strcpy(this->addr.sa_data,data.c_str());
    //this->addr.sa_data=this->sa_data_;

    //initializing socket
    this->socket_=socket(domain,type,protocol);

    //binding socket
    int connection=bind(this->socket_, (const struct sockaddr *) &addr,  sizeof(this->addr));


    //Publishing needed topics into master
    this->pub_scan=nh_.advertise<sensor_msgs::LaserScan> ("Doris/scan",1,true);
    this->pub_odom=nh_.advertise<nav_msgs::Odometry> ("Doris/odom",1,true);
    image_transport::ImageTransport it(nh_);
    this->pub_cam=it.advertise("Doris/cam",1,true);


    //Reading URL camera
    curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, cameraUrl.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &cRealProofs::write_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &cameraStream);


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
                    pub_cam.publish(this->image);
    }


}

void cRealProofs::readFromSocket(void){
     int valread;
     char buffer[368];
    //Establishing queue
     listen(this->socket_,3);

     //Connecting to socket
     connect(this->socket_, (const struct sockaddr *) &addr , sizeof(this->addr));

     //Reading from socket
     valread=recvfrom(this->socket_,buffer,368,0,  (struct sockaddr *) &addr, sizeof(this->addr) <0);

     //imprimir por pantalla
     printf(buffer);
}

