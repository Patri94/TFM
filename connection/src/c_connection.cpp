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
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <curl/curl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <connection/c_connection.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.141592
using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace XmlRpc;

cConnection::cConnection (const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
:nh_(nh),nh_private_(nh_private)
{
    //Initializing TCP and UDP sockets
    addr.sin_port=htons(14004);
    inet_pton(AF_INET,"192.168.1.101",& (addr.sin_addr));

    this->socket_tcp=socket(AF_INET,SOCK_STREAM,0);
    this->socket_=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);

    //connecting sockets
    int result=connect(this->socket_tcp, (sockaddr *) &addr, sizeof(this->addr));
    int connection=bind(this->socket_, (const struct sockaddr *) &addr,  sizeof(this->addr));

    //Publishing laser_data
    this->laser_pub=nh_.advertise<sensor_msgs::LaserScan>("Doris/scan",1,true);
    //Publishing image
    image_transport::ImageTransport it(nh_);
    this->image_pub=it.advertise("Doris/camera",1,true);
    //Publishing tf to laser_link
        this->static_laserTransform.header.frame_id="Doris/cuerpo";
        this->static_laserTransform.child_frame_id="Doris/laser_link";
        this->static_laserTransform.transform.translation.x=0.1;
        this->static_laserTransform.transform.translation.y=0.0;
        this->static_laserTransform.transform.translation.z=0.35;
        this->static_laserTransform.transform.rotation.x=0.0;
        this->static_laserTransform.transform.rotation.y=0.0;
        this->static_laserTransform.transform.rotation.z=0.0;
        this->static_laserTransform.transform.rotation.w=1.0;
        this->static_laserTransform.header.stamp=ros::Time::now();
        this->static_broadcaster.sendTransform(static_laserTransform);
    //Camera
    const string cameraUrl= "http://192.168.0.19/record/current.jpg";
    curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, cameraUrl.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &cConnection::write_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &cameraStream);

}

size_t cConnection::write_data(char *ptr, size_t size, size_t nmemb, void *userdata) {
        std::ostringstream *stream = (std::ostringstream*)userdata;
        size_t count = size * nmemb;
        stream->write(ptr, count);
        return count;
}


void cConnection::readImage(void){
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
                    imshow("Camara",frame);
                    waitKey(30);
                    this->image_msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",frame).toImageMsg();
                    this->image_pub.publish(this->image_msg);

    }


}

void cConnection::readFromSocket(void){
    //cout<<"entro"<<endl;
    long valread;
    char message [1024];
    unsigned int sourceSize= sizeof(this->addr);
    //cout<<"conectando"<<endl;
    //Reading from socket
    valread=recvfrom(this->socket_,message, 1024,0,  (struct sockaddr *) &addr, &sourceSize);

    //this->decoMessage(message,valread);
    if (valread<0){
        perror("recvfrom()");

    }else{
        //cout<<message<<endl;
        this->decoMessage(message,valread);
        memset(&message[0], 0, sizeof(message));

    }
}

void cConnection::decoMessage(char message [], int size){
    string str(message);
    std::string encabezado_odom="POSE_VEL";
    std::string encabezado_laser="LASER";
    std::vector<float> vec;
    //std::string str="$POSE_VEL|5.2,7.6,8.2,7.4,8.5";
    string comp= "|";
    string encabezado=str.substr(1,str.find(comp)-1);
    cout<<encabezado<<endl;
    string variable = str.substr(str.find(comp)+1, str.length());
    std::stringstream ss(variable);
    int i=0;
    while (ss.good()){
       string substr;
       getline(ss,substr,',');
       vec.push_back(stof(substr));
    }
    for (i=0; i< vec.size(); i++)
            std::cout << vec.at(i)<<std::endl;
   /* encabezado="LASER";
    for (i=0;i<360;i++){
        vec.push_back(1);
    }*/
    if(encabezado == encabezado_odom){
        //odom_base
        odom.header.frame_id="Doris/odom";
        odom.child_frame_id="Doris/cuerpo";
        odom.transform.translation.x=vec.at(0);
        odom.transform.translation.y=vec.at(1);
        odom.transform.translation.z=0.0;
        tf2::Quaternion quat;
        quat.setRPY(0,0,vec.at(2));
        odom.transform.rotation.x=quat.x();
        odom.transform.rotation.y=quat.y();
        odom.transform.rotation.z=quat.z();
        odom.transform.rotation.w=quat.w();
        odom.header.stamp=ros::Time::now();
        this->br.sendTransform(odom);


    }
    if (encabezado==encabezado_laser){
      scan.header.stamp=ros::Time::now();
      scan.header.frame_id="Doris/laser_link";
      scan.angle_min=PI;
      scan.angle_max=-PI;
      scan.angle_increment=(2*PI)/(vec.size());
      scan.range_min=0.0;
      scan.range_max=80;
      scan.ranges=vec;
      this->laser_pub.publish(scan);


    }

}
