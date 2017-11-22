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
#include <connection/c_connection.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace XmlRpc;

cConnection::cConnection (const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
:nh_(nh),nh_private_(nh_private)
{
    //Parameters for socket
    nh_.getParam("DorisConnection/domain",this->domain);
    //cout<<"llego"<<endl;
    nh_.getParam("DorisConnection/type",this->type);
    nh_.getParam("DorisConnection/protocol",this->protocol);
    //cout<<"llego"<<endl;
    std::string address_st;
    int family;
    nh_.getParam("DorisConnection/address",address_st);
    nh_.getParam("DorisConnection/port",this->port);
    nh_.getParam("DorisConnection/family",family);
    //cout<<address_st<<endl;
    char* address="192.168.4.111";
    this->addr.sin_family=AF_INET;
    //cout<<"created address1"<<endl;
    //strcpy(address,address_st.c_str());
    //cout<<"created address2"<<endl;
    cout<<port<<endl;
    addr.sin_port=htons(port);
     //addr.sin_addr.s_addr="inet_addr(address)";
    inet_pton(AF_INET,"192.168.4.111",& (addr.sin_addr));
    //cout<<"created address4"<<endl;
    //cout<<inet_addr(address)<<endl;
    cout<<type<<endl;
    cout<<domain<<endl;
    cout<<protocol<<endl;
    //initializing socket
    this->socket_=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    cout<<socket<<endl;
    cout<<"created socket"<<endl;
    //binding socket
    int connection=bind(this->socket_, (const struct sockaddr *) &addr,  sizeof(this->addr));

}

void cConnection::readFromSocket(void){
    cout<<"entro"<<endl;
    long valread;
    char message [1024];
    unsigned int sourceSize= sizeof(this->addr);
    //cout<<"conectando"<<endl;
    //Reading from socket
    valread=recvfrom(this->socket_,message, 1024,0,  (struct sockaddr *) &addr, &sourceSize);
    cout<<valread<<endl;
    //imprimir por pantalla

    if (valread<0){
        perror("recvfrom()");

    }else{
        cout<<message<<endl;
        this->decoMessage(message,valread);
        memset(&message[0], 0, sizeof(message));

    }
}

void cConnection::decoMessage(char message [], int size){
    //std::string encabezado;
    int i=0;
    string str(message);
    string comp= "|";
    string encabezado=str.substr(1,str.find(comp));
    cout<<comp<<endl;
   /* while (&str.at(i) != comp){
        encabezado += str.at(i);
        cout<<encabezado<<endl;
        i++;
    }*/
    cout<<encabezado<<endl;
    /*if (size==20){
        //Mesaje de posición
        float x = (float) (message[11]);
        float y = (float) (message[13]);
        float theta = (float) (message[15]);
        float vl = (float) (message[17]);
        float vtheta = (float) (message[19]);
        cout<<x<<","<<y<<","<<theta<<","<<vl<<","<<theta<<endl;
    }*/

}
