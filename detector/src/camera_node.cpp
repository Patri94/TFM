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
#include <boost/assign/list_of.hpp>
#include <string>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <functional>
#include <signal.h>
#include <detector/camera_node.h>
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
#include <boost/geometry/geometries/adapted/boost_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <XmlRpcException.h>
#include <detector/marker.h>
#include <detector/detector.h>

#define BLOCK_SIZE_FOR_ADAPTIVE_THRESHOLD 75
#define CELL_MARKER_SIZE 7
#define CURVE_SIZE 4
/*#define IMAGE_WIDTH 1812
#define MARKER_HEIGHT 0.545985
#define MARKER_WIDTH 0.3777
#define NUM_CAM 3*/


using namespace cv;
using namespace std;
using namespace tf2;
using namespace nav_msgs;
using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;
using namespace XmlRpc;
/**
 * @brief CameraNode::CameraNode
 * @param nh
 * @param nh_private
 */
CameraNode::CameraNode ( const ros::NodeHandle& nh,const ros::NodeHandle& nh_private)
    :nh_(nh),nh_private_(nh_private)
{
    //Load Parameters
    float  image_width, marker_height, marker_width, num_cam;
    XmlRpc::XmlRpcValue marker_list;
    nh_.getParam("/detector/IMAGE_WIDTH",image_width);


    //Callbacks
    typedef const boost::function< void(const sensor_msgs::ImageConstPtr &)>  callback;
    typedef const boost::function< void(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&)> Callback;
    typedef const boost::function< void(const nav_msgs::Odometry::ConstPtr& msg)> CallbackOdom;
    callback boundImageCallback2 = boost::bind(&CameraNode::imageCallback2,this,_1);
    callback boundImageCallback3 = boost::bind(&CameraNode::imageCallback3,this,_1);
    CallbackOdom boundOdom = boost::bind(&CameraNode::OdomCallback,this,_1);


    //Initialize subscribers for cameras
    image_transport::ImageTransport it(nh_);
    image_transport::Subscriber subcam2= it.subscribe("Doris/camera2/image_raw", 1, boundImageCallback2);
    image_transport::Subscriber subcam3 = it.subscribe("Doris/camera3/image_raw", 1, boundImageCallback3);
    message_filters::Subscriber<Image> image_sub(nh_, "/Doris/camera1/image_raw", 1);
    message_filters::Subscriber<CameraInfo> info_sub(nh_, "/Doris/camera1/camera_info", 1);
    TimeSynchronizer<Image, CameraInfo> sync(image_sub, info_sub, 10);
    sync.registerCallback(boost::bind(&CameraNode::infoCallback, this,_1, _2));
    this->odometry=nh_.subscribe("/Doris/odom",1,boundOdom);
    this->pub_comb=it.advertise("DetectorOutput",1);


    //Initialize publishers

    this->publish_detection=nh_.advertise<detector::detector>("detected_corners",1);

    //Size of markers for the detector
    markerSize = cv::Size(215, 345);

    this->markerCorners.push_back(cv::Point2f(0, 0));
    this->markerCorners.push_back(cv::Point2f(markerSize.width - 1, 0));
    this->markerCorners.push_back(cv::Point2f(markerSize.width - 1, markerSize.height - 1));
    this->markerCorners.push_back(cv::Point2f(0, markerSize.height - 1));

    while(nh_.ok()){
            ros::spinOnce();
            this->detectorTask();


        }

}
/**
 * @brief CameraNode::~CameraNode
 * Destructor of class CameraNode
 */
CameraNode::~CameraNode(){

}
/**
 * @brief CameraNode::LoadMap
 * @return World Coordinates of visual markers
 */

void CameraNode::OdomCallback (const nav_msgs::Odometry::ConstPtr& msg){
    this->estimated_pose=msg->pose.pose;
}



/**
 * @brief CameraNode::infoCallback
 *  Subscriber to the central camera (image and info)
 * @param msg
 * @param cam_info
 */
void CameraNode::infoCallback(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info){
    this->cam1 = cv_bridge::toCvShare(msg, "bgr8")->image;
     this->ms1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->cam1).toImageMsg();
}

/**
 * @brief CameraNode::imageCallback2
 * Left Camera
 * @param msg
 */
void CameraNode::imageCallback2(const sensor_msgs::ImageConstPtr& msg){
    this->cam2 = cv_bridge::toCvShare(msg, "bgr8")->image;
    this->ms2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->cam2).toImageMsg();
}
/**
 * @brief CameraNode::imageCallback3
 * Right Camera
 * @param msg
 */
void CameraNode::imageCallback3(const sensor_msgs::ImageConstPtr& msg){
    this->cam3 = cv_bridge::toCvShare(msg, "bgr8")->image;
    this->ms1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->cam3).toImageMsg();
}

/**
 * @brief CameraNode::Concatenar
 * Make a single image out of all cameras.
 */
void CameraNode::imageBlending(void){
    if (!(this->cam1.empty()) && !(this->cam2.empty()) && !(this->cam3.empty())){
    cv::hconcat(this->cam2,this->cam1,this->comb);
    cv::hconcat(this->comb,this->cam3,this->comb);
    cv::imshow("combi", this->comb);
    cv::waitKey(30);
        }
}

/**
 * @brief CameraNode::LimpiarImagen
 * Treatment of image (erosion and dilation)
 */
void CameraNode::imageTreatment(void){
    if(!(this->comb.empty())){
    int erosionSize = 0;
    cv::Mat erosion;
    cv::Mat element=cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1), cv::Point(erosionSize - 1, erosionSize - 1));
    cv::erode(this->comb, erosion, element);
    cv::dilate(erosion, this->comb, element);
        }
}

void CameraNode::detectorTask(void){
    this->imageBlending();
    this->imageTreatment();
    this->brightnessAverage();
    this->threshold();
    this->findContours();
    this->findCandidates();
    this->OptMarkers.clear();
    this->recognizeMarkers();
    this->publishDetectedCorners();
}

/**
 * @brief CameraNode::brightnessAverage
 */
void CameraNode::brightnessAverage(void){
    if(!(this->comb.empty())){
    cv::cvtColor(this->comb, this->gray, CV_BGR2GRAY);
    int histSize = 256;
    float range[] = { 0,256 };
    const float* histRange = { range };
    bool uniform = true; bool accumulate = false;
    cv::Mat hist;
    cv::calcHist(&gray, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );
    cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0) );
    cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
    for( int i = 1; i < histSize; i++ ){
                    line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
                            cv::Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
                            cv::Scalar( 0, 0, 255), 2, 8, 0  );
            }
    this->brightnessAvg=0;

            for(int i =0; i< 256; i++){
                this->brightnessAvg+= hist.at<float>(i)*i/cv::sum(hist)[0];  //Media ponderada
            }

    //cout<<this->brightnessAvg<<endl;
        }
}

void CameraNode::threshold(){
    if(!(this->comb.empty())){
    cv::cvtColor(this->comb,this->gray, CV_BGR2GRAY);
    float index= 75.0 / 255.0 * this->brightnessAvg ;
    cv::adaptiveThreshold(this->gray,this->thresholded, 255,  CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY_INV,BLOCK_SIZE_FOR_ADAPTIVE_THRESHOLD, (int)index);
        }
}
void CameraNode::findContours (void){
    if (!(this->comb.empty())){
    RNG rng(12345);
    this->minContourPointsAllowed=35;
    this->maxContourPointsAllowed=70;
    this->contours.clear();
    cv::Mat edges;
     vector<Vec4i> hierarchy;
     std::vector<std::vector<cv::Point> > Todos;
    cv::Canny(this->thresholded,edges,100,180,5);
    cv::findContours (edges,Todos,hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    for (unsigned int i = 0; i < Todos.size(); i++){
                    if (Todos.at(i).size() > minContourPointsAllowed){
                            contours.push_back(Todos.at(i));
                    }
    }
      Mat drawing = Mat::zeros( edges.size(), CV_8UC3 );
      for( int i = 0; i< contours.size(); i++ )
         {
           Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
           drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
         }
    //  imshow("Contornos",drawing);
     // cv::waitKey(30);
      edges.release();
}
}

float CameraNode::perimeter(const std::vector<cv::Point2f> &a){
    float result=0, dx, dy;
    for (int i = 0; i < a.size(); i++){
                    dx = a[i].x - a[(i + 1) % a.size()].x;
                    dy = a[i].y - a[(i + 1) % a.size()].y;

                    result += std::sqrt(dx * dx + dy * dy);
    }
    return result;
}
/**
 * @brief CameraNode::findCandidates
 * filtering contours
 */
void CameraNode::findCandidates(void){
     if (!(this->comb.empty())){
    this->minContourLengthAllowed = 100.0;
    this->maxContourLengthAllowed = 4000.0;
    posiblesMarcas.clear();
    std::vector<cv::Point> aprox;
    for (unsigned int i = 0; i < contours.size(); i++){
         //Precisión en la aproximación de la curva
        double eps = contours.at(i).size() * .1;
        cv::approxPolyDP(contours.at(i), aprox, eps, true);
        if (aprox.size() == CURVE_SIZE && cv::isContourConvex(aprox)){
                float minDist = std::numeric_limits<float>::max();
                for (int j = 0; j < aprox.size(); j++){
                           cv::Point side = aprox.at(j) - aprox.at((j + 1) % 4);
                           minDist = std::min(minDist, (float)side.dot(side));
                }
                if (minDist > this->minContourLengthAllowed){
                            Marcador marker;
                            for (int j = 0; j < aprox.size(); j++){
                                       marker.addPoint(cv::Point2f(aprox.at(j).x, aprox.at(j).y));
                            }
                            cv::Point2f v1 = marker.getPoint(1) - marker.getPoint(0);
                            cv::Point2f v2 = marker.getPoint(2) - marker.getPoint(0);
                            double o = (v1.x * v2.y) - (v1.y * v2.x);
                            if (o < 0.0){
                                    cv::Point2f auxPoint = marker.getPoint(1);
                                    marker.setPoint(marker.getPoint(3), 1);
                                    marker.setPoint(auxPoint, 3);
                            }
                            marker.setContourIdx(i);
                            marker.setRotatedRect(cv::minAreaRect(aprox));
                            marker.setArea(cv::contourArea(aprox));
                            posiblesMarcas.push_back(marker);
                }


            }

        }
        std::vector<std::pair<int, int> > closestCandidates;

            for (int i = 0; i < posiblesMarcas.size(); i++){
                    Marcador markerA = posiblesMarcas.at(i);
                    for (int j = i + 1; j < posiblesMarcas.size(); j++){
                            Marcador markerB = posiblesMarcas.at(j);
                            float distSquared = 0;
                            for (int k = 0; k < CURVE_SIZE; k++){
                                    cv::Point v = markerA.getPoint(k) - markerB.getPoint(k);
                                    distSquared += v.dot(v);

                            }
                            distSquared /= 4;
                            if (distSquared < 100){
                                    closestCandidates.push_back(std::pair<int, int>(i, j));
                            }
                    }
    }

            std::vector<bool> removalMask(posiblesMarcas.size(), false);
                    for (int i = 0; i < closestCandidates.size(); i++){
                            float p1 = perimeter(posiblesMarcas.at(closestCandidates.at(i).first).getMarkerPoints());
                            float p2 = perimeter(posiblesMarcas.at(closestCandidates.at(i).second).getMarkerPoints());

                            int index;
                            if (p1 > p2){
                                    index = closestCandidates.at(i).first;
                            } else {
                                    index = closestCandidates.at(i).second;
                            }
                            removalMask.at(index) = true;
            }
                    Marcadores.clear();
                            for (int i = 0; i < posiblesMarcas.size(); i++){
                                    if (!removalMask[i])
                                            Marcadores.push_back(posiblesMarcas[i]);
                            }
                           //Dibujar candidatos
                            imshow("Detection",this->comb);
                            waitKey(30);
                            for (int i=0; i< Marcadores.size();i++){
                                    vector<Point2f> puntos = Marcadores[i].getAllPoints();
                                    cout<<"Número de puntos"<<puntos.size()<<endl;
                                    line (this->comb,puntos[0], puntos[1],Scalar(0,255,0),4);
                                    line (this->comb,puntos[1], puntos[2],Scalar(0,255,0),4);
                                    line (this->comb,puntos[2], puntos[3],Scalar(0,255,0),4);
                                    line (this->comb,puntos[3], puntos[0],Scalar(0,255,0),4);
                                }
                            //cout<<comb.size()<<endl;
                            imshow("Detection",this->comb);
                            waitKey(30);
                    }
}
cv::Mat CameraNode::rotate(cv::Mat input)
{
        cv::Mat out;
        input.copyTo(out);
        for (int i = 0; i < input.rows; i++){
                for (int j = 0; j < input.cols; j++){
                        out.at<uchar>(i, j) = input.at<uchar>(input.cols - j - 1, i);
                }
        }
        return out;
}
/**
 * @brief CameraNode::MarkerDecoder
 * finding identity of marker
 * @param inputGrayscale
 * @param nRrotations
 * @param marker
 * @return
 */
int CameraNode::MarkerDecoder(const cv::Mat& inputGrayscale, int& nRrotations, Marcador &marker){
    int result = 0;
            cv::Mat grey = inputGrayscale;
            cv::threshold(grey, grey, 127, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            int cellHeigth = inputGrayscale.rows / CELL_MARKER_SIZE;
            int cellWidth = inputGrayscale.cols / CELL_MARKER_SIZE;
            for (int y = 0; y < CELL_MARKER_SIZE; y++){
                    int inc = 6;
                    if (y == 0 || y == 6){
                            inc = 1;
                    }

                    for (int x = 0; x < CELL_MARKER_SIZE; x += inc){
                            int cellX = x * cellWidth;
                            int cellY = y * cellHeigth;
                            cv::Mat cell = grey(cv::Rect(cellX, cellY, cellWidth, cellHeigth));
                            int nZ = cv::countNonZero(cell);
                            if (nZ >(cellWidth*cellHeigth) / 2){
                                    result = -1;
                            }
                    }
                    if (result == 0){
                                    cv::Mat bitMatrix = cv::Mat::zeros(5, 5, CV_8UC1);
                                    for (int y = 0; y < 5; y++){
                                            for (int x = 0; x < 5; x++){
                                                    int cellX = (x + 1) * cellWidth;
                                                    int cellY = (y + 1) * cellHeigth;
                                                    cv::Mat cell = grey(cv::Rect(cellX, cellY, cellWidth, cellHeigth));
                                                    int nZ = cv::countNonZero(cell);
                                                    if (nZ >(cellWidth*cellHeigth) / 2){
                                                            bitMatrix.at<uchar>(y, x) = 1;
                                                    }
                                            }
                                    }

                                    cv::Mat rotations[CURVE_SIZE];
                                    int distances[CURVE_SIZE];
                                    rotations[0] = bitMatrix;
                                    distances[0] = hammingDistance(bitMatrix);

                                    std::pair<int, int> minDist(distances[0], 0);

                                    for (int i = 1; i < CURVE_SIZE; i++){
                                            //get the hamming distance to the nearest possible word
                                            rotations[i] = rotate(rotations[i - 1]);
                                            distances[i] = hammingDistance(rotations[i]);

                                            if (distances[i] < minDist.first){
                                                    minDist.first = distances[i];
                                                    minDist.second = i;
                                            }
                                    }

                                    nRrotations = minDist.second;
                                    if (minDist.first == 0){
                                int mapId = 0, sectorId = 0, markerId = 0;
                                markerIdNumber(rotations[nRrotations], mapId, sectorId, markerId);
                                marker.setMapId(mapId);
                                marker.setSectorId(sectorId);
                                marker.setMarkerId(markerId);
                                std::string etiqueta ;
                                ostringstream convert;
                                convert<<markerId;
                                etiqueta=convert.str();
                                putText(this->comb, etiqueta, marker.getPoint(0),CV_FONT_HERSHEY_COMPLEX,0.8,Scalar(0,0,255));
                                imshow("DetectionEti",this->comb);

                            } else {
                                result = -1;
                            }
                            }
                            return result;
                    }
    }

void CameraNode::recognizeMarkers(){
     if (!(this->comb.empty())){
        std::vector<Marcador> goodMarkers;
        for (int i = 0; i < this->Marcadores.size(); i++){
                Marcador marker = this->Marcadores.at(i);
                Mat markerTransform=getPerspectiveTransform(marker.getMarkerPoints(), this->markerCorners);
                cv::warpPerspective(this->gray, canonicalMarkerImage, markerTransform, markerSize);
                int rotations = 0;
                if (MarkerDecoder(canonicalMarkerImage, rotations, marker) == 0){
                        std::vector<cv::Point2f> markerDots = marker.getMarkerPoints();
                        std::rotate(markerDots.begin(), markerDots.begin() + 4 - rotations, markerDots.end());
                        marker.MarkerPoints(markerDots);
                        goodMarkers.push_back(marker);

                }

        }
        OptMarkers = goodMarkers;
        for (int i=0; i< OptMarkers.size();i++){
                vector<Point2f> puntos = OptMarkers[i].getAllPoints();
                //cout<<"Número de puntos"<<puntos.size()<<endl;
                line (this->comb,puntos[0], puntos[1],Scalar(0,255,0),4);
                line (this->comb,puntos[1], puntos[2],Scalar(0,255,0),4);
                line (this->comb,puntos[2], puntos[3],Scalar(0,255,0),4);
                line (this->comb,puntos[3], puntos[0],Scalar(0,255,0),4);
            }
        this->comb_msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",this->comb).toImageMsg();

}
}
void CameraNode::publishDetectedCorners(void){
    if (!(this->comb.empty())){
       detector::detector msg;
        for (int i=0;i<this->OptMarkers.size();i++){
                detector::marker detected;
                geometry_msgs::Point32 corners_m[4];
                std::vector<cv::Point2f> corners_f =this->OptMarkers[i].getMarkerPoints();
                for (int j=0;j<4;j++){
                        geometry_msgs::Point32 corner;
                        corner.x=corners_f[j].x;
                        corner.y=corners_f[j].y;
                        corner.z=0;
                        corners_m[j]=corner;
                        detected.Corners[j]=corners_m[j];

                    }
                //Detected.Corners=Cornersm;
                detected.ID=this->OptMarkers[i].getMarkerID();
                msg.DetectedMarkers.push_back(detected);

           }

        this->publish_detection.publish(msg);
        this->pub_comb.publish(this->comb_msg);
        }

}

float CameraNode::linearInterpolator(const float& x, const cv::Point p1, const cv::Point p2){
        return (((x - p1.x)/(p2.x - p1.x) * (p2.x - p1.x)) + p1.x);
}


int CameraNode::hammingDistance(cv::Mat bits){
    int ids[2][5] = {
                    { 1, 0, 0, 0, 1 },
                    { 1, 0, 1, 1, 1 }
            };

            int dist = 0;
            int sum = 0;

            //Compares the first and last row of the bit matrix with the template matrix ids

            for (int x = 0; x < 5; x++){
                    sum += bits.at<uchar>(0, x) == ids[0][x] ? 0 : 1;
            }

            if (1e5 > sum){
                    dist = sum;
            }

            sum = 0;

            for (int x = 0; x < 5; x++){
                    sum += bits.at<uchar>(4, x) == ids[1][x] ? 0 : 1;
            }

            dist += sum;

    return dist;
}

void CameraNode::markerIdNumber(const cv::Mat &bits, int &mapId, int &sectorId, int &markerId){

    for (int j = 0; j < 5; j++){
        if(bits.at<uchar>(1, j)){
                mapId += (16 / std::pow(2, j)) * bits.at<uchar>(1, j);
        }
    }
    for (int j = 0; j < 5; j++){
        if(bits.at<uchar>(2, j)){
                sectorId += (16 / std::pow(2, j)) * bits.at<uchar>(2, j);
        }
    }
    for (int j = 0; j < 5; j++){
        if(bits.at<uchar>(3, j)){
                markerId += (16 / std::pow(2,j)) * bits.at<uchar>(3, j);
        }
    }
}

int CameraNode::getMapSize(void){
    return this->map.size();
}






