#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/ccalib/omnidir.hpp>
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

#define BLOCK_SIZE_FOR_ADAPTIVE_THRESHOLD 75
#define CELL_MARKER_SIZE 7
#define CURVE_SIZE 4
#define RECTIFIED_IMAGE_WIDTH 1812
#define RECTIFIED_IMAGE_HEIGHT 679

using namespace cv;
using namespace cv::omnidir;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace XmlRpc;
/**
 * @brief cDetector::cDetector
 * @param nh
 * @param nh_private
 */
cDetector::cDetector ( const ros::NodeHandle& nh,const ros::NodeHandle& nh_private)
    :nh_(nh),nh_private_(nh_private)
{
    //Load Parameters
    float  image_width, marker_height, marker_width, num_cam;

    //XmlRpc::XmlRpcValue marker_list;
     nh_.getParam("/detector/IMAGE_WIDTH",image_width);
     nh_.getParam("/DetectorNode/min_ID",min_ID);
     nh_.getParam("/DetectorNode/max_ID",max_ID);
     nh_.getParam("/DetectorNode/min_sector",min_sector);
     nh_.getParam("/DetectorNode/max_sector",max_sector);
     nh_.getParam("/DetectorNode/min_map",min_map);
     nh_.getParam("/DetectorNode/max_map",max_map);
     nh_.getParam("/DetectorNode/simulation",this->simulation);

    //Initialize subscribers for cameras
    switch (this->simulation){
        case 1:
    {
    sub_cam1=nh_.subscribe<sensor_msgs::Image> ("Doris/cam1/image_raw",1,&cDetector::infoCallback,this);
    sub_cam2=nh_.subscribe<sensor_msgs::Image> ("Doris/cam2/image_raw",1,&cDetector::imageCallback2,this);
    sub_cam3=nh_.subscribe<sensor_msgs::Image> ("Doris/cam3/image_raw",1,&cDetector::imageCallback3,this);

    break;
    }
        case 0:
    {
    sub_camDoris=nh_.subscribe<sensor_msgs::Image> ("Doris/camera/image_raw",1,&cDetector::realCallback,this);

    newSize= cv::Size(RECTIFIED_IMAGE_WIDTH, RECTIFIED_IMAGE_HEIGHT);

    camMatrix = cv::Mat(3, 3, CV_32F);
    camMatrix.at<float>(0, 0) = 3.3148337972624245e+02;
    camMatrix.at<float>(0, 1) = 0;
    camMatrix.at<float>(0, 2) = 6.5050896530720797e+02;
    camMatrix.at<float>(1, 0) = 0.0;
    camMatrix.at<float>(1, 1) = 3.3296507853901846e+02;
    camMatrix.at<float>(1, 2) = 4.9324794942591592e+02;
    camMatrix.at<float>(2, 0) = 0.0;
    camMatrix.at<float>(2, 1) = 0.0;
    camMatrix.at<float>(2, 2) = 1.0;

    distCoeff = cv::Mat(4, 1, CV_32F);
    distCoeff.at<float>(0, 0) = -5.0278669230113635e-02;
    distCoeff.at<float>(1, 0) = 2.7927571053875219e-02;
    distCoeff.at<float>(2, 0) = -9.7303697830329119e-03;
    distCoeff.at<float>(3, 0) = 0;

    Knew = cv::Matx33f(newSize.width / (2 * M_PI), 0, 0, 0, newSize.height / M_PI, 0, 0, 0, 1);
    xi = cv::Mat(1, 1, CV_32FC1);
    xi.at<float>(0, 0) = 1.5861076761699640e+00;
    break;
    }
    default:
        break;

    }


    //Initialize publishers
     this->publish_detection=nh_.advertise<detector::messagedet>("/DetectorNode/detection",1,true);
    image_transport::ImageTransport it(nh_);
    this->pub_comb=it.advertise("/DetectorNode/detector_output",1);


    //Size of markers for the detector
    markerSize = cv::Size(215, 345);

    this->markerCorners.push_back(cv::Point2f(0, 0));
    this->markerCorners.push_back(cv::Point2f(markerSize.width - 1, 0));
    this->markerCorners.push_back(cv::Point2f(markerSize.width - 1, markerSize.height - 1));
    this->markerCorners.push_back(cv::Point2f(0, markerSize.height - 1));




}
/**
 * @brief cDetector::~cDetector
 * Destructor of class cDetector
 */
cDetector::~cDetector(){

}


/**
 * @brief cDetector::infoCallback
 *  Subscriber to the central camera (image and info)
 * @param msg
 * @param cam_info
 */
void cDetector::infoCallback(const sensor_msgs::ImageConstPtr& msg){
    //cout<<"Callback"<<endl;
    this->cam1 = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    this->ms1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->cam1).toImageMsg();
   // cv::imshow("cam1", this->cam1);
   // cv::waitKey(30);
}

/**
 * @brief cDetector::imageCallback2
 * Left Camera
 * @param msg
 */
void cDetector::imageCallback2(const sensor_msgs::ImageConstPtr& msg){
    this->cam2 = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    this->ms2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->cam2).toImageMsg();
   // cv::imshow("cam2", this->cam2);
   // cv::waitKey(30);
}
/**
 * @brief cDetector::imageCallback3
 * Right Camera
 * @param msg
 */
void cDetector::imageCallback3(const sensor_msgs::ImageConstPtr& msg){
    this->cam3 = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    this->ms3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->cam3).toImageMsg();
   // cv::imshow("cam3", this->cam3);
   // cv::waitKey(30);
}

void cDetector::realCallback(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat image;
    image=cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    cv::omnidir::undistortImage(image, this->comb, camMatrix, distCoeff, xi, RECTIFY_CYLINDRICAL, Knew, newSize);
}

/**
 * @brief cDetector::Concatenar
 * Make a single image out of all cameras.
 */
void cDetector::imageBlending(void){
    //cout<<"llego"<<endl;
   // cout<<this->cam1.empty()<<this->cam2.empty()<<this->cam3.empty()<<endl;
    if (!(this->cam1.empty()) && !(this->cam2.empty()) && !(this->cam3.empty())){
        //cout<<"entro"<<endl;
    cv::hconcat(this->cam2,this->cam1,this->comb);
    cv::hconcat(this->comb,this->cam3,this->comb);
    //cv::imshow("combi", this->comb);
   // cv::waitKey(30);
        }
}

/**
 * @brief cDetector::LimpiarImagen
 * Treatment of image (erosion and dilation)
 */
void cDetector::imageTreatment(void){
    if(!(this->comb.empty())){
    int erosionSize = 0;
    cv::Mat erosion;
    cv::Mat element=cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1), cv::Point(erosionSize - 1, erosionSize - 1));
    cv::erode(this->comb, erosion, element);
    cv::dilate(erosion, this->comb, element);
        }
}

void cDetector::detectorTask(void){

    if (this->simulation == true){
    this->imageBlending();
    }
    this->imageTreatment();
    this->brightnessAverage();
    this->threshold();
    this->findContours();
    this->findCandidates();
    this->OptMarkers.clear();
    this->recognizeMarkers();
    this->createMessage();

}

/**
 * @brief cDetector::brightnessAverage
 */
void cDetector::brightnessAverage(void){
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

void cDetector::threshold(){
    if(!(this->comb.empty())){
    cv::cvtColor(this->comb,this->gray, CV_BGR2GRAY);
    float index= 75.0 / 255.0 * this->brightnessAvg ;
    cv::adaptiveThreshold(this->gray,this->thresholded, 255,  CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY_INV,BLOCK_SIZE_FOR_ADAPTIVE_THRESHOLD, (int)index);
        }
}
void cDetector::findContours (void){
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

float cDetector::perimeter(const std::vector<cv::Point2f> &a){
    float result=0, dx, dy;
    for (int i = 0; i < a.size(); i++){
                    dx = a[i].x - a[(i + 1) % a.size()].x;
                    dy = a[i].y - a[(i + 1) % a.size()].y;

                    result += std::sqrt(dx * dx + dy * dy);
    }
    return result;
}
/**
 * @brief cDetector::findCandidates
 * filtering contours
 */
void cDetector::findCandidates(void){
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
                            cv::Point2f v3 = marker.getPoint(3) - marker.getPoint(0);
                            double z1 = (v1.x * v2.y) - (v1.y * v2.x);
                                    if (z1 < 0.0){
                                    cv::Point2f auxPoint = marker.getPoint(1);
                                    marker.setPoint(marker.getPoint(3), 1);
                                    marker.setPoint(auxPoint, 3);
                            }
                            //Order Points ( 0 -> top left corner)
                            if (marker.getPoint(1).y < marker.getPoint(3).y){
                                /*for (int i=0;i<4;i++){
                                    cout<<"x:"<< marker.getPoint(i).x<<" y:"<<marker.getPoint(i).y<<endl;
                                }*/
                                cv::Point2f auxPoint =marker.getPoint(0);
                                for (int i=1;i<4;i++){
                                    marker.setPoint(marker.getPoint(i),i-1);
                                }
                                marker.setPoint(auxPoint,3);
                               /*for (int i=0;i<4;i++){
                                   cout<<"x:"<< marker.getPoint(i).x<<" y:"<<marker.getPoint(i).y<<endl;
                               }
                               waitKey();*/
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
                           /* imshow("Detection",this->comb);
                            waitKey(30);
                            for (int i=0; i< Marcadores.size();i++){
                                    vector<Point2f> puntos = Marcadores[i].getAllPoints();
                                    //cout<<"Número de puntos"<<puntos.size()<<endl;
                                    line (this->comb,puntos[0], puntos[1],Scalar(0,255,0),4);
                                    line (this->comb,puntos[1], puntos[2],Scalar(0,255,0),4);
                                    line (this->comb,puntos[2], puntos[3],Scalar(0,255,0),4);
                                    line (this->comb,puntos[3], puntos[0],Scalar(0,255,0),4);
                                }
                            //cout<<comb.size()<<endl;
                            imshow("Detection",this->comb);
                            waitKey(30);*/
                    }
}
cv::Mat cDetector::rotate(cv::Mat input)
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
 * @brief cDetector::MarkerDecoder
 * finding identity of marker
 * @param inputGrayscale
 * @param nRrotations
 * @param marker
 * @return
 */
int cDetector::MarkerDecoder(const cv::Mat& inputGrayscale, int& nRrotations, Marcador &marker){
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

                            } else {
                                result = -1;
                            }
                            }
                            return result;
                    }
    }

void cDetector::recognizeMarkers(){
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
        imshow("DetectionEti",this->comb);
        waitKey(30);

        this->comb_msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",this->comb).toImageMsg();

}
}
void cDetector::createMessage(void){
    if (!(this->comb.empty())){
        detector::messagedet msg_det;
        OptMarkers=this->orderDetection(OptMarkers);
       for (int i=0;i<this->OptMarkers.size();i++){
                detector::marker detected;
                std::vector<cv::Point2f> corners_f =this->OptMarkers[i].getMarkerPoints();
                //Adding undistort Points ??
                for (int j=0;j<4;j++){
                        geometry_msgs::Point32 corner;
                        corner.x=(float)corners_f[j].x;
                        corner.y=(float)corners_f[j].y;
                        corner.z=0.0;
                        detected.Corners.push_back(corner);
                    }
                detected.ID.data=uint8_t(this->OptMarkers[i].getMarkerID());
                detected.map.data=uint8_t(this->OptMarkers[i].getMapID());
                detected.sector.data=uint8_t(this->OptMarkers[i].getSectorID());
                msg_det.DetectedMarkers.push_back(detected);
      }
        //cout<<"detectados"<<msg_det.DetectedMarkers.size()<<endl;
         msg_det.header.frame_id="Doris/cam1_link";
         msg_det.header.stamp=ros::Time::now();
         this->pub_comb.publish(this->comb_msg);
         this->publish_detection.publish(msg_det);
        }

}

void cDetector::publish(void){

}

float cDetector::linearInterpolator(const float& x, const cv::Point p1, const cv::Point p2){
        return (((x - p1.x)/(p2.x - p1.x) * (p2.x - p1.x)) + p1.x);
}


int cDetector::hammingDistance(cv::Mat bits){
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

void cDetector::markerIdNumber(const cv::Mat &bits, int &mapId, int &sectorId, int &markerId){

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

int cDetector::getMapSize(void){
    return this->map.size();
}

std::vector<Marcador> cDetector::orderDetection(std::vector<Marcador> detection){
    Marcador temp;
    //cout<<"cuantos"<<detection.size()<<endl;
    for (int i=0; i<detection.size();i++){
        cout<<"antes de quitar"<<detection[i].getMarkerID()<<" "<<detection[i].getSectorID()<<" "<<detection[i].getMapID()<<" " << i<<endl;
        if(detection[i].getMarkerID()<min_ID || detection[i].getMarkerID()>max_ID || detection[i].getSectorID()<min_sector || detection[i].getSectorID()>max_sector || detection[i].getMapID()!=min_map){
            cout<<"tengo que quitar "<<detection[i].getMarkerID()<<" "<<detection[i].getSectorID()<<" "<<detection[i].getMapID()<<" " << i<<endl;
            detection.erase(detection.begin()+i);
            i=i-1;

            }

    }
        if (detection.size()>=2){
        //First map
        for (int k=0; k<detection.size()-1;k++){
            if (detection[k].getMapID()>detection[k+1].getMapID()){
                temp=detection[k+1];
                detection[k+1]=detection[k];
                detection[k]=temp;
            }

        }

        //Inside map order by sector
        for (int m=0; m<detection.size()-1;m++){
            if (detection[m].getMapID() == detection[m+1].getMapID()){
            if (detection[m].getSectorID()>detection[m+1].getSectorID()){
                temp=detection[m+1];
                detection[m+1]=detection[m];
                detection[m]=temp;
            }
            }

        }

        //Inside sector order by marker ID
        for (int j=0; j<detection.size()-1;j++){
            if (detection[j].getMapID() == detection[j+1].getMapID()){
            if (detection[j].getSectorID() == detection[j+1].getSectorID()){
            if (detection[j].getMarkerID()>detection[j+1].getMarkerID()){
                temp=detection[j+1];
                detection[j+1]=detection[j];
                detection[j]=temp;
            }
        }
        }
    }
        }
        return detection;

    }

/*void cDetector::undistortPoints(cv::InputArray distorted, cv::OutputArray undistorted, cv::InputArray K, cv::InputArray D, cv::InputArray R, cv::InputArray P){
    //CV_INSTRUMENT_REGION()

    // will support only 2-channel data now for points
    CV_Assert(distorted.type() == CV_32FC2 || distorted.type() == CV_64FC2);
    undistorted.create(distorted.size(), distorted.type());

    CV_Assert(P.empty() || P.size() == cv::Size(3, 3) || P.size() == cv::Size(4, 3));
    CV_Assert(R.empty() || R.size() == cv::Size(3, 3) || R.total() * R.channels() == 3);
    CV_Assert(D.total() == 4 && K.size() == cv::Size(3, 3) && (K.depth() == CV_32F || K.depth() == CV_64F));

    cv::Vec2d f, c;
    if (K.depth() == CV_32F){
        cv::Matx33f camMat = K.getMat();
        f = cv::Vec2f(camMat(0, 0), camMat(1, 1));
        c = cv::Vec2f(camMat(0, 2), camMat(1, 2));
    } else {
        cv::Matx33d camMat = K.getMat();
        f = cv::Vec2d(camMat(0, 0), camMat(1, 1));
        c = cv::Vec2d(camMat(0, 2), camMat(1, 2));
    }

    cv::Vec4d k = D.depth() == CV_32F ? (cv::Vec4d)*D.getMat().ptr<cv::Vec4f>(): *D.getMat().ptr<cv::Vec4d>();

    cv::Matx33d RR = cv::Matx33d::eye();
    if (!R.empty() && R.total() * R.channels() == 3)
    {
        cv::Vec3d rvec;
        R.getMat().convertTo(rvec, CV_64F);
        RR = cv::Affine3d(rvec).rotation();
    }
    else if (!R.empty() && R.size() == cv::Size(3, 3))
        R.getMat().convertTo(RR, CV_64F);

    if(!P.empty())
    {
        cv::Matx33d PP;
        P.getMat().colRange(0, 3).convertTo(PP, CV_64F);
        RR = PP * RR;
    }

    // start undistorting
    const cv::Vec2f* srcf = distorted.getMat().ptr<cv::Vec2f>();
    const cv::Vec2d* srcd = distorted.getMat().ptr<cv::Vec2d>();
    cv::Vec2f* dstf = undistorted.getMat().ptr<cv::Vec2f>();
    cv::Vec2d* dstd = undistorted.getMat().ptr<cv::Vec2d>();

    size_t n = distorted.total();
    int sdepth = distorted.depth();

    for(size_t i = 0; i < n; i++ )
    {
        cv::Vec2d pi = sdepth == CV_32F ? (cv::Vec2d)srcf[i] : srcd[i];  // image point
        cv::Vec2d pw((pi[0] - c[0])/f[0], (pi[1] - c[1])/f[1]);      // world point

        double scale = 1.0;

        double theta_d = sqrt(pw[0]*pw[0] + pw[1]*pw[1]);

        // the current camera model is only valid up to 180° FOV
        // for larger FOV the loop below does not converge
        // clip values so we still get plausible results for super fisheye images > 180°
        theta_d = min(max(-CV_PI/2., theta_d), CV_PI/2.);

        if (theta_d > 1e-8)
        {
            // compensate distortion iteratively
            double theta = theta_d;
            for(int j = 0; j < 10; j++ )
            {
                double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta6*theta2;
                theta = theta_d / (1 + k[0] * theta2 + k[1] * theta4 + k[2] * theta6 + k[3] * theta8);
            }

            scale = std::tan(theta) / theta_d;
        }

        cv::Vec2d pu = pw * scale; //undistorted point

        // reproject
        cv::Vec3d pr = RR * cv::Vec3d(pu[0], pu[1], 1.0); // rotated point optionally multiplied by new camera matrix
        cv::Vec2d fi(pr[0]/pr[2], pr[1]/pr[2]);       // final

        if( sdepth == CV_32F )
            dstf[i] = fi;
        else
            dstd[i] = fi;
    }
}*/




