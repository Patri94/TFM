#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
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
#include <detector/marcador.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <detector/marker.h>
#include <detector/messagedet.h>




using namespace tf;


class cDetector{
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    cv::Mat cam1,cam2,cam3,comb,gray,thresholded,canonicalMarkerImage;
    image_transport::Publisher pub_comb;
    sensor_msgs::ImagePtr comb_msg;
    geometry_msgs::Pose estimated_pose;
    float brightnessAvg;
    int minContourPointsAllowed,maxContourPointsAllowed,minContourLengthAllowed,maxContourLengthAllowed;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<Marcador> posiblesMarcas, Marcadores,OptMarkers,map;
    sensor_msgs::ImagePtr ms1, ms2, ms3;
    std::vector<cv::Point2f> markerCorners,DetectedCorners;
    cv::Size markerSize;
    ros::Subscriber odometry, sub_cam1, sub_cam2,sub_cam3;
    ros::Publisher publish_detection;
    //detector::detector msg_detected;

    //Constructor and Destructor
    cDetector(const ros::NodeHandle& nh,  const ros::NodeHandle& nh_private);
    cDetector(): cDetector(ros::NodeHandle(), ros::NodeHandle("~") ){}
   ~cDetector();

    //Callbacks
    void imageCallback2(const sensor_msgs::ImageConstPtr& msg );
    void imageCallback3(const sensor_msgs::ImageConstPtr& msg );
    void infoCallback(const sensor_msgs::ImageConstPtr& msg);

    //Detector
    void imageBlending(void);
    void brightnessAverage(void);
    void threshold(void);
    void imageTreatment(void);
    void Undistort(void);
    void findContours (void);
    void findCandidates(void);
    int hammingDistance(cv::Mat bits);
    void recognizeMarkers(void);
    float linearInterpolator(const float& x, const cv::Point p1, const cv::Point p2);
    int MarkerDecoder(const cv::Mat& inputGrayscale, int& nRrotations, Marcador &marker);
    void markerIdNumber(const cv::Mat &bits, int &mapId, int &sectorId, int &markerId);
    float perimeter(const std::vector<cv::Point2f> &a);
    cv::Mat rotate(cv::Mat input);
    void createMessage(void);
    void publish(void);
    int getMapSize(void);
    std::vector<Marcador> getMap(void);
    void OdomCallback (const nav_msgs::OdometryConstPtr& msg);
    void detectorTask(void);
    std::vector<Marcador> orderDetection(std::vector<Marcador> detection);

};

