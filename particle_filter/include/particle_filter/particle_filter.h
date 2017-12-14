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
#include <visualization_msgs/Marker.h>

using namespace tf;


    class ParticleFilter{
    public:
        ros::NodeHandle nh1_;
        ros::NodeHandle nh_private1_;
         cv::Mat imagen_filter;
        image_geometry::PinholeCameraModel pin_model;
        geometry_msgs::Pose EstimatedPose,Cam1,Cam2,Cam3;
        std::vector<geometry_msgs::TransformStamped> tf_cameras;
        ros::Publisher publicar,publicar_cam1,publicar_cam2,publicar_cam3,publicar_mapa,pub_centros;
        ros::Subscriber detector_subs, odom_subs;
        float  marker_width, num_cam,marker_height,image_width;
        TransformBroadcaster br;
        std::vector<Marcador> map;
        visualization_msgs::Marker pub_map;
        //nav_msgs::odometry odom_msg;
        int simulation;
        sensor_msgs::CameraInfo cam_inf_ed;
        Mat camMatrix, distCoeff,xi;
        geometry_msgs::PoseArray mapa;

        //Constructor and Destructor
       ParticleFilter(const ros::NodeHandle& nh1,  const ros::NodeHandle& nh_private1);
       ParticleFilter(): ParticleFilter(ros::NodeHandle(), ros::NodeHandle("~") ){}
       ~ParticleFilter();

        //Functions
        void LoadCameraInfo(void);
        void LoadMap(std::vector<int>maps,std::vector<int>sectors,std::vector<int>IDs,std::vector<geometry_msgs::Pose> Centros);
        void loadTFCameras(std::vector<geometry_msgs::Pose> pose_cameras);
        std::vector<geometry_msgs::Point> ObservationModel (Marcador Marca, geometry_msgs::Pose CamaraMundo);
        void ErrorCalc();
        std::vector<cv::Point2d> Proyectar(visualization_msgs::Marker CamCoord1,float width, float cam);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        void odomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

};

