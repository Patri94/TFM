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
#include <trabajo/Marcador.hpp>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

using namespace tf;

namespace particle_filterns{
    class particle_filter{
    public:
        ros::NodeHandle nh1_;
        ros::NodeHandle nh_private1_;
         cv::Mat imagenfilter;
        image_geometry::PinholeCameraModel PinModel;
        geometry_msgs::Pose EstimatedPose;
        ros::Publisher Publicar,PublicarCam1,PublicarCam2,PublicarCam3,PublicarMapa;
        ros::Subscriber detector_subs;
        float  marker_width, num_cam,marker_height,image_width;
        TransformBroadcaster br;
        std::vector<Marcador> map;
        visualization_msgs::Marker PubMap;

        sensor_msgs::CameraInfo cam_inf_ed;

        //Constructor and Destructor
        particle_filter(const ros::NodeHandle& nh1,  const ros::NodeHandle& nh_private1);
        particle_filter(): particle_filter(ros::NodeHandle(), ros::NodeHandle("~") ){}
        ~particle_filter();

        //Functions
        void LoadCameraInfo(void);
        void LoadMap(std::vector<int>IDs,std::vector<geometry_msgs::Pose> Centros,float width, float height);
        std::vector<geometry_msgs::Point> ObservationModel (Marcador Marca, geometry_msgs::Pose CamaraMundo);
        void ErrorCalc();
        std::vector<cv::Point2d> Proyectar(visualization_msgs::Marker CamCoord1,float width, float cam);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

};
}
