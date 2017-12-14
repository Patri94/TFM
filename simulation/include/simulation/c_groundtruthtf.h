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
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

using namespace tf;


    class GroundTruth{
    public:
        ros::NodeHandle nh1_;
        ros::NodeHandle nh_private1_;

        ros::Subscriber gtruth_subs;
        TransformBroadcaster br;

        GroundTruth(const ros::NodeHandle& nh1,  const ros::NodeHandle& nh_private1);
        GroundTruth(): GroundTruth(ros::NodeHandle(), ros::NodeHandle("~") ){}

        void gTruthCallback (const nav_msgs::Odometry::ConstPtr& msg);
    };
