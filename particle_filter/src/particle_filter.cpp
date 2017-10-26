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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <XmlRpcException.h>
#include <detector/marker.h>
#include <detector/messagedet.h>
#include <particle_filter/particle_filter.h>
#include <visualization_msgs/Marker.h>

using namespace cv;


    /**
     * @brief particle_filter::particle_filter
     * Constructor of class particle filter
     * @param nh
     * @param nh_private
     */
    ParticleFilter::ParticleFilter ( const ros::NodeHandle& nh1,const ros::NodeHandle& nh_private1)
        :nh1_(nh1),nh_private1_(nh_private1)
    {

        //Taking parameters from launch file

        XmlRpc::XmlRpcValue marker_list,camera_list;
        nh1_.getParam("/particle_filter/IMAGE_WIDTH",image_width);
        nh1_.getParam("/particle_filter/MARKER_HEIGHT",marker_height);
        nh1_.getParam("/particle_filter/MARKER_WIDTH",marker_width);
        nh1_.getParam("/particle_filter/NUM_CAM",num_cam);
        nh1_.getParam("/particle_filter/marker_positions",marker_list);
        nh1_.getParam("/particle_filter/camera_positions",camera_list);
        cout<<"Camaras"<<num_cam<<endl;
        //Reading mapfile
        std::vector<geometry_msgs::Pose> Centros;
        std::vector<int> IDs;
        for(int i=0;i<marker_list.size();i++){
                tf::Matrix3x3 orientation;
                tf::Quaternion Quat;
                geometry_msgs::Pose temp_pose;
                temp_pose.position.x=marker_list[i]["x"];
                temp_pose.position.y=marker_list[i]["y"];
                temp_pose.position.z=marker_list[i]["z"];
                double roll =marker_list[i]["roll"];
                double pitch =marker_list[i]["pitch"];
                double yaw =marker_list[i]["yaw"];
                orientation.setRPY (float(roll),float(pitch),float(yaw));
                orientation.getRotation(Quat);
                temp_pose.orientation.x = double(Quat.x());
                temp_pose.orientation.y = double(Quat.y());
                temp_pose.orientation.z = double(Quat.z());
                temp_pose.orientation.w = double(Quat.w());
                Centros.push_back(temp_pose);
                IDs.push_back(marker_list[i]["ID"]);

            }
        //Reading camerafile
        std::vector<geometry_msgs::Pose> cameras;
        for(int i=0;i<marker_list.size();i++){
                tf::Matrix3x3 orientation;
                tf::Quaternion Quat;
                geometry_msgs::Pose temp_pose;
                temp_pose.position.x=camera_list[i]["x"];
                temp_pose.position.y=camera_list[i]["y"];
                temp_pose.position.z=camera_list[i]["z"];
                double roll =camera_list[i]["roll"];
                double pitch =camera_list[i]["pitch"];
                double yaw =camera_list[i]["yaw"];
                orientation.setRPY (float(roll),float(pitch),float(yaw));
                orientation.getRotation(Quat);
                temp_pose.orientation.x = double(Quat.x());
                temp_pose.orientation.y = double(Quat.y());
                temp_pose.orientation.z = double(Quat.z());
                temp_pose.orientation.w = double(Quat.w());
                cameras.push_back(temp_pose);

            }
        this->publicar=nh1_.advertise<visualization_msgs::Marker> ("marker_pose",1);
        this->publicar_cam1=nh1_.advertise<visualization_msgs::Marker> ("marker_pose_cam1",1);
        this->publicar_cam2=nh1_.advertise<visualization_msgs::Marker> ("marker_pose_cam2",1);
        this->publicar_cam3=nh1_.advertise<visualization_msgs::Marker> ("marker_pose_cam3",1);
        this->publicar_mapa=nh1_.advertise<visualization_msgs::Marker> ("mapa",1);
        detector_subs=nh1_.subscribe<sensor_msgs::Image> ("DetectorNode/detector_output",1,&ParticleFilter::imageCallback,this);
        odom_subs=nh1_.subscribe<nav_msgs::Odometry> ("Doris/odom",1,&ParticleFilter::odomCallback,this);

        this->loadTFCameras(cameras);
        this->LoadMap(IDs,Centros);
        this->LoadCameraInfo();

    }
    /**
     * @brief particle_filter::~particle_filter
     * Destructor of particle filter
     */
    ParticleFilter::~ParticleFilter(){

    }
    void ParticleFilter::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        this->EstimatedPose=msg->pose.pose;

    }

    void ParticleFilter::imageCallback(const sensor_msgs::ImageConstPtr& msg){
        imagen_filter = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        cout<<"Callback"<<endl;
        imshow("Callback",imagen_filter);
        waitKey(30);

    }
    void ParticleFilter::LoadCameraInfo(void){
        sensor_msgs::CameraInfo cam_inf_ed;
        cam_inf_ed.header.frame_id="Doris/cam1";
        cam_inf_ed.height=679;
        cam_inf_ed.width=604;
        cam_inf_ed.distortion_model="plumb_bob";
        double Da[5]={-0.2601958609577983, 0.05505240192232372, 0.0, -0.0045449850126361765, 0.0};
        boost::array<double, 9ul> K={ {174.746839097, 0.0, 906.0, 0.0, 174.746839097, 339.5, 0.0, 0.0, 1.0} } ;
        boost::array<double, 9ul> R={ {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0} };
        boost::array<double, 12ul> P={ {174.64077512103418, 0.0, 906.0, 0.0, 0.0, 174.64077512103418, 339.5, 0.0, 0.0, 0.0, 1.0, 0.0} };
        std::vector<double> D(Da,Da +(sizeof(Da)/sizeof(Da[0])));
        cam_inf_ed.D=D;
        cam_inf_ed.K=K;
        cam_inf_ed.R=R;
        cam_inf_ed.P=P;
        cam_inf_ed.binning_x=0.0;
        cam_inf_ed.binning_y=0.0;
        cam_inf_ed.roi.height=0;
        cam_inf_ed.roi.width=0;

        this->pin_model.fromCameraInfo(cam_inf_ed);
    }


void ParticleFilter::LoadMap(std::vector<int>IDs,std::vector<geometry_msgs::Pose> Centros){

    this->pub_map.header.frame_id="ground_plane__link";
    this->pub_map.pose.orientation.w= 1.0;
    this->pub_map.scale.x=0.1;
    this->pub_map.scale.y=0.1;
    this->pub_map.scale.z=0.1;
    this->pub_map.ns= "spheres";
    this->pub_map.id = 0;
    this->pub_map.type = visualization_msgs::Marker::SPHERE_LIST;
    this->pub_map.action= visualization_msgs::Marker::ADD;
    this->pub_map.color.r = 1.0f;

    this->pub_map.color.a = 1.0;

    for (int i=0;i<Centros.size();i++){
            Marcador Marker;
            geometry_msgs::Pose marker_pose=Centros[i];
            geometry_msgs::TransformStamped tf_marker;
            tf_marker.header.frame_id="ground_plane__link";
            tf_marker.child_frame_id="Marca"+std::to_string(i);
            tf_marker.transform.translation.x=marker_pose.position.x;
            tf_marker.transform.translation.y=marker_pose.position.y;
            tf_marker.transform.translation.z=marker_pose.position.z;
            tf_marker.transform.rotation=marker_pose.orientation;

            //0=topleftcorner
            for (int i=0;i<4;i++){
                    geometry_msgs::PointStamped relative_corner;
                    relative_corner.point.x=marker_width/2;
                    relative_corner.point.y=marker_height/2;
                    relative_corner.point.z=0;

                    if(i==1 or i==2){
                            cout<<"entro1"<<endl;
                            relative_corner.point.x=-marker_width/2;
                        }
                    if(i==0 or i==1){
                            cout<<"entro2"<<endl;
                            relative_corner.point.y=-marker_height/2;
                        }
                    geometry_msgs::PointStamped global_corner;
                    tf2::doTransform(relative_corner,global_corner,tf_marker);
                    Marker.setCorner(global_corner.point);
                    pub_map.points.push_back(global_corner.point);
                }
            Marker.setMarkerId(IDs[i]);
            this->map.push_back(Marker);




        }







}

void ParticleFilter::loadTFCameras(std::vector<geometry_msgs::Pose> pose_cameras){
     cout<<"entro"<<endl;
     cout<<pose_cameras.size()<<endl;
    for (int i=0; i<pose_cameras.size();i++){
        tf::Vector3 Trasl (pose_cameras[i].position.x,pose_cameras[i].position.y,pose_cameras[i].position.z);
        geometry_msgs::TransformStamped inv_tf_cam_st,tf_cam_st;
        tf::Quaternion QuatT (pose_cameras[i].orientation.x,pose_cameras[i].orientation.y,pose_cameras[i].orientation.z,pose_cameras[i].orientation.w);
        tf::Transform tf_cam, inv_tfcam;
        tf_cam.setOrigin(Trasl);
        tf_cam.setRotation(QuatT);
        inv_tfcam=tf_cam.inverse();
        transformTFToMsg(tf_cam,tf_cam_st.transform);
        transformTFToMsg(inv_tfcam,inv_tf_cam_st.transform);
        inv_tf_cam_st.header.frame_id="camera_link";
        inv_tf_cam_st.child_frame_id="Cam"+to_string(i);
        tf_cameras.push_back(inv_tf_cam_st);
        this->br.sendTransform(inv_tf_cam_st);

    }





}

std::vector<cv::Point2d> ParticleFilter::Proyectar(visualization_msgs::Marker cam_center_coord, float width, float cam){
   geometry_msgs::PointStamped cam_center_coord_st,cam_trans_coord_st;
    std::vector<cv::Point2d> Pixels;
    for (int i=0;i<cam_center_coord.points.size();i++){
         cam_center_coord_st.point=cam_center_coord.points[i];
    float angulo;
    angulo = atan2(double(cam_center_coord.points[i].x),double(cam_center_coord.points[i].z));
    if (angulo<0){
            angulo=angulo+(2*M_PI);
        }
    cv::Point2d Pixel,offset;
    offset.x=width/cam;
    offset.y=0;
    cv::Point3d Coord;
    if (angulo>M_PI and angulo<5.2333){
           //camera2
            tf2::doTransform(cam_center_coord_st,cam_trans_coord_st,tf_cameras[1]);
            Coord.x=cam_trans_coord_st.point.x;
            Coord.y=cam_trans_coord_st.point.y;
            Coord.z=cam_trans_coord_st.point.z;
            Pixel=this->pin_model.project3dToPixel(Coord);
            Pixel=Pixel-offset;

        }else{
            //CAM3
            if(angulo>1.047 and angulo<M_PI){
                tf2::doTransform(cam_center_coord_st,cam_trans_coord_st,tf_cameras[2]);
                Coord.x=cam_trans_coord_st.point.x;
                Coord.y=cam_trans_coord_st.point.y;
                Coord.z=cam_trans_coord_st.point.z;
                Pixel=this->pin_model.project3dToPixel(Coord);
                Pixel=Pixel+offset;
                }else{//CAM1
                tf2::doTransform(cam_center_coord_st,cam_trans_coord_st,tf_cameras[0]);
                Coord.x=cam_trans_coord_st.point.x;
                Coord.y=cam_trans_coord_st.point.y;
                Coord.z=cam_trans_coord_st.point.z;
                Pixel=this->pin_model.project3dToPixel(Coord);
                }
        }
     Pixels.push_back(Pixel);
        }

    return Pixels;

}


std::vector<geometry_msgs::Point> ParticleFilter::ObservationModel (Marcador Marca, geometry_msgs::Pose CamaraMundo){
    //Pose CAM;
    tf::Transform MundTrob, invMundTrob,RobTCam,invRobotTCam;
    tf::Quaternion RotCam;
    //From Robot base to camera
    RotCam.setRPY(-M_PI/2,0,-M_PI/2);//Pich de M_PI/2
    RobTCam.setOrigin(tf::Vector3(0,0,1.4));
    RobTCam.setRotation(RotCam);
    tf::Quaternion QMundRCam (CamaraMundo.orientation.x,CamaraMundo.orientation.y,CamaraMundo.orientation.z,CamaraMundo.orientation.w);
    tf::Vector3 Trasl1 (CamaraMundo.position.x,CamaraMundo.position.y,CamaraMundo.position.z);
    //From World to Robot
    MundTrob.setRotation(QMundRCam);
    MundTrob.setOrigin(Trasl1);
    //Inverse the transformation--> inversa del mundo a la camara
    invRobotTCam=RobTCam.inverse();
    invMundTrob = MundTrob.inverse();
    geometry_msgs::TransformStamped MundTrobSt, RobotTCamSt;
    MundTrobSt.header.frame_id="ground_plane__link";
    MundTrobSt.child_frame_id="EstimatedPose";
    RobotTCamSt.header.frame_id="EstimatedPose";
    RobotTCamSt.child_frame_id="camera_link";
    transformTFToMsg(MundTrob,MundTrobSt.transform);
    transformTFToMsg(RobTCam,RobotTCamSt.transform);
    this->br.sendTransform(MundTrobSt);
    this->br.sendTransform(RobotTCamSt);

    //Pose Transformation
    geometry_msgs::TransformStamped invMundTrobStamped,invRobotTCamSt;
    transformTFToMsg(invMundTrob,invMundTrobStamped.transform);
    transformTFToMsg(invRobotTCam,invRobotTCamSt.transform);
    std::vector<geometry_msgs::Point> RelativaCorners,PoseWorld;
    //std::vector<geometry_msgs::Transform> Corners = Marca.getTransformCorners();
    PoseWorld=Marca.getPoseWorld();
    for (int i=0;i<4;i++){
            geometry_msgs::PointStamped CornerRelPose,Inter,WorldPose;
            WorldPose.point=PoseWorld[i];
             tf2::doTransform(WorldPose,Inter,invMundTrobStamped);
             tf2::doTransform(Inter,CornerRelPose,invRobotTCamSt);
            RelativaCorners.push_back(CornerRelPose.point);

        }
    //cout<<"Tengo la posicion relativa"<<endl;
    return RelativaCorners;



}

void ParticleFilter::ErrorCalc(){
     //base_link attached to cuerpo (z=0.05)
     cv::Mat copia = imagen_filter;
    if(!(imagen_filter.empty())){
            for (int j=0;j<this->map.size();j++){
    geometry_msgs::Pose Supuesta;
    Supuesta.position.x=0;
    Supuesta.position.y=0;
    Supuesta.position.z=0;
    EstimatedPose.position.z=0.0;
    tf::Quaternion Quat;
    tf::Matrix3x3 Mat;
    geometry_msgs::Quaternion QuatMs;
    Mat.setRPY(0,0,0);
    Mat.getRotation(Quat);
    tf::quaternionTFToMsg (Quat,QuatMs);
    Supuesta.orientation=QuatMs;
    std::vector<cv::Point2d> proyeccion;
    std::vector<geometry_msgs::Point> Relative=this->ObservationModel(this->map[j],EstimatedPose);
    //cout<<"AfterObsModel"<<endl;
    this->map[j].setRelativePose(Relative);
    visualization_msgs::Marker CornersRelativePose;
    CornersRelativePose.header.frame_id="camera_link";
    for (int i=0;i<4;i++){
            geometry_msgs::Point position = Relative[i];
            geometry_msgs::PointStamped msg;
            msg.header.frame_id="camera_link";
            msg.point=Relative[i];
            CornersRelativePose.points.push_back(Relative[i]);
        }
   this->publicar.publish(CornersRelativePose);
   proyeccion=Proyectar(CornersRelativePose,image_width,num_cam);
   line (this->imagen_filter,proyeccion[0], proyeccion[1],Scalar(0,0,255),1);
   line (this->imagen_filter,proyeccion[1], proyeccion[2],Scalar(0,0,255),1);
   line (this->imagen_filter,proyeccion[2], proyeccion[3],Scalar(0,0,255),1);
   line (this->imagen_filter,proyeccion[3], proyeccion[0],Scalar(0,0,255),1);

   int ID=this->map[j].getMarkerID();
   std::string etiqueta ;
   ostringstream convert;
   convert<<ID;
   etiqueta=convert.str();
   putText(this->imagen_filter, etiqueta, proyeccion[0],CV_FONT_HERSHEY_COMPLEX,0.8,Scalar(0,0,255));
        }

   imshow("Prueba",this->imagen_filter);
   waitKey(30);
        }

}

