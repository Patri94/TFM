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
#include <trabajo/Marker.h>
#include <trabajo/Detector.h>
#include <particle_filter/particle_filterns.hpp>
#include <visualization_msgs/Marker.h>

using namespace cv;

namespace particle_filterns{
    /**
     * @brief particle_filter::particle_filter
     * Constructor of class particle filter
     * @param nh
     * @param nh_private
     */
    particle_filter::particle_filter ( const ros::NodeHandle& nh1,const ros::NodeHandle& nh_private1)
        :nh1_(nh1),nh_private1_(nh_private1)
    {
       // typedef const boost::function< void(const sensor_msgs::ImageConstPtr &)>  callback;
       // callback boundImageCallback = boost::bind(&particle_filter::imageCallback,this,_1);
       // image_transport::ImageTransport it(nh1_);
        //image_transport::Subscriber detector("DetectorOutput",nh_1,boundImageCallback);
        //image_transport::Subscriber detector= it.subscribe("DetectorOutput", 1, boundImageCallback);


        //Taking parameters from launch file

        XmlRpc::XmlRpcValue marker_list;
        nh1_.getParam("/particle_filter/IMAGE_WIDTH",image_width);
        nh1_.getParam("/particle_filter/MARKER_HEIGHT",marker_height);
        nh1_.getParam("/particle_filter/MARKER_WIDTH",marker_width);
        nh1_.getParam("/particle_filter/NUM_CAM",num_cam);
        nh1_.getParam("/particle_filter/marker_positions",marker_list);
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
        this->Publicar=nh1_.advertise<visualization_msgs::Marker> ("MarkerPose",1);
        this->PublicarCam1=nh1_.advertise<visualization_msgs::Marker> ("MarkerPoseCam1",1);
        this->PublicarCam2=nh1_.advertise<visualization_msgs::Marker> ("MarkerPoseCam2",1);
        this->PublicarCam3=nh1_.advertise<visualization_msgs::Marker> ("MarkerPoseCam3",1);
        this->PublicarMapa=nh1_.advertise<visualization_msgs::Marker> ("Mapa",1);
                detector_subs=nh1_.subscribe<sensor_msgs::Image> ("DetectorOutput",1,&particle_filter::imageCallback,this);


        this->LoadMap(IDs,Centros,marker_width,marker_height);
        this->LoadCameraInfo();

    }
    /**
     * @brief particle_filter::~particle_filter
     * Destructor of particle filter
     */
    particle_filter::~particle_filter(){

    }
    void particle_filter::imageCallback(const sensor_msgs::ImageConstPtr& msg){
        imagenfilter = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        cout<<"Callback"<<endl;
        imshow("Callback",imagenfilter);
        waitKey(30);

    }
    void particle_filter::LoadCameraInfo(void){
        sensor_msgs::CameraInfo cam_inf_ed;
        cam_inf_ed.header.frame_id="Cam1";
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

        this->PinModel.fromCameraInfo(cam_inf_ed);
        //this->cam1 = cv_bridge::toCvShare(msg, "bgr8")->image;
        // this->ms1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->cam1).toImageMsg();
    }


void particle_filter::LoadMap(std::vector<int>IDs,std::vector<geometry_msgs::Pose> Centros,float width, float height){

    this->PubMap.header.frame_id="ground_plane__link";
    this->PubMap.pose.orientation.w= 1.0;
    this->PubMap.scale.x=0.1;
    this->PubMap.scale.y=0.1;
    this->PubMap.scale.z=0.1;
    this->PubMap.ns= "spheres";
    this->PubMap.id = 0;
    this->PubMap.type = visualization_msgs::Marker::SPHERE_LIST;
    this->PubMap.action= visualization_msgs::Marker::ADD;
    this->PubMap.color.r = 1.0f;

    this->PubMap.color.a = 1.0;

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
                    relative_corner.point.x=width/2;
                    relative_corner.point.y=height/2;
                    relative_corner.point.z=0;

                    if(i==1 or i==2){
                            cout<<"entro1"<<endl;
                            relative_corner.point.x=-width/2;
                        }
                    if(i==0 or i==1){
                            cout<<"entro2"<<endl;
                            relative_corner.point.y=-height/2;
                        }
                    geometry_msgs::PointStamped global_corner;
                    tf2::doTransform(relative_corner,global_corner,tf_marker);
                    Marker.setCorner(global_corner.point);
                    PubMap.points.push_back(global_corner.point);
                }
            Marker.setMarkerId(IDs[i]);
            this->map.push_back(Marker);




        }







}

std::vector<cv::Point2d> particle_filter::Proyectar(visualization_msgs::Marker CamCoord1, float width, float cam){
    //TF to CAM1 (Center)
    geometry_msgs::Pose Cam1;
    Cam1.position.x=0.0;
    Cam1.position.y=0.0;
    Cam1.position.z=0.1;
    geometry_msgs::Quaternion QuatCam1;
    QuatCam1=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    Cam1.orientation=QuatCam1;
    tf::Quaternion QuatT;
    QuatT.setRPY(0,0,0);
    geometry_msgs::PoseStamped Cam1St;
    tf::Transform CamTCam1,invCamTCam1;
    geometry_msgs::TransformStamped invCamTCam1St,CamTCam1St;
    tf::Vector3 Trasl (Cam1.position.x,Cam1.position.y,Cam1.position.z);
    CamTCam1.setOrigin(Trasl);
    CamTCam1.setRotation(QuatT);
    invCamTCam1=CamTCam1.inverse();
    transformTFToMsg(CamTCam1,CamTCam1St.transform);
    transformTFToMsg(invCamTCam1,invCamTCam1St.transform);
    CamTCam1St.header.frame_id="camera_link";
    CamTCam1St.child_frame_id="Cam1";
    this->br.sendTransform(CamTCam1St);
    //TF to CAM2 (Left)
    geometry_msgs::Pose Cam2;
    Cam2.position.x=-0.08660254;
    Cam2.position.y=0.0;
    Cam2.position.z=-0.05;
    geometry_msgs::Quaternion QuatCam2;
    QuatCam2=tf::createQuaternionMsgFromRollPitchYaw(0,-2.093,0);
    Cam2.orientation=QuatCam2;
    tf::Quaternion QuatT2;
    QuatT2.setRPY(0,-2.093,0);
    geometry_msgs::PoseStamped Cam2St;
    tf::Transform CamTCam2,invCamTCam2;
    geometry_msgs::TransformStamped invCamTCam2St,CamTCam2St;
    tf::Vector3 Trasl2 (Cam2.position.x,Cam2.position.y,Cam2.position.z);
    CamTCam2.setOrigin(Trasl2);
    CamTCam2.setRotation(QuatT2);
    invCamTCam2=CamTCam2.inverse();
    transformTFToMsg(CamTCam2,CamTCam2St.transform);
    transformTFToMsg(invCamTCam2,invCamTCam2St.transform);
    CamTCam2St.header.frame_id="camera_link";
    CamTCam2St.child_frame_id="Cam2";
    this->br.sendTransform(CamTCam2St);
    //TF a Cam3 (Right)
    geometry_msgs::Pose Cam3;
    Cam3.position.x=0.08660254;
    Cam3.position.y=0.0;
    Cam3.position.z=-0.05;
    geometry_msgs::Quaternion QuatCam3;
    QuatCam3=tf::createQuaternionMsgFromRollPitchYaw(0,2.093,0);
    Cam3.orientation=QuatCam3;
    tf::Quaternion QuatT3;
    QuatT3.setRPY(0,2.093,0);
    geometry_msgs::PoseStamped Cam3St;
    tf::Transform CamTCam3,invCamTCam3;
    geometry_msgs::TransformStamped invCamTCam3St,CamTCam3St;
    tf::Vector3 Trasl3 (Cam3.position.x,Cam3.position.y,Cam3.position.z);
    CamTCam3.setOrigin(Trasl3);
    CamTCam3.setRotation(QuatT3);
    invCamTCam3=CamTCam3.inverse();
    transformTFToMsg(CamTCam3,CamTCam3St.transform);
    transformTFToMsg(invCamTCam3,invCamTCam3St.transform);
    CamTCam3St.header.frame_id="camera_link";
    CamTCam3St.child_frame_id="Cam3";
    this->br.sendTransform(CamTCam3St);

    //Corners in CAM1 Coordinates
    visualization_msgs::Marker MarkersCam1;
    MarkersCam1.header.frame_id="Cam1";
    MarkersCam1.pose.orientation.w= 1.0;
    MarkersCam1.scale.x=0.1;
    MarkersCam1.scale.y=0.1;
    MarkersCam1.scale.z=0.1;
    MarkersCam1.ns= "spheres";
    MarkersCam1.id = 0;
    MarkersCam1.type = visualization_msgs::Marker::SPHERE_LIST;
    MarkersCam1.action= visualization_msgs::Marker::ADD;
    MarkersCam1.color.r = 1.0f;

    MarkersCam1.color.a = 1.0;
    MarkersCam1.header.frame_id="Cam1";
    for (int i=0;i<CamCoord1.points.size();i++){
            geometry_msgs::PointStamped CoordCam1St,CoordCamTransSt;
            CoordCam1St.point=CamCoord1.points[i];
            tf2::doTransform(CoordCam1St,CoordCamTransSt,invCamTCam1St);
            MarkersCam1.points.push_back(CoordCamTransSt.point);
        }
    this->PublicarCam1.publish(MarkersCam1);
    //Corners in CAM2 Coordinates
   visualization_msgs::Marker MarkersCam2;
    MarkersCam2.header.frame_id="Cam2";
    for (int i=0;i<CamCoord1.points.size();i++){
            geometry_msgs::PointStamped CoordCam2St,CoordCamTransSt2;
            CoordCam2St.point=CamCoord1.points[i];
            tf2::doTransform(CoordCam2St,CoordCamTransSt2,invCamTCam2St);
            MarkersCam2.points.push_back(CoordCamTransSt2.point);
        }
    this->PublicarCam2.publish(MarkersCam2);
    //Corners in CAM3 Coordinates
    visualization_msgs::Marker MarkersCam3;
    MarkersCam3.header.frame_id="Cam3";
    for (int i=0;i<CamCoord1.points.size();i++){
            geometry_msgs::PointStamped CoordCam3St,CoordCamTransSt3;
            CoordCam3St.point=CamCoord1.points[i];
            tf2::doTransform(CoordCam3St,CoordCamTransSt3,invCamTCam3St);
            MarkersCam3.points.push_back(CoordCamTransSt3.point);
        }
    this->PublicarCam3.publish(MarkersCam3);

    //Which Camera?
    std::vector<cv::Point2d> Pixels;
    for (int i=0;i<CamCoord1.points.size();i++){
     //Angle respect to Z of CAM1
    float angulo;
    angulo = atan2(double(CamCoord1.points[i].x),double(CamCoord1.points[i].z));
    if (angulo<0){
            angulo=angulo+(2*M_PI);
        }
    cv::Point2d Pixel,offset,Pixel2;
    offset.x=width/cam;
    //cout<<"offset"<<offset<<endl;
    offset.y=0;
    cv::Point3d Coord;
    //CAM2
    if (angulo>M_PI and angulo<5.2333){
            Coord.x=MarkersCam2.points[i].x;
            Coord.y=MarkersCam2.points[i].y;
            Coord.z=MarkersCam2.points[i].z;
            Pixel=this->PinModel.project3dToPixel(Coord);
            Pixel=Pixel-offset;

        }else{
            //CAM3
            if(angulo>1.047 and angulo<M_PI){
                    Coord.x=MarkersCam3.points[i].x;
                    Coord.y=MarkersCam3.points[i].y;
                    Coord.z=MarkersCam3.points[i].z;
                    Pixel=this->PinModel.project3dToPixel(Coord);
                    Pixel=Pixel+offset;
                }else{//CAM1
                    Coord.x=MarkersCam1.points[i].x;
                    Coord.y=MarkersCam1.points[i].y;
                    Coord.z=MarkersCam1.points[i].z;
                    Pixel=this->PinModel.project3dToPixel(Coord);
                }
        }
     Pixels.push_back(Pixel);
     //cout<<Pixel.x<<" "<<Pixel.y<<endl;
        }

    return Pixels;

}


std::vector<geometry_msgs::Point> particle_filter::ObservationModel (Marcador Marca, geometry_msgs::Pose CamaraMundo){
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

void particle_filter::ErrorCalc(){
   // cout<<"llego"<<endl;
     cv::Mat copia = imagenfilter;
    // cout<<"llego"<<endl;
   // if (imagenfilter != imagenfilter){
    if(!(imagenfilter.empty())){
            for (int j=0;j<this->map.size();j++){
    geometry_msgs::Pose Supuesta,Supuesta2;
    Supuesta.position.x=0;
    Supuesta.position.y=0;
    Supuesta.position.z=0;
    tf::Quaternion Quat;
    tf::Matrix3x3 Mat;
    geometry_msgs::Quaternion QuatMs;
    Mat.setRPY(0,0,0);
    Mat.getRotation(Quat);
    tf::quaternionTFToMsg (Quat,QuatMs);
    Supuesta.orientation=QuatMs;
    std::vector<cv::Point2d> proyeccion;
    std::vector<geometry_msgs::Point> Relative=this->ObservationModel(this->map[j],Supuesta);
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
   this->Publicar.publish(CornersRelativePose);
   proyeccion=Proyectar(CornersRelativePose,image_width,num_cam);
   line (this->imagenfilter,proyeccion[0], proyeccion[1],Scalar(0,0,255),1);
   line (this->imagenfilter,proyeccion[1], proyeccion[2],Scalar(0,0,255),1);
   line (this->imagenfilter,proyeccion[2], proyeccion[3],Scalar(0,0,255),1);
   line (this->imagenfilter,proyeccion[3], proyeccion[0],Scalar(0,0,255),1);

   int ID=this->map[j].getMarkerID();
   std::string etiqueta ;
   ostringstream convert;
   convert<<ID;
   etiqueta=convert.str();
   putText(this->imagenfilter, etiqueta, proyeccion[0],CV_FONT_HERSHEY_COMPLEX,0.8,Scalar(0,0,255));
        }

   imshow("Prueba",this->imagenfilter);
   waitKey(30);
        }

}
}
