/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL Marler routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_marker.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <vector>

#include "amcl_doris/sensors/amcl_marker.h"

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLMarker::AMCLMarker(int simualtion) : AMCLSensor()

{

  //this->map = map;
  this->simulation=simulation;
  this->LoadCameraInfo();


  return;
}

AMCLMarker::~AMCLMarker()
{
  if(temp_obs.size()>0){
      temp_obs.clear();

  }
}


void 
AMCLMarker::SetModelLikelihoodField(double z_hit,
                                   double z_rand,
                                   double sigma_hit,
                                   double landa,
                                   double marker_coeff)
{
  this->model_type = MARKER_MODEL_LIKELIHOOD;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->landa=landa;
  this->marker_coeff=marker_coeff;
}




////////////////////////////////////////////////////////////////////////////////
// Apply the laser sensor model
bool AMCLMarker::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  // Apply the laser sensor model
   // cout<<"update sensor marker cpp"<<endl;
 if(this->model_type == MARKER_MODEL_LIKELIHOOD)
    pf_update_sensor(pf, (pf_sensor_model_fn_t) ObservationLikelihood, data);
  return true;
}




double AMCLMarker::ObservationLikelihood(AMCLMarkerData *data, pf_sample_set_t* set)
{
  cout<<"in particle filter"<<endl;

  AMCLMarker *self;

  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;
  double total_weight;
  double pz,p;
  std::vector<float> z;
  self = (AMCLMarker*) data->sensor;
  std::vector<Marcador> observation=data->markers_obs;
  //cout<<"landa in likelihood"<<self->landa<<endl;
  if (!self->image_filter.empty()){

  total_weight = 0.0;
  int i;
  std::vector<Marcador> detected_from_map;
  float gaussian_norm=1/(sqrt(2*M_PI*self->sigma_hit*self->sigma_hit));

  for(int k=0;k<observation.size();k++){
        for (int j=0; j<self->map.size();j++){

            if(self->map[j].getMarkerID()==observation[k].getMarkerID() && self->map[j].getSectorID()==observation[k].getSectorID() && self->map[j].getMapID()==observation[k].getMapID()){
                //cout<<"+1"<<endl;
                //waitKey();
                detected_from_map.push_back(self->map[j]);
            }

        }
  }
  for (i=0;i< set->sample_count; i++){
      //cout<<"after 1st for"<<endl;
      sample=set-> samples + i;
      pose = sample->pose;
      p=1.0;

      //Initialize parameters
      double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
      //sqrt(2) beacuse of the normalization with height and width of image.
      double z_rand_mult=1.0/sqrt(2);


      geometry_msgs::Pose sample_pose;
      tf::Quaternion quat;
      geometry_msgs::Quaternion quat_msg;
      sample_pose.position.x=pose.v[0];
      sample_pose.position.y=pose.v[1];
      sample_pose.position.z=0.0;

      pose.v[2]=fmod(pose.v[2],2*M_PI);
      if (pose.v[2]<0){
          pose.v[2]=pose.v[2]+2*M_PI;
      }
      //cout<<pose.v[2]<<endl;
      quat.setRPY(0,0,pose.v[2]);
      tf::quaternionTFToMsg(quat,quat_msg);
      sample_pose.orientation=quat_msg;

      for (int j=0;j<observation.size();j++){

          //Calculate projection of marker corners
          std::vector<geometry_msgs::Point> relative_to_cam=self->CalculateRelativePose(detected_from_map[j],sample_pose);
          //cout<<"after relative pose"<<endl;
           std::vector<cv::Point2d> projection;
           if (self->simulation == 1){
               //cout<<"simu"<<endl;
           projection=self->projectPoints(relative_to_cam);
            }
          if(self->simulation == 0){
              //cout<<"real"<<endl;
               std::vector<cv::Point3f>rel;
               //cout<<"llego"<<endl;
               for (int i=0; i< relative_to_cam.size(); i++){
                   cout<<"llegodentro"<<endl;
                                 cv::Point3d Coord;
                                 Coord.x=relative_to_cam[i].x;
                                 Coord.y=relative_to_cam[i].y;
                                 Coord.z=relative_to_cam[i].z;
                                 rel.push_back(Coord);

                             }
               std::vector<cv::Point2f> imagePoints;
               cout<<"llego"<<endl;
               cv::Mat rvec(3,1,cv::DataType<double>::type);
               cv::Mat tvec(3,1,cv::DataType<double>::type);
               rvec.at<double>(0)=0.0;
               rvec.at<double>(1)=0.0;
               rvec.at<double>(2)=0.0;
               tvec.at<double>(0)=0.0;
               tvec.at<double>(1)=0.0;
               tvec.at<double>(2)=0.0;
               cout<<"antes de proyectar"<<endl;
               cout<<rel.size()<<endl;

           cv::omnidir::projectPoints(rel,imagePoints,rvec,tvec,self->camMatrix,self->xi,self->distCoeff);
           cout<<imagePoints.size()<<endl;
           cout<<"despues de proyectar1"<<endl;
           for(int i=0; i<imagePoints.size();i++){
                projection.push_back( cv::Point2d( (double)imagePoints[i].x, (double)imagePoints[i].y  ) );
           }
           cout<<"despues de proyectar"<<endl;
           }

          //Calculate mean error in pixels
          std::vector<cv::Point2f> Puntos=observation[j].getMarkerPoints();
          //Compute probability for every corner
          z=self->calculateError(observation[j].getMarkerPoints(),projection);
          float ztot=std::accumulate(z.begin(), z.end(), 0.0);
          for (int i=0;i<4;i++){
              pz=0.0;
              //Opción1:Gaussian model
              //pz+=self->z_hit*exp(-(z[i]*z[i]) / z_hit_denom);
              //Random measurements
              //pz+=self->z_rand*z_rand_mult;
             // cout<<"pz: "<<pz<<endl;
              //p+=pz*pz*pz;
              //Opción 2:Distribución exponencial (Humanoid P12)
              //pz+=z[i];
              pz+=self->landa*exp(-self->landa*ztot);
              p+=pz*pz*pz;
          }

        /*  if (pz>1.0){
              cout<<"mayor"<<endl;
          }*/

      }
      sample->weight *= self->marker_coeff * p;
      //cout<<"weight of sample"<<sample->weight<<endl;
      total_weight += sample->weight;
      //cout<<"despues de asignar peso a particula"<<endl;


  }
  //cout<<"total weight"<<total_weight<<endl;
  return(total_weight);
  //cout<<"hesalido Marker"<<endl;
  }
}
std::vector<float> AMCLMarker::calculateError(std::vector<cv::Point2f> projection_detected, std::vector<cv::Point2d> projection_map){

    //normalizing error with width and height of image.
    //float image_height=679;

    std::vector<float> errorv;
    for (int i=0;i<4;i++){
        float errorx,errory;
        float error=0.0;
        errorx=abs(projection_map[i].x-projection_detected[i].x)/image_width;
        //cout<<"errorx:"<<errorx<<endl;
        errory=abs(projection_map[i].y-projection_detected[i].y)/image_height;
        //cout<<"errory:"<<errory<<endl;
        error+=sqrt((errorx*errorx)+(errory*errory));
        errorv.push_back(error);
        //cout<<"error"<<error<<endl;
        if(error>sqrt(2)){
            waitKey();
        }
    }
    //cout<<"error"<<error<<endl;
    return errorv;

}

std::vector<cv::Point2d> AMCLMarker::projectPoints(std::vector<geometry_msgs::Point> cam_center_coord){
   //cout<<"entra en relative"<<endl;
   geometry_msgs::PointStamped cam_center_coord_st,cam_trans_coord_st;
    std::vector<cv::Point2d> Pixels;
    for (int i=0;i<cam_center_coord.size();i++){
         cam_center_coord_st.point=cam_center_coord[i];
    float angulo;
    angulo = atan2(double(cam_center_coord[i].x),double(cam_center_coord[i].z));
    angulo=fmod(angulo,2*M_PI);
    if (angulo<0){
            angulo=angulo+(2*M_PI);
        }
    //cout<<"angulo"<<angulo<<endl;
    cv::Point2d Pixel,offset;
    //cout<<num_cam<<" "<<image_width<<endl;
    offset.x=image_width/num_cam;
    offset.y=0;
    //cout<<"offset"<<offset.x;
    cv::Point3d Coord;
    if (angulo>M_PI and angulo<5.2333333){
           //camera2
            //cout<<"entra2"<<endl;
            tf2::doTransform(cam_center_coord_st,cam_trans_coord_st,tf_cameras[1]);
            Coord.x=cam_trans_coord_st.point.x;
            Coord.y=cam_trans_coord_st.point.y;
            Coord.z=cam_trans_coord_st.point.z;
            Pixel=this->pin_model.project3dToPixel(Coord);
            Pixel=Pixel-offset;

        }else{
            //CAM3
            if(angulo>M_PI/3 and angulo<M_PI){
                //cout<<"entra3"<<endl;
                tf2::doTransform(cam_center_coord_st,cam_trans_coord_st,tf_cameras[2]);
                Coord.x=cam_trans_coord_st.point.x;
                Coord.y=cam_trans_coord_st.point.y;
                Coord.z=cam_trans_coord_st.point.z;
                Pixel=this->pin_model.project3dToPixel(Coord);
                Pixel=Pixel+offset;
                }else{//CAM1
                //cout<<"entra1"<<endl;
                tf2::doTransform(cam_center_coord_st,cam_trans_coord_st,tf_cameras[0]);
                Coord.x=cam_trans_coord_st.point.x;
                Coord.y=cam_trans_coord_st.point.y;
                Coord.z=cam_trans_coord_st.point.z;
                Pixel=this->pin_model.project3dToPixel(Coord);
                }
        }
     Pixels.push_back(Pixel);
        }
    //cout<<"sale"<<endl;

    return Pixels;

}

void AMCLMarker::LoadCameraInfo(void){
    sensor_msgs::CameraInfo cam_inf_ed;
    //if (this->simulation == 1){
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
   //}
   //if (this->simulation == 0){
    camMatrix = cv::Mat(3, 3, CV_32F);
    camMatrix.at<float>(0, 0) = 8.5101024687735935e+02;
    camMatrix.at<float>(0, 1) = -2.2255059056366439e-01;
    camMatrix.at<float>(0, 2) = 6.5571465382877625e+02;
    camMatrix.at<float>(1, 0) = 0.0;
    camMatrix.at<float>(1, 1) = 8.5170243585411265e+02;;
    camMatrix.at<float>(1, 2) = 5.1216084358475405e+02;
    camMatrix.at<float>(2, 0) = 0.0;
    camMatrix.at<float>(2, 1) = 0.0;
    camMatrix.at<float>(2, 2) = 1.0;

    distCoeff = cv::Mat(4, 1, CV_32F);
    distCoeff.at<float>(0, 0) = -4.2648301140911193e-01;
    distCoeff.at<float>(1, 0) = 3.1105618959437248e-01;
    distCoeff.at<float>(2, 0) = -1.3775384616268102e-02;
    distCoeff.at<float>(3, 0) = -1.9560559208606078e-03;
    distCoeff.at<float>(3, 0) = 0;

    xi=1.5861076761699640e+00;

   // }


    this->pin_model.fromCameraInfo(cam_inf_ed);

}

std::vector<geometry_msgs::Point> AMCLMarker::CalculateRelativePose (Marcador Marca, geometry_msgs::Pose CamaraMundo){
    //Pose CAM;
    tf::Transform MundTrob, invMundTrob,RobTCam,invRobotTCam;
    tf::Quaternion RotCam;
    //From Robot base to camera
   //Pitch de M_PI/2
   switch (this->simulation){
    case 1:
    {
        cout<<"simuproy"<<endl;
     RotCam.setRPY(-M_PI/2,0,-M_PI/2);
    RobTCam.setOrigin(tf::Vector3(0,0,1.3925));
    break;
    }
    case 0:
        cout<<"realrelative"<<endl;
    {
        RotCam.setRPY(0,0,-M_PI/2);
        RobTCam.setOrigin(tf::Vector3(-0.26,0,1.41));
        break;
    }
    default:
        break;
    }
    //RobTCam.setOrigin(tf::Vector3(0,0,1.3925));
    RobTCam.setRotation(RotCam);
    tf::Quaternion QMundRCam (CamaraMundo.orientation.x,CamaraMundo.orientation.y,CamaraMundo.orientation.z,CamaraMundo.orientation.w);
    tf::Vector3 Trasl1 (CamaraMundo.position.x,CamaraMundo.position.y,CamaraMundo.position.z);
    //cout<<"after transformation"<<endl;
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
    //this->br_marker.sendTransform(MundTrobSt);
    //this->br_marker.sendTransform(RobotTCamSt);

    //Pose Transformation
    geometry_msgs::TransformStamped invMundTrobStamped,invRobotTCamSt;
    transformTFToMsg(invMundTrob,invMundTrobStamped.transform);
    transformTFToMsg(invRobotTCam,invRobotTCamSt.transform);
    std::vector<geometry_msgs::Point> RelativaCorners,PoseWorld;
    //std::vector<geometry_msgs::Transform> Corners = Marca.getTransformCorners();
    //cout<<"antes de get pose world"<<endl;
    PoseWorld=Marca.getPoseWorld();
    for (int i=0;i<4;i++){
            geometry_msgs::PointStamped CornerRelPose,Inter,WorldPose;
            WorldPose.point=PoseWorld[i];
             tf2::doTransform(WorldPose,Inter,invMundTrobStamped);
             tf2::doTransform(Inter,CornerRelPose,invRobotTCamSt);
             RelativaCorners.push_back(CornerRelPose.point);

        }
    //cout<<"Tengo la posicion relativa"<<endl;
    cout<<"salgo"<<endl;
    return RelativaCorners;



}


