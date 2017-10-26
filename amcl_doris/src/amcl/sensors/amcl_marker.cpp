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

#include "amcl/sensors/amcl_marker.h"

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLMarker::AMCLMarker() : AMCLSensor()

{

  //this->map = map;
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
                                   double sigma_hit)
{
  this->model_type = MARKER_MODEL_LIKELIHOOD;
  this->z_hit = z_hit;
  //this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;


}




////////////////////////////////////////////////////////////////////////////////
// Apply the laser sensor model
bool AMCLMarker::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  // Apply the laser sensor model
    cout<<"update sensor marker cpp"<<endl;
 if(this->model_type == MARKER_MODEL_LIKELIHOOD)
    cout<<"afterif"<<endl;
    //cout<<data->sensor->markers_obs.size()<<endl;
    pf_update_sensor(pf, (pf_sensor_model_fn_t) ObservationLikelihood, data);
  return true;
}




double AMCLMarker::ObservationLikelihood(AMCLMarkerData *data, pf_sample_set_t* set)
{
  //cout<<"in particle filter"<<endl;
  AMCLMarker *self;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;
  double total_weight;
  double z, pz,p;
  self = (AMCLMarker*) data->sensor;
  if (!self->image_filter.empty()){
  //self->z_hit=100;
  self->z_hit=1.0;
  self->sigma_hit=100;
  total_weight = 0.0;
  int i;
  std::vector<Marcador> detected_from_map;
  float gaussian_norm=1/(sqrt(2*M_PI*self->sigma_hit));

  //Extract only detected markers from map
  for(int k=0;k<data->markers_obs.size();k++){
        for (int j=0; j<self->map.size();j++){
            if(self->map[j].getMarkerID()==data->markers_obs[k].getMarkerID()){
                detected_from_map.push_back(self->map[j]);
            }

        }
  }
  /*for (int i=0; i<data->markers_obs.size();i++){
      cout<<"IDs observados"<<data->markers_obs[i].getMarkerID()<<endl;
  }
  for (int i=0; i<detected_from_map.size();i++){
      cout<<"IDs mapa"<<detected_from_map[i].getMarkerID()<<endl;
  }*/
   //cout<<"number of samples"<<set->sample_count<<endl;
   //waitKey();

  for (i=0;i< set->sample_count; i++){
      //cout<<"after 1st for"<<endl;
      sample=set-> samples + i;
      pose = sample->pose;
      p=1.0;
      //Initialize parameters
      double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
      //cout<<"after zhitdenom"<<endl;
      geometry_msgs::Pose sample_pose;
      tf::Quaternion quat;
      geometry_msgs::Quaternion quat_msg;
      sample_pose.position.x=pose.v[1];
      sample_pose.position.y=pose.v[2];
      sample_pose.position.z=0.0;
      quat.setRPY(0,0,pose.v[3]);
      tf::quaternionTFToMsg(quat,quat_msg);
      sample_pose.orientation=quat_msg;
      //cout<<"after building pose"<<endl;
      std::vector<Marcador> observation=data->markers_obs;
      //cout<<"observation size"<<observation.size()<<endl;
      for (int j=0;j< data->markers_obs.size();j++){
          //cout<<"mapID"<<detected_from_map[j].getMarkerID()<<endl;
          //cout<<"detectedID"<<observation[j].getMarkerID()<<endl;
          //waitKey();
          pz = 0.0;
          //Calculate projection of marker corners
          //cout<<"detectados"<<detected_from_map.size()<<endl;
          std::vector<geometry_msgs::Point> relative_to_cam=self->CalculateRelativePose(detected_from_map[j],sample_pose);
          //cout<<"after relative pose"<<endl;
          std::vector<cv::Point2d> projection=self->projectPoints(relative_to_cam);
          //cout<<projection.size()<<endl;
          line (self->image_filter,projection[0], projection[1],Scalar(0,0,255),1);
          line (self->image_filter,projection[1], projection[2],Scalar(0,0,255),1);
          line (self->image_filter,projection[2], projection[3],Scalar(0,0,255),1);
          line (self->image_filter,projection[3], projection[0],Scalar(0,0,255),1);
          imshow("Proyeccion",self->image_filter);
          waitKey(30);
          //cout<<"after image"<<endl;
          //Calculate mean error in pixels
          //cout<<projection.size()<<endl;
          //cout<<observation[j].getMarkerPoints().size()<<endl;
          //cout<<"anted de error"<<endl;
          //cout<<"detected"<<endl;
          std::vector<cv::Point2f> Puntos=observation[j].getMarkerPoints();
          /*for(int i=0;i<4;i++){
              cout<<"xdet:"<<Puntos[i].x<<endl;
              cout<<"ydet:"<<Puntos[i].y<<endl;
              cout<<"xmap:"<<projection[i].x<<endl;
              cout<<"ymap:"<<projection[i].y<<endl;
          }*/
          //waitKey();
          z=self->calculateMeanError(observation[j].getMarkerPoints(),projection);
          //cout<<"despues de mean error"<<endl;
          //Gaussian model
          //cout<<"z_hit"<<self->z_hit<<endl;
         // cout<<"z_hit"<<self->z_hit<<endl;
          //cout<<"z_hit_denom"<<z_hit_denom<<endl;
          pz+=gaussian_norm* exp(-(z*z) / z_hit_denom);
          //cout<<"pz:"<<pz<<endl;
          //cout<< "despues de Gaussian model"<<endl;
          if (pz>1.0){
              cout<<"mayor"<<endl;
          }
          //Exception if non-consistent result
          //assert(pz <= 1.0);
          //assert(pz >= 0.0);

          //Combination of prababilities
          p += pz*pz*pz;
          //cout<<"p:"<<p<<endl;

      }
      sample->weight *= p;
      total_weight += sample->weight;
      //cout<<"despues de asignar peso a particula"<<endl;


  }

  return(total_weight);
  //cout<<"hesalido Marker"<<endl;
  }
}
float AMCLMarker::calculateMeanError(std::vector<cv::Point2f> projection_detected, std::vector<cv::Point2d> projection_map){
    //cout<<"entro en mean error"<<endl;
    float error=0;
    for (int i=0;i<4;i++){
        float errorx,errory;
        errorx=abs(projection_map[i].x-projection_detected[i].x);
        //cout<<"errorx:"<<errorx<<endl;
        errory=abs(projection_map[i].y-projection_detected[i].y);
        //cout<<"errory:"<<errory<<endl;
        error+=sqrt((errorx*errorx)+(errory*errory));
        //cout<<"error"<<error<<endl;
    }
    //cout<<"error"<<error<<endl;
    return error;

}

std::vector<cv::Point2d> AMCLMarker::projectPoints(std::vector<geometry_msgs::Point> cam_center_coord){
   geometry_msgs::PointStamped cam_center_coord_st,cam_trans_coord_st;
    std::vector<cv::Point2d> Pixels;
    for (int i=0;i<cam_center_coord.size();i++){
         cam_center_coord_st.point=cam_center_coord[i];
    float angulo;
    angulo = atan2(double(cam_center_coord[i].x),double(cam_center_coord[i].z));
    if (angulo<0){
            angulo=angulo+(2*M_PI);
        }
    cv::Point2d Pixel,offset;
    //cout<<num_cam<<" "<<image_width<<endl;
    offset.x=image_width/num_cam;
    offset.y=0;
    //cout<<"offset"<<offset.x;
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

void AMCLMarker::LoadCameraInfo(void){
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

    this->pin_model.fromCameraInfo(cam_inf_ed);

}

std::vector<geometry_msgs::Point> AMCLMarker::CalculateRelativePose (Marcador Marca, geometry_msgs::Pose CamaraMundo){
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
    //this->br_marker.sendTransform(MundTrobSt);
    //this->br_marker.sendTransform(RobotTCamSt);

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

/*void AMCLLaser::reallocTempData(int new_max_samples, int new_max_obs){
  if(temp_obs){
    for(int k=0; k < max_samples; k++){
      delete [] temp_obs[k];
    }
    delete []temp_obs; 
  }
  max_obs = new_max_obs; 
  max_samples = fmax(max_samples, new_max_samples); 

  temp_obs = new double*[max_samples]();
  for(int k=0; k < max_samples; k++){
    temp_obs[k] = new double[max_obs]();
  }
}*/
