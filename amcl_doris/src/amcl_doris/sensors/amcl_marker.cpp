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
                                   double sigma_hit,
                                   double landa)
{
  this->model_type = MARKER_MODEL_LIKELIHOOD;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->landa=landa;


}




////////////////////////////////////////////////////////////////////////////////
// Apply the laser sensor model
bool AMCLMarker::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  // Apply the laser sensor model
   // cout<<"update sensor marker cpp"<<endl;
 if(this->model_type == MARKER_MODEL_LIKELIHOOD)
    //cout<<"afterif"<<endl;
   /* if ((data==data)){
        cout<<"good data"<<endl;
    }
    if ((pf==pf)){
        cout<<"good pf"<<endl;
    }*/
    //cout<<(AMCLMarkerData*)data->markers_obs->size()<<endl;
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
  if (!self->image_filter.empty()){
  //self->z_hit=100;
  //self->z_hit=1.0;
  total_weight = 0.0;
  int i;
  std::vector<Marcador> detected_from_map;
  float gaussian_norm=1/(sqrt(2*M_PI*self->sigma_hit*self->sigma_hit));
  cout<<self->map.size()<<endl;
  cout<<observation.size()<<endl;
  //Extract only detected markers from map
  for(int k=0;k<observation.size();k++){
        for (int j=0; j<self->map.size();j++){
            cout<<"map"<<self->map[j].getMapID()<<endl;
            cout<<"sector"<<self->map[j].getSectorID()<<endl;
            cout<<"ID"<<self->map[j].getMarkerID()<<endl;
            cout<<"map"<<observation[k].getMapID()<<endl;
            cout<<"sector"<<observation[k].getSectorID()<<endl;
            cout<<"ID"<<observation[k].getMarkerID()<<endl;

            waitKey();
            if(self->map[j].getMarkerID()==observation[k].getMarkerID() && self->map[j].getSectorID()==observation[k].getSectorID() && self->map[j].getMapID()==observation[k].getMapID()){
                cout<<"+1"<<endl;
                waitKey();
                detected_from_map.push_back(self->map[j]);
            }

        }
  }
  cout<<"in map"<<detected_from_map.size()<<endl;
  /*for (int i=0;i<observation.size();i++){
      cout<<observation.getMarkerID()<<endl;
  }*/
  //waitKey();
  //cout<<"map"<<detected_from_map.size()<<endl;
  //cout<<"observed"<<observation.size()<<endl;
  //cout<<"camaras"<<self->num_cam<<endl;
  //cout<<"imagen"<<self->image_width<<endl;
  for (i=0;i< set->sample_count; i++){
      //cout<<"after 1st for"<<endl;
      sample=set-> samples + i;
      pose = sample->pose;
      p=1.0;
      //Initialize parameters
      double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
      //sqrt(2) beacuse of the normalization with height and width of image.
      double z_rand_mult=1.0/sqrt(2);
      //cout<<"after zhitdenom"<<z_hit_denom<<endl;
      geometry_msgs::Pose sample_pose;
      tf::Quaternion quat;
      geometry_msgs::Quaternion quat_msg;
      sample_pose.position.x=pose.v[0];
      sample_pose.position.y=pose.v[1];
      sample_pose.position.z=0.0;
      /*cout<<"Pose particula"<<endl;
      cout<<"x: "<<sample_pose.position.x<<endl;
      cout<<"y: "<<sample_pose.position.y<<endl;*/

      pose.v[2]=fmod(pose.v[2],2*M_PI);
      if (pose.v[2]<0){
          pose.v[2]=pose.v[2]+2*M_PI;
      }
      //cout<<pose.v[2]<<endl;
      quat.setRPY(0,0,pose.v[2]);
      tf::quaternionTFToMsg(quat,quat_msg);
      sample_pose.orientation=quat_msg;
      //cout<<"after building pose"<<endl;
      //cout<<"weight before"<<sample->weight<<endl;
      //cout<<"observation size"<<observation.size()<<endl;
      for (int j=0;j<observation.size();j++){
          //cout<<"mapID"<<detected_from_map[j].getMarkerID()<<endl;
          //cout<<"detectedID"<<observation[j].getMarkerID()<<endl;
          //waitKey();

          //Calculate projection of marker corners
          //cout<<"detectados"<<detected_from_map.size()<<endl;
         // cout<<"antes de relative pose"<<endl;
          std::vector<geometry_msgs::Point> relative_to_cam=self->CalculateRelativePose(detected_from_map[j],sample_pose);
          //cout<<"after relative pose"<<endl;
          std::vector<cv::Point2d> projection=self->projectPoints(relative_to_cam);
          //cout<<projection.size()<<endl;
        /*  line (self->image_filter,projection[0], projection[1],Scalar(0,0,255),1);
          line (self->image_filter,projection[1], projection[2],Scalar(0,0,255),1);
          line (self->image_filter,projection[2], projection[3],Scalar(0,0,255),1);
          line (self->image_filter,projection[3], projection[0],Scalar(0,0,255),1);
          imshow("Proyeccion",self->image_filter);
          waitKey(30);*/
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
          //Compute probability for every corner
          z=self->calculateError(observation[j].getMarkerPoints(),projection);

          for (int i=0;i<4;i++){
              pz=0.0;
              //Opción1:Gaussian model
              pz+=self->z_hit*exp(-(z[i]*z[i]) / z_hit_denom);
              //Random measurements
              pz+=self->z_rand*z_rand_mult;
             // cout<<"pz: "<<pz<<endl;
              p+=pz*pz*pz;
              //Opción 2:Distribución exponencial (Humanoid P12)
              //pz+=z[i];
              //p+=self->landa*exp(-self->landa*pz);
          }

          //cout<<"despues de mean error"<<endl;
          //Gaussian model
          //cout<<"z_hit"<<self->z_hit<<endl;
         // cout<<"z_hit"<<self->z_hit<<endl;
          //cout<<"z_hit_denom"<<z_hit_denom<<endl;

          //cout<<"pz:"<<pz<<endl;
          //cout<< "despues de Gaussian model"<<endl;

          if (pz>1.0){
              cout<<"mayor"<<endl;
          }
          //Exception if non-consistent result
          //assert(pz <= 1.0);
          //assert(pz >= 0.0);

         // cout<<"p:"<<p<<endl;

      }
      sample->weight *= p;
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
    //cout<<"entro en mean error"<<endl;
    //normalizing error with width and height of image.
    float image_height=679;

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
    RobTCam.setOrigin(tf::Vector3(0,0,1.3925));
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
