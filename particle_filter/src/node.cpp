#include <particle_filter/particle_filterns.hpp>

int main(int argc, char **argv){
      ros::init(argc, argv, "particle_filter");
      ros::NodeHandle nh, nh2;
      ros::Rate r (10);
      particle_filterns::particle_filter ParticleObj;
      while(ros::ok()){
              ros::spinOnce();
              r.sleep();
              ParticleObj.ErrorCalc();
              ParticleObj.PublicarMapa.publish(ParticleObj.PubMap);
          }


}
