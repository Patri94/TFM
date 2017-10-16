#include <particle_filter/particle_filter.h>

int main(int argc, char **argv){
      ros::init(argc, argv, "particle_filter");
      ros::Rate r (10);
      ParticleFilter particle_obj;
      while(ros::ok()){
              ros::spinOnce();
              r.sleep();
              particle_obj.ErrorCalc();
              particle_obj.publicar_mapa.publish(particle_obj.pub_map);
          }


}
