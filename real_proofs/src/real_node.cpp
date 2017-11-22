
#include <real_proofs/c_real.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "RealNode");
      cRealProofs real_obj;

      ros::Rate r (10);
      while(ros::ok()){
              ros::spinOnce();
              r.sleep();

          }


}
