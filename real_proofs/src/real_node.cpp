
#include <real_proofs/c_real.h>

int main(int argc, char **argv){
      Aria::init();
      cRealProofs real_obj;
      real_obj.robot= new ArRobot();
      real_obj.connector = new ArRobotConnector();
      real_obj.connector->connectRobot();
      ros::init(argc, argv, "RealNode");
      ros::Rate r (10);
      while(ros::ok()){
              ros::spinOnce();
              r.sleep();

          }


}
