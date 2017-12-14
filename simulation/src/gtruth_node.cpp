#include <simulation/c_groundtruthtf.h>

int main(int argc, char **argv){
      ros::init(argc, argv, "ground_truth");
      ros::NodeHandle nh;
      ros::Rate r (10);
      GroundTruth gtruth_obj;
      while(ros::ok()){
              ros::spinOnce();
          }


}
