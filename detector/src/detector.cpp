
#include <detector/c_detector.h>

int main(int argc, char **argv){
      ros::init(argc, argv, "DetectorNode");
      cDetector detector_obj;
      ros::Rate r (10);
      while(ros::ok()){
              ros::spinOnce();
              detector_obj.detectorTask();
              r.sleep();

          }


}
