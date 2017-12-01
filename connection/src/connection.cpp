#include <sstream>
#include <fstream>
#include <connection/c_connection.h>

using namespace std;
int main(int argc, char **argv){

      ros::init(argc, argv, "DorisConnection");
      cConnection connection_obj;
      cout<<"created"<<endl;
      ros::Rate r (10);
      while(ros::ok()){
              ros::spinOnce();
              //ROS_INFO("Hoal /n");
                      //cout<<ros::Time::now()<<endl;
              connection_obj.readImage();
              connection_obj.readFromSocket();
              r.sleep();

          }


}

