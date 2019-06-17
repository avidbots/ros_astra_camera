#include "astra_camera/multi_astra_driver.h"
#include "astra_camera/astra_driver.h"

int main(int argc, char **argv){

  ROS_INFO("multi_astra_camera_node init");
  ros::init(argc, argv, "multi_astra_camera_node");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  astra_wrapper::MultiAstraDriver drv(n, pnh);

  ros::spin();

  return 0;
}
