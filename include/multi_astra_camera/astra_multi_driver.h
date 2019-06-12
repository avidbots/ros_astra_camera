#ifndef ASTRA_MULTI_DRIVER_H
#define ASTRA_MULTI_DRIVER_H

#include <ros/ros.h>

#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <sensor_msgs/Image.h>
#include <std_srvs/SetBool.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <multi_astra_camera/AstraConfig.h>
#include "multi_astra_camera/astra_device_manager.h"
#include "multi_astra_camera/astra_device.h"
#include "multi_astra_camera/astra_advanced_device.h"
#include "multi_astra_camera/astra_video_mode.h"
#include "multi_astra_camera/GetSerial.h"

#include "multi_astra_camera/astra_device_manager.h"
#include "multi_astra_camera/astra_driver.h"
#include "multi_astra_camera/astra_registration_info.h"

namespace astra_wrapper
{

class AstraMultiDriver
{
public:
  AstraMultiDriver(ros::NodeHandle& n, ros::NodeHandle& pnh);
  ~AstraMultiDriver();

private:
  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;
  boost::shared_ptr<AstraDeviceManager> device_manager_;

  std::map<std::string, boost::shared_ptr<AstraDriver>> astras_;

  ros::Subscriber astra_register_sub_;

  void RegistrationCallback(const multi_astra_camera::astra_registration_info::ConstPtr& msg);
};

}

#endif
