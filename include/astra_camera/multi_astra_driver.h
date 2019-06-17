#ifndef MULTI_ASTRA_DRIVER_H
#define MULTI_ASTRA_DRIVER_H

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

#include <astra_camera/AstraConfig.h>
#include "astra_camera/astra_device_manager.h"
#include "astra_camera/astra_device.h"
#include "astra_camera/astra_advanced_device.h"
#include "astra_camera/astra_video_mode.h"
#include "astra_camera/GetSerial.h"

#include "astra_camera/astra_device_manager.h"
#include "astra_camera/astra_advanced_driver.h"
#include "astra_camera/astra_registration_info.h"

namespace astra_wrapper
{

class MultiAstraDriver
{
public:
  MultiAstraDriver(ros::NodeHandle& n, ros::NodeHandle& pnh);
  ~MultiAstraDriver();

private:
  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;
  boost::shared_ptr<AstraDeviceManager> device_manager_;

  std::map<std::string, boost::shared_ptr<AstraAdvancedDriver>> astras_;

  ros::Subscriber astra_register_sub_;

  void RegistrationCallback(const astra_camera::astra_registration_info::ConstPtr& msg);
};

}

#endif
