#include "multi_astra_camera/astra_multi_driver.h"
#include <boost/make_shared.hpp>

namespace astra_wrapper
{

AstraMultiDriver::AstraMultiDriver(ros::NodeHandle& n, ros::NodeHandle& pnh) :
    nh_(n),
    pnh_(pnh),
    device_manager_(AstraDeviceManager::getSingelton())
{
  ROS_INFO_STREAM(GetLogPrefix("AstraMultiDriver", ""));

  astra_register_sub_ = nh_.subscribe("/astra_registration", 1, &AstraMultiDriver::RegistrationCallback, this);
}

void AstraMultiDriver::RegistrationCallback(const multi_astra_camera::astra_registration_info::ConstPtr& msg)
{
  ROS_INFO_STREAM(GetLogPrefix("AstraMultiDriver", msg->ns) << "serial_no: " << msg->serial_no << ", is_advanced: " << (int)msg->is_advanced);

  std::string serial_no = msg->serial_no;
  serial_no.erase(0, 7); // Discard prefix "serial_"
  if (astras_.find(serial_no) == astras_.end())
  {
    astras_[serial_no] = boost::make_shared<AstraDriver>(nh_, pnh_, msg->ns, serial_no, msg->is_advanced);
  }
  else
  {
    ROS_INFO_STREAM(GetLogPrefix("AstraMultiDriver", msg->ns) << "had been registered, destory it frist, then create a new instance!");
    astras_[serial_no] = nullptr; // Must do this to make sure the desconstruct will be called before creating a new instance
    astras_[serial_no] = boost::make_shared<AstraDriver>(nh_, pnh_, msg->ns, serial_no, msg->is_advanced);
  }
}

}
