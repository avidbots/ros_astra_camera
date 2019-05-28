#include "astra_camera/astra_multi_driver.h"
#include <boost/make_shared.hpp>

namespace astra_wrapper
{

AstraMultiDriver::AstraMultiDriver(ros::NodeHandle& n, ros::NodeHandle& pnh) :
    nh_(n),
    pnh_(pnh),
    device_manager_(AstraDeviceManager::getSingelton())
{
  ROS_INFO("AstraMultiDriver::AstraMultiDriver");

  astra_register_sub_ = nh_.subscribe("/astra_registration", 1, &AstraMultiDriver::RegistrationCallback, this);
}

void AstraMultiDriver::RegistrationCallback(const astra_camera::astra_registration_info::ConstPtr& msg)
{
  ROS_INFO("AstraMultiDriver::RegistrationCallback, ns: %s, serial_no: %s, is_advanced: %d", msg->ns.c_str(), msg->serial_no.c_str(), msg->is_advanced);

  std::string serial_no = msg->serial_no;
  serial_no.erase(0, 7); // Discard prefix "serial_"
  if (astras_.find(serial_no) == astras_.end())
  {
    astras_[serial_no] = boost::make_shared<AstraDriver>(nh_, pnh_, msg->ns, serial_no, msg->is_advanced);
  }
  else
  {
    ROS_INFO("AstraMultiDriver::RegistrationCallback, %s had been registered!", serial_no.c_str());
  }
}

}
