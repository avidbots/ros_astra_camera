#include "astra_camera/multi_astra_driver.h"
#include "astra_camera/astra_frame_reader.h"
#include <boost/make_shared.hpp>

namespace astra_wrapper
{

MultiAstraDriver::MultiAstraDriver(ros::NodeHandle& n, ros::NodeHandle& pnh) :
    nh_(n),
    pnh_(pnh),
    device_manager_(AstraDeviceManager::getSingelton())
{
  ROS_INFO_STREAM(GetLogPrefix("MultiAstraDriver", ""));

  openni::Status rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK)
    THROW_OPENNI_EXCEPTION("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());

  double sleep_time_before_read = 20.;
  double sleep_time_after_read = 62.;
  pnh_.param("sleep_time_before_read", sleep_time_before_read, sleep_time_before_read); 
  pnh_.param("sleep_time_after_read", sleep_time_after_read, sleep_time_after_read); 
  AstraFrameReader::getSingleton()->SetSleepTime(sleep_time_before_read, sleep_time_after_read);

  astra_register_sub_ = nh_.subscribe("/astra_registration", 10, &MultiAstraDriver::RegistrationCallback, this);
}

MultiAstraDriver::~MultiAstraDriver()
{
  astra_register_sub_.shutdown();
  openni::OpenNI::shutdown();
}

void MultiAstraDriver::RegistrationCallback(const astra_camera::astra_registration_info::ConstPtr& msg)
{
  ROS_INFO_STREAM(GetLogPrefix("MultiAstraDriver", msg->ns) << "serial_no: " << msg->serial_no << ", is_advanced: " << (int)msg->is_advanced);

  std::string serial_no = msg->serial_no;
  serial_no.erase(0, 7); // Discard prefix "serial_"
  if (astras_.find(serial_no) == astras_.end())
  {
    astras_[serial_no] = boost::make_shared<AstraAdvancedDriver>(nh_, pnh_, msg->ns, serial_no, msg->is_advanced);
  }
  else
  {
    ROS_INFO_STREAM(GetLogPrefix("MultiAstraDriver", msg->ns) << "had been registered, destory it frist, then create a new instance!");
    astras_[serial_no] = nullptr; // Must do this to make sure the desconstruct will be called before creating a new instance
    astras_[serial_no] = boost::make_shared<AstraAdvancedDriver>(nh_, pnh_, msg->ns, serial_no, msg->is_advanced);
  }
}

}
