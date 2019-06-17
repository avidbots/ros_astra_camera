#include "astra_camera/multi_astra_driver.h"
#include <nodelet/nodelet.h>

namespace astra_camera
{

class MultiAstraDriverNodelet : public nodelet::Nodelet
{
public:
  MultiAstraDriverNodelet()  {};

  ~MultiAstraDriverNodelet() {}

private:
  virtual void onInit()
  {
    ROS_INFO("MultiAstraDriverNodelet::onInit");
    lp.reset(new astra_wrapper::MultiAstraDriver(getNodeHandle(), getPrivateNodeHandle()));
  };

  boost::shared_ptr<astra_wrapper::MultiAstraDriver> lp;
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(astra_camera::MultiAstraDriverNodelet, nodelet::Nodelet)
