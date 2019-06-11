#include "openni2/OpenNI.h"

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/replace.hpp>

#include "multi_astra_camera/astra_driver.h"
#include "multi_astra_camera/astra_advanced_device.h"
#include "multi_astra_camera/astra_exception.h"
#include "multi_astra_camera/astra_convert.h"
#include "multi_astra_camera/astra_frame_listener.h"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <string>

namespace astra_wrapper
{

AstraAdvancedDevice::AstraAdvancedDevice(const std::string& device_URI, const std::string& ns, const std::string& serial_no) throw (AstraException) : 
  AstraDevice(device_URI, ns),
  uri_(device_URI),
  serial_no_(serial_no),
  callback_(0)
{
  ROS_INFO_STREAM(GetLogPrefix("AstraAdvancedDevice", ns) << "uri: " << uri_ << ", serial_no: " << serial_no);
  depth_frame_reader_ = AstraFrameReader::getSingleton();
}

AstraAdvancedDevice::~AstraAdvancedDevice()
{
  ROS_INFO_STREAM(GetLogPrefix("AstraAdvancedDevice", ns_) << "STARTED");

  stopAllStreams();

  shutdown();

  openni_device_->close();
  ROS_INFO_STREAM(GetLogPrefix("AstraAdvancedDevice", ns_) << "FINISHED");
}

void AstraAdvancedDevice::startDepthStream()
{
  ROS_INFO_STREAM(GetLogPrefix("AstraAdvancedDevice", ns_) << "STARTED");
  boost::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

  if (stream)
  {
    stream->setMirroringEnabled(false);
    stream->start();
    depth_frame_reader_->Register(uri_, ns_, serial_no_, stream);
    depth_frame_reader_->setCallback(uri_, callback_);
    depth_video_started_ = true;
  }
}

void AstraAdvancedDevice::stopDepthStream()
{
  ROS_INFO_STREAM(GetLogPrefix("AstraAdvancedDevice", ns_) << "STARTED");
  if (depth_video_stream_.get() != 0)
  {
    depth_video_started_ = false;
    depth_frame_reader_->Unregister(uri_);
    depth_video_stream_->stop();
  }
  ROS_INFO_STREAM(GetLogPrefix("AstraAdvancedDevice", ns_) << "FINISHED");
}

void AstraAdvancedDevice::setUseDeviceTimer(bool enable)
{
  if (ir_frame_listener)
    ir_frame_listener->setUseDeviceTimer(enable);

  if (color_frame_listener)
    color_frame_listener->setUseDeviceTimer(enable);

  if (depth_frame_reader_)
    depth_frame_reader_->setUseDeviceTimer(enable);
}

void AstraAdvancedDevice::setDepthFrameCallback(FrameCallbackFunction callback)
{
  callback_ = callback;
}

}

