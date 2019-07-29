#include "openni2/OpenNI.h"

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/replace.hpp>

#include "astra_camera/astra_driver.h"
#include "astra_camera/astra_advanced_device.h"
#include "astra_camera/astra_exception.h"
#include "astra_camera/astra_convert.h"
#include "astra_camera/astra_frame_listener.h"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <string>

namespace astra_wrapper
{

AstraAdvancedDevice::AstraAdvancedDevice(const std::string& device_URI, const std::string& ns, const std::string& serial_no, const bool projector_control) throw (AstraException) : 
  AstraDevice(device_URI, ns),
  uri_(device_URI),
  serial_no_(serial_no),
  depth_callback_(0),
  color_callback_(0),
  projector_control_(projector_control)
{
  ROS_INFO_STREAM(GetLogPrefix("AstraAdvancedDevice", ns) << "uri: " << uri_ << ", serial_no: " << serial_no);
  frame_reader_ = AstraFrameReader::getSingleton();
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
    frame_reader_->RegisterDepth(uri_, ns_, serial_no_, stream, projector_control_);
    frame_reader_->setDepthCallback(uri_, depth_callback_);
    depth_video_started_ = true;
  }
}

void AstraAdvancedDevice::startColorStream()
{
  ROS_INFO_STREAM(GetLogPrefix("AstraAdvancedDevice", ns_) << "STARTED");
  boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

  if (stream)
  {
    stream->setMirroringEnabled(false);
    stream->start();
    frame_reader_->RegisterColor(uri_, ns_, serial_no_, stream, projector_control_);
    frame_reader_->setColorCallback(uri_, color_callback_);
    color_video_started_ = true;
  }
}

void AstraAdvancedDevice::stopDepthStream()
{
  ROS_INFO_STREAM(GetLogPrefix("AstraAdvancedDevice", ns_) << "STARTED");
  if (depth_video_stream_.get() != 0)
  {
    depth_video_started_ = false;
    frame_reader_->UnregisterDepth(uri_);
    depth_video_stream_->stop();
  }
  ROS_INFO_STREAM(GetLogPrefix("AstraAdvancedDevice", ns_) << "FINISHED");
}

void AstraAdvancedDevice::stopColorStream()
{
  ROS_INFO_STREAM(GetLogPrefix("AstraAdvancedDevice", ns_) << "STARTED");
  if (color_video_stream_.get() != 0)
  {
    color_video_started_ = false;
    frame_reader_->UnregisterDepth(uri_);
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

  if (frame_reader_)
    frame_reader_->setUseDeviceTimer(enable);
}

void AstraAdvancedDevice::setDepthFrameCallback(FrameCallbackFunction callback)
{
  depth_callback_ = callback;
}

void AstraAdvancedDevice::setColorFrameCallback(FrameCallbackFunction callback)
{
  color_callback_ = callback;
}

}
