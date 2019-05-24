#include "openni2/OpenNI.h"

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/replace.hpp>

#include "astra_camera/astra_driver.h"
#include "astra_camera/astra_advanced_device.h"
#include "astra_camera/astra_exception.h"
#include "astra_camera/astra_convert.h"
#include "astra_camera/astra_frame_listener.h"
#include "astra_camera/astra_frame_reader.h"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <string>

namespace astra_wrapper
{

AstraAdvancedDevice::AstraAdvancedDevice(const std::string& device_URI) throw (AstraException) : 
  AstraDevice(device_URI)
{
  ROS_INFO("AstraAdvancedDevice::AstraAdvancedDevice, START construct!");
  depth_frame_reader = boost::make_shared<AstraFrameReader>(device_URI);
}

AstraAdvancedDevice::~AstraAdvancedDevice()
{
  stopAllStreams();

  shutdown();

  openni_device_->close();
}

void AstraAdvancedDevice::startDepthStream()
{
  ROS_INFO("AstraAdvancedDevice::startDepthStream, START");
  boost::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

  if (stream)
  {
    stream->setMirroringEnabled(false);
    stream->start();
    depth_frame_reader->Start(stream);
    depth_video_started_ = true;
  }
}

void AstraAdvancedDevice::stopDepthStream()
{
  ROS_INFO("AstraAdvancedDevice::stopDepthStream");
  if (depth_video_stream_.get() != 0)
  {
    depth_video_started_ = false;
    depth_frame_reader->Stop();
    depth_video_stream_->stop();
  }
}

void AstraAdvancedDevice::setUseDeviceTimer(bool enable)
{
  if (ir_frame_listener)
    ir_frame_listener->setUseDeviceTimer(enable);

  if (color_frame_listener)
    color_frame_listener->setUseDeviceTimer(enable);

  if (depth_frame_reader)
    depth_frame_reader->setUseDeviceTimer(enable);
}

void AstraAdvancedDevice::setDepthFrameCallback(FrameCallbackFunction callback)
{
  depth_frame_reader->setCallback(callback);
}

}

