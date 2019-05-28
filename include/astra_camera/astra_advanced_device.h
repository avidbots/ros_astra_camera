#ifndef ASTRA_ADVANCED_DEVICE_H
#define ASTRA_ADVANCED_DEVICE_H

#include "astra_camera/astra_device.h"

namespace astra_wrapper
{

class AstraFrameReader;

class AstraAdvancedDevice : public AstraDevice
{
 public: 
  AstraAdvancedDevice(const std::string& device_URI) throw (AstraException);
  virtual ~AstraAdvancedDevice();

  void startDepthStream();
  void stopDepthStream();
  void setDepthFrameCallback(FrameCallbackFunction callback);
  void setUseDeviceTimer(bool enable);

 protected:
  boost::shared_ptr<AstraFrameReader> depth_frame_reader;
  std::string uri_;
  FrameCallbackFunction callback_;
};

}

#endif
