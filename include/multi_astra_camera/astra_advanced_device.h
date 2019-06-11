#ifndef ASTRA_ADVANCED_DEVICE_H
#define ASTRA_ADVANCED_DEVICE_H

#include "multi_astra_camera/astra_device.h"
#include "multi_astra_camera/astra_frame_reader.h"

namespace astra_wrapper
{

class AstraAdvancedDevice : public AstraDevice
{
 public: 
  AstraAdvancedDevice(const std::string& device_URI, const std::string& ns, const std::string& serial_no) throw (AstraException);
  virtual ~AstraAdvancedDevice();

  void startDepthStream();
  void stopDepthStream();
  void setDepthFrameCallback(FrameCallbackFunction callback);
  void setUseDeviceTimer(bool enable);

 protected:
  boost::shared_ptr<AstraFrameReader> depth_frame_reader_;
  std::string uri_;
  std::string serial_no_;
  FrameCallbackFunction callback_;
};

}

#endif
