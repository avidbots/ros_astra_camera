#ifndef ASTRA_ADVANCED_DEVICE_H
#define ASTRA_ADVANCED_DEVICE_H

#include "astra_camera/astra_device.h"
#include "astra_camera/astra_frame_reader.h"

namespace astra_wrapper
{

class AstraAdvancedDevice : public AstraDevice
{
 public: 
  AstraAdvancedDevice(const std::string& device_URI, const std::string& ns, const std::string& serial_no, const bool projector_control) throw (AstraException);
  virtual ~AstraAdvancedDevice();

  void startDepthStream();
  void stopDepthStream();
  void startColorStream();
  void stopColorStream();
  void setDepthFrameCallback(FrameCallbackFunction callback);
  void setColorFrameCallback(FrameCallbackFunction callback);
  void setUseDeviceTimer(bool enable);

 protected:
  boost::shared_ptr<AstraFrameReader> frame_reader_;
  std::string uri_;
  std::string serial_no_;
  FrameCallbackFunction depth_callback_;
  FrameCallbackFunction color_callback_;
  bool projector_control_;
};

}

#endif
