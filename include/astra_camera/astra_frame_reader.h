#ifndef ASTRA_FRAME_READER_H_
#define ASTRA_FRAME_READER_H_ 

#include "astra_camera/astra_device.h"
#include "openni2/OpenNI.h"
#include "astra_camera/COBDevice.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <map>
#include <mutex>
#include <thread>
#include <boost/make_shared.hpp>

namespace astra_wrapper
{

class AstraTimerFilter;

class AstraFrameReader
{
public:
  struct FrameContext
  {
    boost::shared_ptr<openni::VideoStream> depth_video_stream;
    boost::shared_ptr<openni::VideoStream> color_video_stream;
    std::string uri;
    std::string ns;
    std::string serial_no;
    openni::VideoFrameRef depth_frame;
    openni::VideoFrameRef color_frame;
    COBDevice cob_device;
    FrameCallbackFunction depth_callback;
    FrameCallbackFunction color_callback;
    bool projector_control;

    void TurnOnProjector(const bool turn_on)
    {
      uint16_t buf1(0), buf2(0);
      buf1 = turn_on ? 1 : 0;
      auto ret = cob_device.SendCmd(85, &buf1, 2, &buf2, 2);
      if (ret < 0)
        ROS_WARN_STREAM_THROTTLE(10, GetLogPrefix("AstraFrameReader", ns) << "send_cmd: Output control transfer failed, return: " << ret);
    }
  };

  AstraFrameReader();
  virtual ~AstraFrameReader();
  static boost::shared_ptr<AstraFrameReader> getSingleton();

  // Setters
  void setDepthCallback(const std::string& uri, FrameCallbackFunction& depth_callback);
  void setColorCallback(const std::string& uri, FrameCallbackFunction& color_callback);
  void setUseDeviceTimer(bool enable);
  void SetSleepTime(const double sleep_time_before_read, const double sleep_time_after_read)
  {
    sleep_time_before_read_ = sleep_time_before_read;
    sleep_time_after_read_ = sleep_time_after_read;
  }

  void Start();
  void Stop();

  void RegisterCamera(const std::string& uri, const std::string& ns, const std::string& serial_no, const bool projector_contol);
  void RegisterDepth(const std::string& uri, const std::string& ns, const std::string& serial_no, const boost::shared_ptr<openni::VideoStream>& depth_video_stream, const bool projector_control);
  void RegisterColor(const std::string& uri, const std::string& ns, const std::string& serial_no, const boost::shared_ptr<openni::VideoStream>& color_video_stream, const bool projector_control);
  void UnregisterCamera(const std::string& uri);
  void UnregisterDepth(const std::string& uri);
  void UnregisterColor(const std::string& uri);
  bool IsRegistered(const std::string& uri, const bool projector_control);

private:
  bool user_device_timer_;
  boost::shared_ptr<AstraTimerFilter> depth_timer_filter_;
  double depth_prev_time_stamp_;
  boost::shared_ptr<AstraTimerFilter> color_timer_filter_;
  double color_prev_time_stamp_;
  bool reading_;
  bool projector_control_pause_;
  bool projector_control_paused_;
  bool non_projector_control_pause_;
  bool non_projector_control_paused_;
  std::map<std::string, boost::shared_ptr<FrameContext>> projector_control_frame_contexts_;
  std::map<std::string, boost::shared_ptr<FrameContext>> non_projector_control_frame_contexts_;
  std::thread projector_control_reading_thread_;
  std::thread non_projector_control_reading_thread_;
  std::mutex projector_control_mutex_;
  std::mutex non_projector_control_mutex_;

  static boost::shared_ptr<AstraFrameReader> singleton_;

  double sleep_time_before_read_;
  double sleep_time_after_read_;

  void ReadFrames(std::map<std::string, boost::shared_ptr<FrameContext>>& context, const bool pause, bool& paused, const bool projector_control);
  void ReadRgbAndDepthFrame(FrameContext& context, const bool projector_control, const int context_cnt);
  void ReadFrame(const std::string& ns, boost::shared_ptr<openni::VideoStream>& video_stream, openni::VideoFrameRef* video_frame, FrameCallbackFunction& callback, AstraTimerFilter& timer_filter, double& prev_time_stamp);

  void RegisterStream(const std::string& uri, const std::string& ns, const std::string& serial_no, const boost::shared_ptr<openni::VideoStream>& video_stream, const bool projector_control);
  void UnregisterStream(const std::string& uri, const bool projector_control, boost::shared_ptr<openni::VideoStream>& video_stream);

  boost::shared_ptr<FrameContext> GetContext(const std::string& uri);
};

}

#endif
