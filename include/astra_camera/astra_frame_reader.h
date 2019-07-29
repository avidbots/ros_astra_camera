#ifndef ASTRA_FRAME_READER_H_
#define ASTRA_FRAME_READER_H_ 

#include "astra_camera/astra_device.h"
#include "openni2/OpenNI.h"
#include "astra_camera/COBDevice.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <map>
#include <thread>
#include <mutex>
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
      {
        ROS_WARN_STREAM(ns << ": TurnOnProjector failed, ret: "<<ret<<", try to reset cob device!");
        //cob_device.CloseDevice();
        //auto ret = cob_device.InitDevice();
        //ROS_ERROR_STREAM(ns<<": InitDevice: ret: " << ret);
        //ret = cob_device.OpenDevice(uri.c_str());
        //ROS_ERROR_STREAM(ns<<": OpenDevice: ret: " << ret);
      }
    }
  };

  AstraFrameReader();

  virtual ~AstraFrameReader();

  static boost::shared_ptr<AstraFrameReader> getSingleton();

  void setDepthCallback(const std::string& uri, FrameCallbackFunction& depth_callback)
  {
    auto context_iter = projector_control_frame_contexts_.find(uri);
    bool projector_control = false;
    if (context_iter == projector_control_frame_contexts_.end())
    {
      context_iter = non_projector_control_frame_contexts_.find(uri);
      if (context_iter == non_projector_control_frame_contexts_.end())
      {
        ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", uri) << "doesn't exist!");
        return;
      }
      else
      {
        projector_control = false;
      }
    }
    else
      projector_control = true;

    if (projector_control)
    {
      projector_control_frame_contexts_[uri]->depth_callback = depth_callback;
    }
    else
    {
      non_projector_control_frame_contexts_[uri]->depth_callback = depth_callback;
    }
  }

  void setColorCallback(const std::string& uri, FrameCallbackFunction& color_callback)
  {
    auto context_iter = projector_control_frame_contexts_.find(uri);
    bool projector_control = false;
    if (context_iter == projector_control_frame_contexts_.end())
    {
      context_iter = non_projector_control_frame_contexts_.find(uri);
      if (context_iter == non_projector_control_frame_contexts_.end())
      {
        ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", uri) << "doesn't exist!");
        return;
      }
      else
      {
        projector_control = false;
      }
    }
    else
      projector_control = true;

    if (projector_control)
    {
      projector_control_frame_contexts_[uri]->color_callback = color_callback;
    }
    else
    {
      non_projector_control_frame_contexts_[uri]->color_callback = color_callback;
    }
  }

  void setUseDeviceTimer(bool enable);

  void Start();
  void Stop();

  void RegisterDepth(const std::string& uri, const std::string& ns, const std::string& serail_no, const boost::shared_ptr<openni::VideoStream>& depth_stream, const bool projector_contol);
  void UnregisterDepth(const std::string& uri);
  void RegisterColor(const std::string& uri, const std::string& ns, const std::string& serail_no, const boost::shared_ptr<openni::VideoStream>& color_stream, const bool projector_contol);
  void UnregisterColor(const std::string& uri);
  void UnregisterCamera(const std::string& uri);

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
  std::mutex mutex_;

  static boost::shared_ptr<AstraFrameReader> singleton_;

  void ReadFrames(std::map<std::string, boost::shared_ptr<FrameContext>>& context, const bool pause, bool& paused, const bool projector_control);
  void ReadRgbAndDepthFrame(const std::string& uri, FrameContext& context, const bool projector_control, const int context_cnt);
  void ReadFrame(const std::string& ns, openni::VideoStream& video_stream, openni::VideoFrameRef* video_frame, FrameCallbackFunction& callback, AstraTimerFilter& timer_filter, double& prev_time_stamp);
};

}

#endif
