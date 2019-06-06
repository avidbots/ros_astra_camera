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
    boost::shared_ptr<openni::VideoStream> video_stream;
    std::string ns;
    std::string serial_no;
    openni::VideoFrameRef depth_frame;
    COBDevice cob_device;
    FrameCallbackFunction callback;

    void TurnOnProjector(const bool turn_on)
    {
      uint16_t buf1(0), buf2(0);
      buf1 = turn_on ? 1 : 0;
      cob_device.SendCmd(85, &buf1, 2, &buf2, 2);
    }
  };

  AstraFrameReader();

  virtual ~AstraFrameReader();

  static boost::shared_ptr<AstraFrameReader> getSingleton();

  void setCallback(const std::string& uri, FrameCallbackFunction& callback)
  {
    if (frame_contexts_.find(uri) == frame_contexts_.end())
    {
      ROS_ERROR_STREAM(GetLogPrefix("AstraFrameReader", uri) << "hasn't been registered!");
      return;
    }
    frame_contexts_[uri]->callback = callback;
  }

  void setUseDeviceTimer(bool enable);

  void Start();
  void Stop();

  void Register(const std::string& uri, const std::string& ns, const std::string& serail_no, const boost::shared_ptr<openni::VideoStream>& video_stream);
  void Unregister(const std::string& uri);

private:
  bool user_device_timer_;
  boost::shared_ptr<AstraTimerFilter> timer_filter_;
  double prev_time_stamp_;
  bool reading_;
  bool pause_;
  bool paused_;
  std::map<std::string, boost::shared_ptr<FrameContext>> frame_contexts_;

  std::thread reading_thread_;
  std::mutex mutex_;
  ros::Publisher reset_pub_;

  static boost::shared_ptr<AstraFrameReader> singleton_;

  void ReadFrames();
  void ReadOneFrame(const std::string& uri, FrameContext& context);
  void ReadOneFrame(FrameContext& context);
};

}

#endif
