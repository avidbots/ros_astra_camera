#ifndef ASTRA_FRAME_READER_H_
#define ASTRA_FRAME_READER_H_ 

#include "astra_camera/astra_device.h"
#include "openni2/OpenNI.h"
#include "astra_camera/COBDevice.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <map>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/make_shared.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

namespace astra_wrapper
{

namespace bi = boost::interprocess;

class AstraTimerFilter;

class AstraFrameReader
{
public:
  AstraFrameReader(const std::string& uri);

  virtual ~AstraFrameReader() {};

  void setCallback(FrameCallbackFunction& callback)
  {
    callback_ = callback;
  }

  void setUseDeviceTimer(bool enable);

  void Start(const boost::shared_ptr<openni::VideoStream>& stream_ptr);
  void Stop();

private:
  FrameCallbackFunction callback_;
  bool user_device_timer_;
  boost::shared_ptr<AstraTimerFilter> timer_filter_;
  double prev_time_stamp_;
  bool reading_;
  COBDevice cob_device_;
  std::string uri_;

  openni::VideoFrameRef depth_frame_;

  //ros::Timer timer_;
  //static boost::interprocess::named_mutex read_mutex_; //{bi::open_or_create, "astra_frame_read_mutex"};

  //static managed_shared_memory managed_shm_;
  bool* read_mutex_;
  bi::managed_shared_memory shm_;

  void ReadFrame(const boost::shared_ptr<openni::VideoStream>& video_stream, openni::VideoFrameRef& frame);
  void TurnOnProjector(const bool turn_on);
};

}

#endif
