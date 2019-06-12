#include "openni2/OpenNI.h"
#include <sys/ipc.h>
#include <sys/shm.h>

#include "multi_astra_camera/astra_frame_reader.h"
#include "multi_astra_camera/astra_timer_filter.h"
#include "multi_astra_camera/astra_registration_info.h"

#include <sensor_msgs/image_encodings.h>

#include <ros/ros.h>

#define TIME_FILTER_LENGTH 15

namespace astra_wrapper
{

boost::shared_ptr<AstraFrameReader> AstraFrameReader::singleton_;

AstraFrameReader::AstraFrameReader() :
    user_device_timer_(false),
    timer_filter_(new AstraTimerFilter(TIME_FILTER_LENGTH)),
    prev_time_stamp_(0.0),
    reading_(true),
    pause_(false),
    paused_(false)
{
  ros::Time::init();
  ros::NodeHandle nh;

  reset_pub_ = nh.advertise<multi_astra_camera::astra_registration_info>("/astra_registration", 1);

  Start();
}

AstraFrameReader::~AstraFrameReader()
{
  Stop();
}

boost::shared_ptr<AstraFrameReader> AstraFrameReader::getSingleton()
{
  if (singleton_.get()==0)
    singleton_ = boost::make_shared<AstraFrameReader>();

  return singleton_;
}

void AstraFrameReader::setUseDeviceTimer(bool enable)
{
  user_device_timer_ = enable;

  if (user_device_timer_)
    timer_filter_->clear();
}

void AstraFrameReader::ReadFrames()
{
  if (pause_)
  {
    usleep(100 * 1000);
    paused_ = true;
    return;
  }
  else paused_ = false;

  if (frame_contexts_.empty())
  {
    usleep(100 * 1000);
  }

  for (auto& context : frame_contexts_)
  {
    ReadOneFrame(context.first, *(context.second));
  }
}

void AstraFrameReader::ReadOneFrame(const std::string& uri, FrameContext& context)
{
  int pStreamIndex(0);
  auto p = context.video_stream.get();
  auto ret = openni::OpenNI::waitForAnyStream(&p, 1, &pStreamIndex, 165); // Must add this, since readFrame is a blocking method
  if (ret != openni::STATUS_OK)
  {
    ROS_ERROR_STREAM_THROTTLE(5, GetLogPrefix("AstraFrameReader", context.ns) << "reading frame timeout!");
    return;
  }

  if (frame_contexts_.size() == 2) // Only support 2 advanced cameras running at the same time
  {
    context.TurnOnProjector(false);
    usleep(30 * 1000);
  }

  context.video_stream->readFrame(&context.depth_frame);

  if (context.depth_frame.isValid() && context.callback)
  {
    sensor_msgs::ImagePtr image(new sensor_msgs::Image);

    ros::Time ros_now = ros::Time::now();

    if (!user_device_timer_)
    {
      image->header.stamp = ros_now;

      ROS_DEBUG_STREAM(GetLogPrefix("AstraFrameReader", context.ns) << "Time interval between frames: " << (float)((ros_now.toSec()-prev_time_stamp_)*1000.0) << " ms");

      prev_time_stamp_ = ros_now.toSec();
    }
    else
    {
      uint64_t device_time = context.depth_frame.getTimestamp();

      double device_time_in_sec = static_cast<double>(device_time)/1000000.0;
      double ros_time_in_sec = ros_now.toSec();

      double time_diff = ros_time_in_sec-device_time_in_sec;

      timer_filter_->addSample(time_diff);

      double filtered_time_diff = timer_filter_->getMedian();

      double corrected_timestamp = device_time_in_sec+filtered_time_diff;

      image->header.stamp.fromSec(corrected_timestamp);

      ROS_DEBUG("Time interval between frames: %.4f ms", (float)((corrected_timestamp-prev_time_stamp_)*1000.0));
      ROS_DEBUG_STREAM(GetLogPrefix("AstraFrameReader", context.ns) << "Time interval between frames: " << (float)((corrected_timestamp-prev_time_stamp_)*1000.0) << " ms");

      prev_time_stamp_ = corrected_timestamp;
    }

    image->width = context.depth_frame.getWidth();
    image->height = context.depth_frame.getHeight();

    std::size_t data_size = context.depth_frame.getDataSize();

    image->data.resize(data_size);
    memcpy(&image->data[0], context.depth_frame.getData(), data_size);

    image->is_bigendian = 0;

    const openni::VideoMode& video_mode = context.depth_frame.getVideoMode();
    switch (video_mode.getPixelFormat())
    {
      case openni::PIXEL_FORMAT_DEPTH_1_MM:
        image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        image->step = sizeof(unsigned char) * 2 * image->width;
        break;
      case openni::PIXEL_FORMAT_DEPTH_100_UM:
        image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        image->step = sizeof(unsigned char) * 2 * image->width;
        break;
      case openni::PIXEL_FORMAT_SHIFT_9_2:
        image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        image->step = sizeof(unsigned char) * 2 * image->width;
        break;
      case openni::PIXEL_FORMAT_SHIFT_9_3:
        image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        image->step = sizeof(unsigned char) * 2 * image->width;
        break;

      case openni::PIXEL_FORMAT_RGB888:
        image->encoding = sensor_msgs::image_encodings::RGB8;
        image->step = sizeof(unsigned char) * 3 * image->width;
        break;
      case openni::PIXEL_FORMAT_YUV422:
        image->encoding = sensor_msgs::image_encodings::YUV422;
        image->step = sizeof(unsigned char) * 4 * image->width;
        break;
      case openni::PIXEL_FORMAT_GRAY8:
        image->encoding = sensor_msgs::image_encodings::MONO8;
        image->step = sizeof(unsigned char) * 1 * image->width;
        break;
      case openni::PIXEL_FORMAT_GRAY16:
        image->encoding = sensor_msgs::image_encodings::MONO16;
        image->step = sizeof(unsigned char) * 2 * image->width;
        break;
      case openni::PIXEL_FORMAT_JPEG:
      default:
        ROS_ERROR_STREAM(GetLogPrefix("AstraFrameReader", context.ns) << "invalid image encoding!");
        break;
    }

    context.callback(image);
  }

  if (frame_contexts_.size() == 2)
  {
    usleep(50 * 1000);
    context.TurnOnProjector(true);
  }
  else
    usleep(165 * 1000);

}

void AstraFrameReader::Start()
{
  ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", ""));
  reading_ = true;
  reading_thread_ = std::thread([this]() -> void { while (reading_ && ros::ok()) ReadFrames(); });
}

void AstraFrameReader::Stop() {
  ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", ""));
  reading_ = false;
  reading_thread_.join();

  for (auto& context : frame_contexts_)
  {
    context.second->TurnOnProjector(false);
  }
}

void AstraFrameReader::Register(const std::string& uri, const std::string& ns, const std::string& serial_no, const boost::shared_ptr<openni::VideoStream>& video_stream)
{
  ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", ns));
  mutex_.lock();
  pause_ = true;
  while (!paused_ && ros::ok()) usleep(20);

  if (frame_contexts_.find(uri) != frame_contexts_.end())
  {
    ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", ns) << "has been registered!");
  }
  else
  {
    boost::shared_ptr<FrameContext> context = boost::make_shared<FrameContext>();
    context->ns = ns;
    context->uri = uri;
    context->serial_no = serial_no;
    context->video_stream = video_stream;
    context->cob_device.InitDevice();                                                                                                                                                 
    context->cob_device.OpenDevice(uri.c_str());
    frame_contexts_[uri] = context;
    ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", ns) << "FINISHED, registered!");
  }
  pause_ = false;
  mutex_.unlock();
}

void AstraFrameReader::Unregister(const std::string& uri)
{
  ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", uri) << "STARTED");
  mutex_.lock();

  pause_ = true;
  while (!paused_ && ros::ok())
  {
    usleep(20);
  }

  if (frame_contexts_.find(uri) != frame_contexts_.end())
  {
    frame_contexts_[uri]->TurnOnProjector(false);
    frame_contexts_[uri]->cob_device.CloseDevice();
    frame_contexts_[uri] = nullptr; // release the source first
    frame_contexts_.erase(uri);
  }
    
  ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", uri) << "FINISHED, unregistered");

  pause_ = false;
  mutex_.unlock();
}

}
