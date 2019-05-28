#include "openni2/OpenNI.h"
#include <sys/ipc.h>
#include <sys/shm.h>

#include "astra_camera/astra_frame_reader.h"
#include "astra_camera/astra_timer_filter.h"

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
    reading_(false)
{
  ros::Time::init();
  ros::NodeHandle nh;

  Start();
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

//void AstraFrameReader::ReadFrames()
//{
//  if (frame_contexts_.size() < 2)
//  {
//    usleep(1000);
//    return;
//  }
//
//  auto context1 = frame_contexts_.begin()->second;
//  auto context2 = (++ frame_contexts_.begin())->second;
//
//  //context2->TurnOnProjector(true);
//  //usleep(10*1000);
//  //context1->TurnOnProjector(false);
//  //usleep(10*1000);
//  //ReadOneFrame(*context1);
//  //usleep(70*1000);
//  //context2->TurnOnProjector(false);
//  //usleep(10*1000);
//  //ReadOneFrame(*context2);
//  //context1->TurnOnProjector(true);
//  //usleep(80*1000);
//}

void AstraFrameReader::ReadFrames()
{
  for (auto& context : frame_contexts_)
  {
    ReadOneFrame(context.first, *(context.second));
  }

  if (frame_contexts_.empty())
  {
    usleep(1000 * 1000);
  }
}

void AstraFrameReader::ReadOneFrame(FrameContext& context)
{
  context.video_stream->readFrame(&context.depth_frame);

  //ROS_INFO_THROTTLE(5, "AstraFrameReader::ReadOneFrame, frame.isValid: %d, callback_back: %d", context.depth_frame.isValid(), context.callback != 0);

  if (context.depth_frame.isValid() && context.callback)
  {
    sensor_msgs::ImagePtr image(new sensor_msgs::Image);

    ros::Time ros_now = ros::Time::now();

    if (!user_device_timer_)
    {
      image->header.stamp = ros_now;

      ROS_DEBUG("Time interval between frames: %.4f ms", (float)((ros_now.toSec()-prev_time_stamp_)*1000.0));

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
        ROS_ERROR("Invalid image encoding");
        break;
    }

    context.callback(image);
  }
}

void AstraFrameReader::ReadOneFrame(const std::string& uri, FrameContext& context)
{
  if (frame_contexts_.size() == 2) // Only support 2 advanced cameras running at the same time
  {
    //for (auto& context : frame_contexts_)
    //{
    //  if (context.first != uri) context.second->TurnOnProjector(true);
    //}
    context.TurnOnProjector(false);
    usleep(30 * 1000);
  }

  context.video_stream->readFrame(&context.depth_frame);

  //ROS_INFO_THROTTLE(5, "AstraFrameReader::ReadOneFrame, frame.isValid: %d, callback_back: %d", context.depth_frame.isValid(), context.callback != 0);

  if (context.depth_frame.isValid() && context.callback)
  {
    sensor_msgs::ImagePtr image(new sensor_msgs::Image);

    ros::Time ros_now = ros::Time::now();

    if (!user_device_timer_)
    {
      image->header.stamp = ros_now;

      ROS_DEBUG("Time interval between frames: %.4f ms", (float)((ros_now.toSec()-prev_time_stamp_)*1000.0));

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
        ROS_ERROR("Invalid image encoding");
        break;
    }

    context.callback(image);
  }

  if (frame_contexts_.size() == 2)
  {
    usleep(52 * 1000);
    //for (auto& context : frame_contexts_)
    //{
    //  if (context.first != uri) context.second->TurnOnProjector(false);
    //}
    context.TurnOnProjector(true);
  }
  else
    usleep(166 * 1000);

}

void AstraFrameReader::Start()
{
  ROS_INFO("AstraFrameReader::Start");
  reading_ = true;
  reading_thread_ = std::thread([this]() -> void { while (reading_ && ros::ok()) ReadFrames(); });
}

void AstraFrameReader::Stop() {
  ROS_INFO("AstraFrameReader::Stop");
  reading_ = false;
  reading_thread_.join();

  for (auto& context : frame_contexts_)
  {
    context.second->TurnOnProjector(false);
  }
}

void AstraFrameReader::Register(const std::string& uri, const boost::shared_ptr<openni::VideoStream>& video_stream)
{
  ROS_INFO("AstraFrameReader::Register, START, uri: %s!", uri.c_str());
  if (frame_contexts_.find(uri) != frame_contexts_.end())
  {
    ROS_INFO("AstraFrameReader::Register, %s has been registered!", uri.c_str());
  }
  else
  {
    boost::shared_ptr<FrameContext> context = boost::make_shared<FrameContext>();
    context->video_stream = video_stream;
    context->cob_device.InitDevice();                                                                                                                                                 
    context->cob_device.OpenDevice(uri.c_str());
    frame_contexts_[uri] = context;
  }
}

void AstraFrameReader::Unregister(const std::string& uri)
{
  if (frame_contexts_.find(uri) != frame_contexts_.end())
  {
    frame_contexts_.erase(uri);
  }
  else
  {
    ROS_INFO("AstraFrameReader::Register, %s isn't existed!", uri.c_str());
  }
}

}

