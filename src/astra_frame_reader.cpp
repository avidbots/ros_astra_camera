#include "openni2/OpenNI.h"
#include <thread>
#include <sys/ipc.h>
#include <sys/shm.h>

#include "astra_camera/astra_frame_reader.h"
#include "astra_camera/astra_timer_filter.h"

#include <sensor_msgs/image_encodings.h>

#include <ros/ros.h>

#define TIME_FILTER_LENGTH 15

namespace astra_wrapper
{

AstraFrameReader::AstraFrameReader(const std::string& uri) :
    callback_(0),
    user_device_timer_(false),
    timer_filter_(new AstraTimerFilter(TIME_FILTER_LENGTH)),
    prev_time_stamp_(0.0),
    reading_(false),
    uri_(uri)
{
  ros::Time::init();
  ros::NodeHandle nh;

  cob_device_.InitDevice();                                                                                                                                                 
  cob_device_.OpenDevice(uri_.c_str());
}

void AstraFrameReader::setUseDeviceTimer(bool enable)
{
  user_device_timer_ = enable;

  if (user_device_timer_)
    timer_filter_->clear();
}

void AstraFrameReader::ReadFrame(const boost::shared_ptr<openni::VideoStream>& video_stream, openni::VideoFrameRef& frame)
{
  boost::interprocess::named_mutex read_mutex{boost::interprocess::open_or_create, "astra_frame_read_mutex"};
  read_mutex.lock();

  TurnOnProjector(false);
  usleep(10 * 1000);

  video_stream->readFrame(&frame);

  ROS_INFO_THROTTLE(5, "AstraFrameReader::ReadFrame, frame.isValid: %d, callback_back: %d", frame.isValid(), callback_ != 0);

  if (frame.isValid() && callback_)
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
      uint64_t device_time = frame.getTimestamp();

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

    image->width = frame.getWidth();
    image->height = frame.getHeight();

    std::size_t data_size = frame.getDataSize();

    image->data.resize(data_size);
    memcpy(&image->data[0], frame.getData(), data_size);

    image->is_bigendian = 0;

    const openni::VideoMode& video_mode = frame.getVideoMode();
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

    callback_(image);
  }

  usleep(80 * 1000);
  TurnOnProjector(true);

  read_mutex.unlock();
}

void AstraFrameReader::Start(const boost::shared_ptr<openni::VideoStream>& stream_ptr)
{
  ROS_INFO("AstraFrameReader::Start");
  reading_ = true;
  std::thread reading_thread([this, stream_ptr]() -> void { while (reading_ && ros::ok()) ReadFrame(stream_ptr, depth_frame_); });
  reading_thread.join();
}

void AstraFrameReader::Stop() {
  ROS_INFO("AstraFrameReader::Stop");
  reading_ = false;
}

void AstraFrameReader::TurnOnProjector(const bool turn_on)
{
  cob_device_.InitDevice();                                                                                                                                                 
  cob_device_.OpenDevice(uri_.c_str());

  uint16_t buf1(0), buf2(0);
  buf1 = turn_on ? 1 : 0;
  cob_device_.SendCmd(85, &buf1, 2, &buf2, 2);
}

}

