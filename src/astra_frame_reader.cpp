#include "openni2/OpenNI.h"
#include <sys/ipc.h>
#include <sys/shm.h>

#include "astra_camera/astra_frame_reader.h"
#include "astra_camera/astra_timer_filter.h"
#include "astra_camera/astra_registration_info.h"

#include <sensor_msgs/image_encodings.h>

#include <ros/ros.h>

#define TIME_FILTER_LENGTH 15

namespace astra_wrapper
{

boost::shared_ptr<AstraFrameReader> AstraFrameReader::singleton_;

AstraFrameReader::AstraFrameReader() :
    user_device_timer_(false),
    reading_(true),
    projector_control_pause_(false),
    projector_control_paused_(false),
    non_projector_control_pause_(false),
    non_projector_control_paused_(false),
    sleep_time_before_read_(20),
    sleep_time_after_read_(60)
{
  ros::Time::init();
  ros::NodeHandle nh;

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
  {
    for (auto& context : projector_control_frame_contexts_)
    {
      context.second->depth_timer_filter->clear();
      context.second->color_timer_filter->clear();
    }
    for (auto& context : non_projector_control_frame_contexts_)
    {
      context.second->depth_timer_filter->clear();
      context.second->color_timer_filter->clear();
    }
  }
}

void AstraFrameReader::ReadFrames(std::map<std::string, boost::shared_ptr<FrameContext>>& contexts, const bool pause, bool& paused, const bool projector_control)
{
  if (pause)
  {
    usleep(100 * 1000);
    paused = true;
    return;
  }
  else paused = false;

  if (contexts.empty())
  {
    usleep(100 * 1000);
  }

  for (auto& context : contexts)
  {
    ReadRgbAndDepthFrame(*(context.second), contexts.size() == 2 ? projector_control : false, contexts.size()); // Only support 2 projector controllable cameras running at the same time
  }
}

void AstraFrameReader::ReadRgbAndDepthFrame(FrameContext& context, const bool projector_control, const int context_cnt)
{
  if (projector_control)
  {
    context.TurnOnProjector(false);
    usleep(sleep_time_before_read_ * 1000);
  }

  ReadFrame(context.ns, context.depth_video_stream, &context.depth_frame, context.depth_callback, *(context.depth_timer_filter), context.depth_prev_time_stamp);
  ReadFrame(context.ns, context.color_video_stream, &context.color_frame, context.color_callback, *(context.color_timer_filter), context.color_prev_time_stamp);

  if (projector_control)
  {
    usleep(sleep_time_after_read_ * 1000);
    context.TurnOnProjector(true);
  }
  else
    usleep((1000./(6.1*context_cnt)) * 1000);
}

void AstraFrameReader::ReadFrame(const std::string& ns, boost::shared_ptr<openni::VideoStream>& video_stream, openni::VideoFrameRef* video_frame, FrameCallbackFunction& callback, AstraTimerFilter& timer_filter, double& prev_time_stamp)
{
  if (!video_stream)
  {
    ROS_WARN_STREAM_THROTTLE(10, GetLogPrefix("AstraFrameReader", ns) << "video_stream is null!");
    return;
  }
  int pStreamIndex(0);
  auto p = video_stream.get();
  auto ret = openni::OpenNI::waitForAnyStream(&p, 1, &pStreamIndex, 2); // Must add this, since readFrame is a blocking method
  if (ret != openni::STATUS_OK)
  {
    ROS_WARN_STREAM_THROTTLE(10, GetLogPrefix("AstraFrameReader", ns) << "reading frame timeout!");
    return;
  }

  video_stream->readFrame(video_frame);

  if (video_frame->isValid() && callback)
  {
    sensor_msgs::ImagePtr image(new sensor_msgs::Image);

    ros::Time ros_now = ros::Time::now();

    if (!user_device_timer_)
    {
      image->header.stamp = ros_now;

      ROS_DEBUG_STREAM(GetLogPrefix("AstraFrameReader", ns) << "Time interval between frames: " << (float)((ros_now.toSec()-prev_time_stamp)*1000.0) << " ms");

      prev_time_stamp = ros_now.toSec();
    }
    else
    {
      uint64_t device_time = video_frame->getTimestamp();

      double device_time_in_sec = static_cast<double>(device_time)/1000000.0;
      double ros_time_in_sec = ros_now.toSec();

      double time_diff = ros_time_in_sec-device_time_in_sec;

      timer_filter.addSample(time_diff);

      double filtered_time_diff = timer_filter.getMedian();

      double corrected_timestamp = device_time_in_sec+filtered_time_diff;

      image->header.stamp.fromSec(corrected_timestamp);

      ROS_DEBUG_STREAM(GetLogPrefix("AstraFrameReader", ns) << "Time interval between frames: " << (float)((corrected_timestamp-prev_time_stamp)*1000.0) << " ms");

      prev_time_stamp = corrected_timestamp;
    }

    image->width = video_frame->getWidth();
    image->height = video_frame->getHeight();

    std::size_t data_size = video_frame->getDataSize();

    image->data.resize(data_size);
    memcpy(&image->data[0], video_frame->getData(), data_size);

    image->is_bigendian = 0;

    const openni::VideoMode& video_mode = video_frame->getVideoMode();
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
        ROS_ERROR_STREAM(GetLogPrefix("AstraFrameReader", ns) << "invalid image encoding!");
        break;
    }

    callback(image);
  }
}

void AstraFrameReader::Start()
{
  ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", ""));
  reading_ = true;
  projector_control_reading_thread_ = std::thread([this]() -> void { while (reading_ && ros::ok()) ReadFrames(projector_control_frame_contexts_, projector_control_pause_, projector_control_paused_, true); });
  non_projector_control_reading_thread_ = std::thread([this]() -> void { while (reading_ && ros::ok()) ReadFrames(non_projector_control_frame_contexts_, non_projector_control_pause_, non_projector_control_paused_, false); });
}

void AstraFrameReader::Stop() {
  ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", ""));
  reading_ = false;
  projector_control_reading_thread_.join();
  non_projector_control_reading_thread_.join();

  for (auto& context : projector_control_frame_contexts_)
  {
    context.second->TurnOnProjector(false);
  }
}

bool AstraFrameReader::IsRegistered(const std::string& uri, const bool projector_control)
{
  auto& contexts = projector_control ? projector_control_frame_contexts_ : non_projector_control_frame_contexts_;
  return contexts.find(uri) != contexts.end();
}

void AstraFrameReader::RegisterCamera(const std::string& uri, const std::string& ns, const std::string& serial_no, const bool projector_control)
{
  if (IsRegistered(uri, projector_control))
  {
    ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", ns) << "has been registered!");
    return;
  }
  else
  {
    boost::shared_ptr<FrameContext> context = boost::make_shared<FrameContext>();
    context->ns = ns;
    context->uri = uri;
    context->serial_no = serial_no;
    context->cob_device.InitDevice();                                                                                                                                                 
    context->cob_device.OpenDevice(uri.c_str());
    context->projector_control = projector_control;
    context->depth_timer_filter = boost::make_shared<AstraTimerFilter>(TIME_FILTER_LENGTH);
    context->color_timer_filter = boost::make_shared<AstraTimerFilter>(TIME_FILTER_LENGTH);
    context->depth_prev_time_stamp = 0.0;
    context->color_prev_time_stamp = 0.0;

    auto& contexts = projector_control ? projector_control_frame_contexts_ : non_projector_control_frame_contexts_;
    contexts[uri] = context;
    ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", ns) << "FINISHED, registered!" << ", projector_control: " << (int)projector_control);
  }
}

void AstraFrameReader::RegisterDepth(const std::string& uri, const std::string& ns, const std::string& serial_no, const boost::shared_ptr<openni::VideoStream>& depth_video_stream, const bool projector_control)
{
  RegisterStream(uri, ns, serial_no, depth_video_stream, projector_control);
}

void AstraFrameReader::RegisterColor(const std::string& uri, const std::string& ns, const std::string& serial_no, const boost::shared_ptr<openni::VideoStream>& color_video_stream, const bool projector_control)
{
  RegisterStream(uri, ns, serial_no, color_video_stream, projector_control);
}

void AstraFrameReader::RegisterStream(const std::string& uri, const std::string& ns, const std::string& serial_no, const boost::shared_ptr<openni::VideoStream>& video_stream, const bool projector_control)
{
  if (!video_stream)
  {
    ROS_WARN_STREAM(GetLogPrefix("AstraFrameReader", ns) << "video_stream is null!");
    return;
  }

  std::lock_guard<std::mutex> lock(projector_control ? projector_control_mutex_ : non_projector_control_mutex_);
  auto& pause = projector_control ? projector_control_pause_ : non_projector_control_pause_;
  auto& paused = projector_control ? projector_control_paused_ : non_projector_control_paused_;
  pause = true;
  while (!paused && ros::ok()) usleep(20 * 1000);
  RegisterCamera(uri, ns, serial_no, projector_control);
  auto& contexts = projector_control ? projector_control_frame_contexts_ : non_projector_control_frame_contexts_;
  if (video_stream->getSensorInfo().getSensorType() == openni::SENSOR_DEPTH)
    contexts[uri]->depth_video_stream = video_stream;
  else if (video_stream->getSensorInfo().getSensorType() == openni::SENSOR_COLOR)
    contexts[uri]->color_video_stream = video_stream;
  else
    ROS_ERROR_STREAM(GetLogPrefix("AstraFrameReader", ns) << "serial_no: " << serial_no << ", wrong video stream type! ");
  pause = false;
}

boost::shared_ptr<AstraFrameReader::FrameContext> AstraFrameReader::GetContext(const std::string& uri)
{
  auto context_iter = projector_control_frame_contexts_.find(uri);
  if (context_iter == projector_control_frame_contexts_.end())
  {
    context_iter = non_projector_control_frame_contexts_.find(uri);
    if (context_iter == non_projector_control_frame_contexts_.end())
    {
      ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", uri) << "doesn't exist!");
      return nullptr;
    }
  }
  return context_iter->second;
}

void AstraFrameReader::UnregisterStream(const std::string& uri, const bool projector_control, boost::shared_ptr<openni::VideoStream>& video_stream)
{
  std::lock_guard<std::mutex> lock(projector_control ? projector_control_mutex_ : non_projector_control_mutex_);
  auto& pause = projector_control ? projector_control_pause_ : non_projector_control_pause_;
  pause = true;
  auto& paused = projector_control ? projector_control_paused_ : non_projector_control_paused_;
  while (!paused && ros::ok()) usleep(20 * 1000);
  video_stream = nullptr;
  ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", uri) << "FINISHED, unregistered");
  pause = false;
}

void AstraFrameReader::UnregisterDepth(const std::string& uri)
{
  ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", uri) << "STARTED");

  auto context = GetContext(uri);
  if (!context)
  {
    ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", uri) << "was not registered!");
    return;
  }

  UnregisterStream(uri, context->projector_control, context->projector_control ? projector_control_frame_contexts_[uri]->depth_video_stream : non_projector_control_frame_contexts_[uri]->depth_video_stream);
}

void AstraFrameReader::UnregisterColor(const std::string& uri)
{
  ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", uri) << "STARTED");

  auto context = GetContext(uri);
  if (!context)
  {
    ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", uri) << "was not registered!");
    return;
  }

  UnregisterStream(uri, context->projector_control, context->projector_control ? projector_control_frame_contexts_[uri]->color_video_stream : non_projector_control_frame_contexts_[uri]->color_video_stream);
}

void AstraFrameReader::UnregisterCamera(const std::string& uri)
{
  ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", uri) << "STARTED");

  auto context = GetContext(uri);
  if (!context)
  {
    ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", uri) << "was not registered!");
    return;
  }

  if (context->projector_control)
  {
    std::lock_guard<std::mutex> lock(projector_control_mutex_);
    projector_control_pause_ = true;
    while (!projector_control_paused_ && ros::ok()) usleep(20 * 1000);

    projector_control_frame_contexts_[uri]->TurnOnProjector(false);
    projector_control_frame_contexts_[uri]->cob_device.CloseDevice();
    projector_control_frame_contexts_[uri] = nullptr; // release the source first
    projector_control_frame_contexts_.erase(uri);
      
    ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", uri) << "FINISHED, unregistered");

    projector_control_pause_ = false;
  }
  else
  {
    std::lock_guard<std::mutex> lock(non_projector_control_mutex_);
    non_projector_control_pause_ = true;
    while (!non_projector_control_paused_ && ros::ok()) usleep(20 * 1000);

    non_projector_control_frame_contexts_[uri]->TurnOnProjector(false);
    non_projector_control_frame_contexts_[uri]->cob_device.CloseDevice();
    non_projector_control_frame_contexts_[uri] = nullptr; // release the source first
    non_projector_control_frame_contexts_.erase(uri);
      
    ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", uri) << "FINISHED, unregistered");

    non_projector_control_pause_ = false;
  }
}

void AstraFrameReader::setDepthCallback(const std::string& uri, FrameCallbackFunction& depth_callback)
{
  auto context = GetContext(uri);
  if (!context)
  {
    ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", uri) << "was not registered!");
    return;
  }

  if (context->projector_control)
  {
    projector_control_frame_contexts_[uri]->depth_callback = depth_callback;
  }
  else
  {
    non_projector_control_frame_contexts_[uri]->depth_callback = depth_callback;
  }
}

void AstraFrameReader::setColorCallback(const std::string& uri, FrameCallbackFunction& color_callback)
{
  auto context = GetContext(uri);
  if (!context)
  {
    ROS_INFO_STREAM(GetLogPrefix("AstraFrameReader", uri) << "was not registered!");
    return;
  }

  if (context->projector_control)
  {
    projector_control_frame_contexts_[uri]->color_callback = color_callback;
  }
  else
  {
    non_projector_control_frame_contexts_[uri]->color_callback = color_callback;
  }
}

}
