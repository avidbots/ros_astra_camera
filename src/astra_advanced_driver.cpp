#include "astra_camera/astra_advanced_driver.h"
#include "astra_camera/astra_exception.h"
#include "astra_camera/astra_registration_info.h"

#include <openni2/OpenNI.h>

#include <unistd.h>  
#include <stdlib.h>  
#include <stdio.h>  
#include <mutex>
#include <sys/shm.h>  
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

namespace astra_wrapper
{

AstraAdvancedDriver::AstraAdvancedDriver(const ros::NodeHandle& n, const ros::NodeHandle& pnh, const std::string& ns, const std::string& serial_no, const bool projector_control) :
    nh_(n),
    pnh_(pnh),
    device_manager_(AstraDeviceManager::getSingelton()),
    device_(nullptr),
    device_id_(serial_no),
    ns_(ns),
    config_init_(false),
    ir_subscribers_(false),
    color_subscribers_(false),
    depth_subscribers_(false),
    depth_raw_subscribers_(false),
    enable_streaming_(false),
    projector_control_(projector_control),
    stop_init_(false)
{
  if (ns_.empty()) pnh_.getParam("ns", ns_);
  genVideoModeTableMap();
  
  // Create service for enable/disable streaming, keep this service out of Init(), to receive request to reset the camera
  enable_streaming_srv_ = nh_.advertiseService("/" + ns_ + "/driver/enable_streams", &AstraAdvancedDriver::EnableStreamsSrvCallback, this);

  Init();
}

void AstraAdvancedDriver::Init()
{
  ROS_INFO_STREAM(GetLogPrefix("Init", ns_));

  readConfigFromParameterServer(); // The params have to be read first before initDevice

  initDevice();
  if (!IsInit()) return; // Must return if init failed
  
  // Initialize dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(ros::NodeHandle("/" + ns_)));
  reconfigure_server_->setCallback(boost::bind(&AstraAdvancedDriver::configCb, this, _1, _2));

  while (!config_init_ && !stop_init_ && ros::ok())
  {
    ROS_INFO_STREAM(GetLogPrefix("Init", ns_) << "Waiting for dynamic reconfigure configuration.");
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }

  setHealthTimers();
  advertiseROSTopics();

  device_->setDepthFrameCallback(boost::bind(&AstraAdvancedDriver::newDepthFrameCallback, this, _1));
  device_->setColorFrameCallback(boost::bind(&AstraAdvancedDriver::newColorFrameCallback, this, _1));
  device_->setIRFrameCallback(boost::bind(&AstraAdvancedDriver::newIRFrameCallback, this, _1));

  EnableStreaming(true); // This Func will make sure color stream being started before depth stream started
}

void AstraAdvancedDriver::Destroy()
{
  ROS_INFO_STREAM(GetLogPrefix("Destroy", ns_));

  // Stop reset thread
  stop_init_ = true;
  ros::Duration(0.5).sleep();
  if (reset_thread_.joinable())
    reset_thread_.join();

  // Release deivce
  if (device_)
  {
    if (device_) device_ = nullptr; // Release device
    usleep(10*1000);
  }

  // Shut down ros related objects
  depth_callback_timer_.stop();
  pub_color_.shutdown();
  pub_depth_.shutdown();
  pub_depth_raw_.shutdown();
  pub_ir_.shutdown();
  pub_projector_info_.shutdown();
  usleep(10*1000);

  // Reset variables
  config_init_ = false;
  ir_subscribers_ = false;
  color_subscribers_ = false;
  depth_subscribers_ = false;
  depth_raw_subscribers_ = false;
  enable_streaming_ = false;
}

AstraAdvancedDriver::~AstraAdvancedDriver()
{
  ROS_INFO_STREAM(GetLogPrefix("AstraAdvancedDriver", ns_));
  Destroy();
}

bool AstraAdvancedDriver::EnableStreamsSrvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  ROS_INFO_STREAM(GetLogPrefix("AstraAdvancedDriver", ns_) << "START, req.data: "<< (int)req.data << ", enable_streaming_: " << enable_streaming_ <<  ", rgb_preferred_: " << rgb_preferred_);

  EnableStreaming(req.data);
  res.success = true;
  if (req.data != enable_streaming_)
  {
    res.message = req.data ? "Enabling streaming failed" : "Disabling streaming failed";
  }
  else
  {
    res.message = req.data ? "Enabling streaming succeed" : "Disabling streaming succeed";
  }

  ROS_INFO_STREAM(GetLogPrefix("EnableStreamSrvCallback", ns_) << "FINISHED, req.data: " << (int)req.data << ", enable_streaming_: " << enable_streaming_);
  return true;
}

void AstraAdvancedDriver::EnableStreaming(const bool& enable)
{
  ROS_INFO_STREAM(GetLogPrefix("EnableStreaming", ns_) << "START, enable: "<< (int)enable << ", enable_streaming_: " << enable_streaming_);
  if (!IsInit())
  {
    ROS_ERROR_STREAM(GetLogPrefix("EnableStreaming", ns_) << "device hasn't been initialized");
    return;
  }
  if (!enable && enable_streaming_) // Disable streaming
  {
    enable_streaming_ = false;
    // Must stop depth first
    if (device_ && device_->isDepthStreamStarted())
    {
      device_->stopDepthStream();
      depth_callback_timer_.stop();
    }
    if (device_ && device_->isIRStreamStarted()) device_->stopIRStream();
    if (device_ && device_->isColorStreamStarted()) device_->stopColorStream();
  }

  if (enable && !enable_streaming_) // Enable streaming
  {
    // Must start color first
    if (!rgb_preferred_)
    {
      if (device_ && device_->isColorStreamStarted()) device_->stopColorStream();
      if (device_ && !device_->isIRStreamStarted()) device_->startIRStream();
    }
    else
    {
      if (device_ && device_->isIRStreamStarted()) device_->stopIRStream();
      if (device_ && !device_->isColorStreamStarted()) device_->startColorStream();
    }

    if (device_ && !device_->isDepthStreamStarted())  // Must do depth streaming after color streams, otherwise send_cmd error could be happening when reading depth frames
    {
      device_->startDepthStream();
      depth_callback_timer_.start();
    }

    enable_streaming_ = true;
  }
}

void AstraAdvancedDriver::ResetThis()
{
  ROS_WARN_STREAM(GetLogPrefix("ResetThis", ns_));
  Destroy();
  stop_init_ = false;
  reset_thread_ = std::thread([this]() -> void {
    while (ros::ok() && !device_ && !stop_init_)
    {
      ROS_ERROR_STREAM(GetLogPrefix("ResetThis", ns_) << "failed to initialize camera");
      ros::Duration(5).sleep();
      Init();
    }
    ROS_INFO_STREAM(GetLogPrefix("ResetThis", ns_) << "reset succeed!");
    });
}

void AstraAdvancedDriver::setHealthTimers() {
  auto reset_this = [this](const ros::TimerEvent&) -> void
  {
    ROS_WARN_STREAM(GetLogPrefix("setHealthTimers", ns_) << " driver timeout! Resetting");
    ResetThis();
  };
  depth_callback_timer_ = nh_.createTimer(depth_callback_timeout_, reset_this, false, false);
}

void AstraAdvancedDriver::advertiseROSTopics()
{
  if (!IsInit())
  {
    ROS_ERROR_STREAM(GetLogPrefix("advertiseROSTopics", ns_) << "device hasn't been initialized");
    return;
  }

  // Allow remapping namespaces rgb, ir, depth, depth_registered
  ros::NodeHandle color_nh(nh_, "/" + ns_ + "/rgb");
  image_transport::ImageTransport color_it(color_nh);
  ros::NodeHandle ir_nh(nh_, "/" + ns_ + "/ir");
  image_transport::ImageTransport ir_it(ir_nh);
  ros::NodeHandle depth_nh(nh_, "/" + ns_ + "/depth");
  image_transport::ImageTransport depth_it(depth_nh);
  ros::NodeHandle depth_raw_nh(nh_, "/" + ns_ + "/depth");
  image_transport::ImageTransport depth_raw_it(depth_raw_nh);
  ros::NodeHandle projector_nh(nh_, "/" + ns_ + "/projector");
  // Advertise all published topics

  // Prevent connection callbacks from executing until we've set all the publishers. Otherwise
  // connectCb() can fire while we're advertising (say) "depth/image_raw", but before we actually
  // assign to pub_depth_raw_. Then pub_depth_raw_.getNumSubscribers() returns 0, and we fail to start
  // the depth generator.
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  // Asus Xtion PRO does not have an RGB camera
  //ROS_WARN("-------------has color sensor is %d----------- ", device_->hasColorSensor());
  if (device_->hasColorSensor())
  {
    image_transport::SubscriberStatusCallback itssc = boost::bind(&AstraAdvancedDriver::imageConnectCb, this);
    ros::SubscriberStatusCallback rssc = boost::bind(&AstraAdvancedDriver::imageConnectCb, this);
    pub_color_ = color_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
  }

  if (device_->hasIRSensor())
  {
    image_transport::SubscriberStatusCallback itssc = boost::bind(&AstraAdvancedDriver::imageConnectCb, this);
    ros::SubscriberStatusCallback rssc = boost::bind(&AstraAdvancedDriver::imageConnectCb, this);
    pub_ir_ = ir_it.advertiseCamera("image", 1, itssc, itssc, rssc, rssc);
  }

  if (device_->hasDepthSensor())
  {
    image_transport::SubscriberStatusCallback itssc = boost::bind(&AstraAdvancedDriver::depthConnectCb, this);
    ros::SubscriberStatusCallback rssc = boost::bind(&AstraAdvancedDriver::depthConnectCb, this);
    pub_depth_raw_ = depth_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
    pub_depth_ = depth_raw_it.advertiseCamera("image", 1, itssc, itssc, rssc, rssc);
    pub_projector_info_ = projector_nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1, rssc, rssc);
  }

  ////////// CAMERA INFO MANAGER

  // Pixel offset between depth and IR images.
  // By default assume offset of (5,4) from 9x7 correlation window.
  // NOTE: These are now (temporarily?) dynamically reconfigurable, to allow tweaking.
  //param_nh.param("depth_ir_offset_x", depth_ir_offset_x_, 5.0);
  //param_nh.param("depth_ir_offset_y", depth_ir_offset_y_, 4.0);

  // The camera names are set to [rgb|depth]_[serial#], e.g. depth_B00367707227042B.
  // camera_info_manager substitutes this for ${NAME} in the URL.
  std::string serial_number = device_->getStringID();
  std::string color_name, ir_name;

  color_name = "rgb_"   + serial_number;
  ir_name  = "depth_" + serial_number;

  // Load the saved calibrations, if they exist
  color_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(color_nh, color_name, color_info_url_);
  ir_info_manager_  = boost::make_shared<camera_info_manager::CameraInfoManager>(ir_nh,  ir_name,  ir_info_url_);

  get_serial_server = nh_.advertiseService("/" + ns_ + "/get_serial", &AstraAdvancedDriver::getSerialCb,this);

}

bool AstraAdvancedDriver::getSerialCb(astra_camera::GetSerialRequest& req, astra_camera::GetSerialResponse& res) {
  if (!IsInit())
  {
    ROS_ERROR_STREAM(GetLogPrefix("getSerialCb", ns_) << "device hasn't been initialized");
    return false;
  }

  res.serial = device_manager_->getSerial(device_->getUri());
  return true;
}

void AstraAdvancedDriver::configCb(Config &config, uint32_t level)
{
  ROS_INFO_STREAM(GetLogPrefix("configCb", ns_));

  rgb_preferred_ = config.rgb_preferred;

  if (config_init_ && old_config_.rgb_preferred != config.rgb_preferred)
    imageConnectCb();

  depth_ir_offset_x_ = config.depth_ir_offset_x;
  depth_ir_offset_y_ = config.depth_ir_offset_y;
  z_offset_mm_ = config.z_offset_mm;
  z_scaling_ = config.z_scaling;

  ir_time_offset_ = ros::Duration(config.ir_time_offset);
  color_time_offset_ = ros::Duration(config.color_time_offset);
  depth_time_offset_ = ros::Duration(config.depth_time_offset);

  ROS_INFO_STREAM(GetLogPrefix("configCb", ns_) << "color_mode: " << config.color_mode);
  if (lookupVideoModeFromDynConfig(config.ir_mode, ir_video_mode_)<0)
  {
    ROS_ERROR_STREAM(GetLogPrefix("configCb", ns_) << "Undefined IR video mode received from dynamic reconfigure");
    exit(-1);
  }

  if (lookupVideoModeFromDynConfig(config.color_mode, color_video_mode_)<0)
  {
    ROS_ERROR_STREAM(GetLogPrefix("configCb", ns_) << "Undefined color video mode received from dynamic reconfigure");
    exit(-1);
  }

  if (lookupVideoModeFromDynConfig(config.depth_mode, depth_video_mode_)<0)
  {
    ROS_ERROR_STREAM(GetLogPrefix("configCb", ns_) << "Undefined depth video mode received from dynamic reconfigure");
    exit(-1);
  }
  ROS_INFO_STREAM(GetLogPrefix("configCb", ns_) << "depth_video_mode_.frame_rate_: " << depth_video_mode_.frame_rate_ << ", depth_video_mode_.pixel_format_: "
      << (int)depth_video_mode_.pixel_format_ << ", depth_video_mode_.x_resolution_: " << depth_video_mode_.x_resolution_ << ", depth_video_mode_.y_resolution_: "
      << depth_video_mode_.y_resolution_);

  // assign pixel format

  ir_video_mode_.pixel_format_ = PIXEL_FORMAT_GRAY16;
  color_video_mode_.pixel_format_ = PIXEL_FORMAT_RGB888;
  depth_video_mode_.pixel_format_ = PIXEL_FORMAT_DEPTH_1_MM;

  color_depth_synchronization_ = config.color_depth_synchronization;
  depth_registration_ = config.depth_registration;

  auto_exposure_ = config.auto_exposure;
  auto_white_balance_ = config.auto_white_balance;

  use_device_time_ = config.use_device_time;

  ir_data_skip_ = config.ir_data_skip+1;
  color_data_skip_ = config.color_data_skip+1;
  depth_data_skip_ = config.depth_data_skip+1;

  applyConfigToOpenNIDevice();

  config_init_ = true;

  old_config_ = config;
}

void AstraAdvancedDriver::setIRVideoMode(const AstraVideoMode& ir_video_mode)
{
  if (!IsInit())
  {
    ROS_ERROR_STREAM(GetLogPrefix("setIRVideoMode", ns_) << "device hasn't been initialized");
    return;
  }

  if (device_->isIRVideoModeSupported(ir_video_mode))
  {
    if (ir_video_mode != device_->getIRVideoMode())
    {
      device_->setIRVideoMode(ir_video_mode);
    }

  }
  else
  {
    ROS_ERROR_STREAM(GetLogPrefix("setIRVideoMode", ns_) << "Unsupported IR video mode - " << ir_video_mode);
  }
}
void AstraAdvancedDriver::setColorVideoMode(const AstraVideoMode& color_video_mode)
{
  if (!IsInit())
  {
    ROS_ERROR_STREAM(GetLogPrefix("setColorVideoMode", ns_) << "device hasn't been initialized");
    return;
  }

  if (device_->isColorVideoModeSupported(color_video_mode))
  {
    if (color_video_mode != device_->getColorVideoMode())
    {
      device_->setColorVideoMode(color_video_mode);
    }
  }
  else
  {
    ROS_ERROR_STREAM(GetLogPrefix("setColorVideoMode", ns_) << "Unsupported color video mode - " << color_video_mode);
  }
}
void AstraAdvancedDriver::setDepthVideoMode(const AstraVideoMode& depth_video_mode)
{
  if (!IsInit())
  {
    ROS_ERROR_STREAM(GetLogPrefix("setDepthVideoMode", ns_) << "device hasn't been initialized");
    return;
  }

  if (device_->isDepthVideoModeSupported(depth_video_mode))
  {
    if (depth_video_mode != device_->getDepthVideoMode())
    {
      device_->setDepthVideoMode(depth_video_mode);
    }
  }
  else
  {
    ROS_ERROR_STREAM(GetLogPrefix("setDepthVideoMode", ns_) << "Unsupported depth video mode - " << depth_video_mode);
  }
}

void AstraAdvancedDriver::applyConfigToOpenNIDevice()
{
  if (!IsInit())
  {
    ROS_ERROR_STREAM(GetLogPrefix("applyConfigToOpenNIdevice", ns_) << "device hasn't been initialized");
    return;
  }

  device_->setIRDataSkip(ir_data_skip_);
  device_->setColorDataSkip(color_data_skip_);
  device_->setDepthDataSkip(depth_data_skip_);

  setIRVideoMode(ir_video_mode_);
  if (device_->hasColorSensor())
  {
  	setColorVideoMode(color_video_mode_);
  }
  setDepthVideoMode(depth_video_mode_);

  if (device_->isImageRegistrationModeSupported())
  {
    try
    {
      if (!config_init_ || (old_config_.depth_registration != depth_registration_))
        device_->setImageRegistrationMode(depth_registration_);
    }
    catch (const AstraException& exception)
    {
      ROS_ERROR("Could not set image registration. Reason: %s", exception.what());
    }
  }

  try
  {
    if (!config_init_ || (old_config_.color_depth_synchronization != color_depth_synchronization_))
      device_->setDepthColorSync(color_depth_synchronization_);
  }
  catch (const AstraException& exception)
  {
    ROS_ERROR("Could not set color depth synchronization. Reason: %s", exception.what());
  }

  try
  {
    if (!config_init_ || (old_config_.auto_exposure != auto_exposure_))
      device_->setAutoExposure(auto_exposure_);
  }
  catch (const AstraException& exception)
  {
    ROS_ERROR("Could not set auto exposure. Reason: %s", exception.what());
  }

  try
  {
    if (!config_init_ || (old_config_.auto_white_balance != auto_white_balance_))
      device_->setAutoWhiteBalance(auto_white_balance_);
  }
  catch (const AstraException& exception)
  {
    ROS_ERROR("Could not set auto white balance. Reason: %s", exception.what());
  }

  device_->setUseDeviceTimer(use_device_time_);
}

// imageConnectCb won't be able to start/stop color stream because of usb communication issue
void AstraAdvancedDriver::imageConnectCb()
{
  if (!IsInit())
  {
    ROS_ERROR_STREAM(GetLogPrefix("imageConnectCb", ns_) << "device hasn't been initialized");
    return;
  }

  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  bool ir_started = device_->isIRStreamStarted();
  bool color_started = device_->isColorStreamStarted();

  ir_subscribers_ = pub_ir_.getNumSubscribers() > 0;
  color_subscribers_ = pub_color_.getNumSubscribers() > 0;

  if (color_subscribers_ && (!ir_subscribers_ || rgb_preferred_))
  {
    if (ir_subscribers_)
      ROS_ERROR("Cannot stream RGB and IR at the same time. Streaming RGB only.");

    if (ir_started)
    {
      ROS_INFO("Stopping IR stream.");
      device_->stopIRStream();
    }

    if (!color_started)
    {
      if (enable_streaming_) {
        ROS_WARN_STREAM(GetLogPrefix("imageConnectCb", ns_) << "Color streaming should've been started, color streaming only can be controlled by construct and EnableStreaming()");
        //device_->startColorStream();
      }
    }
  }
  else if (ir_subscribers_ && (!color_subscribers_ || !rgb_preferred_))
  {

    if (color_subscribers_)
      ROS_ERROR("Cannot stream RGB and IR at the same time. Streaming IR only.");

    if (color_started)
    {
      // color streaming only can be controlled by EnableStreaming()
      //device_->stopColorStream();
    }

    if (!ir_started)
    {
      if (enable_streaming_) {
        ROS_INFO("Starting IR stream.");
        device_->startIRStream(); // TODO: IR streaming could have the same usb port communication issue with rgb streaming, since it isn't used for now, we live with it
      }
    }
  }
  else
  {
    if (color_started)
    {
      // color streaming only can be controlled by construct and EnableStreaming()
      //device_->stopColorStream();
    }
    if (ir_started)
    {
      ROS_INFO("Stopping IR stream.");
      device_->stopIRStream();
    }
  }
}

void AstraAdvancedDriver::depthConnectCb()
{
  if (!IsInit())
  {
    ROS_ERROR_STREAM(GetLogPrefix("depthConnectCb", ns_) << "device hasn't been initialized");
    return;
  }

  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  depth_subscribers_ = pub_depth_.getNumSubscribers() > 0;
  depth_raw_subscribers_ = pub_depth_raw_.getNumSubscribers() > 0;
  projector_info_subscribers_ = pub_projector_info_.getNumSubscribers() > 0;

  bool need_depth = depth_subscribers_ || depth_raw_subscribers_;

  if (need_depth && !device_->isDepthStreamStarted())
  {
    if (enable_streaming_) {
      ROS_INFO("Starting depth stream.");
      device_->startDepthStream();
      depth_callback_timer_.start();
    }
  }
  else if (!need_depth && device_->isDepthStreamStarted())
  {
    ROS_INFO("Stopping depth stream.");
    device_->stopDepthStream();
  }
}

void AstraAdvancedDriver::newIRFrameCallback(sensor_msgs::ImagePtr image)
{
  if (ir_subscribers_)
  {
    image->header.frame_id = ir_frame_id_;
    image->header.stamp = image->header.stamp + ir_time_offset_;

    pub_ir_.publish(image, getIRCameraInfo(image->width, image->height, image->header.stamp));
  }
}

void AstraAdvancedDriver::newColorFrameCallback(sensor_msgs::ImagePtr image)
{
  if (color_subscribers_)
  {
    image->header.frame_id = color_frame_id_;
    image->header.stamp = image->header.stamp + color_time_offset_;

    pub_color_.publish(image, getColorCameraInfo(image->width, image->height, image->header.stamp));
  }
}

void AstraAdvancedDriver::newDepthFrameCallback(sensor_msgs::ImagePtr image)
{
  depth_callback_timer_.setPeriod(depth_callback_timeout_, true);
  if (depth_raw_subscribers_||depth_subscribers_||projector_info_subscribers_)
  {
    image->header.stamp = image->header.stamp + depth_time_offset_;

    if (z_offset_mm_ != 0)
    {
      uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
      for (unsigned int i = 0; i < image->width * image->height; ++i)
        if (data[i] != 0)
              data[i] += z_offset_mm_;
    }

    if (fabs(z_scaling_ - 1.0) > 1e-6)
    {
      uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
      for (unsigned int i = 0; i < image->width * image->height; ++i)
        if (data[i] != 0)
              data[i] = static_cast<uint16_t>(data[i] * z_scaling_);
    }

    sensor_msgs::CameraInfoPtr cam_info;

    if (depth_registration_)
    {
      image->header.frame_id = color_frame_id_;
      cam_info = getColorCameraInfo(image->width,image->height, image->header.stamp);
    } else
    {
      image->header.frame_id = depth_frame_id_;
      cam_info = getDepthCameraInfo(image->width,image->height, image->header.stamp);
    }

    if (depth_raw_subscribers_)
    {
      pub_depth_raw_.publish(image, cam_info);
    }

    if (depth_subscribers_ )
    {
      sensor_msgs::ImageConstPtr floating_point_image = rawToFloatingPointConversion(image);
      pub_depth_.publish(floating_point_image, cam_info);
    }

    // Projector "info" probably only useful for working with disparity images
    if (projector_info_subscribers_)
    {
      pub_projector_info_.publish(getProjectorCameraInfo(image->width, image->height, image->header.stamp));
    }
  }
}

// Methods to get calibration parameters for the various cameras
sensor_msgs::CameraInfoPtr AstraAdvancedDriver::getDefaultCameraInfo(int width, int height, bool is_ir_camera) const
{
  sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
  if (!IsInit())
  {
    ROS_ERROR_STREAM(GetLogPrefix("getDefaultCameraInfo", ns_) << "device hasn't been initialized");
    return info;
  }

  // left parameters are for IR camera, right parameters are for RBG camera
  OBCameraParams camera_params = device_->getCameraParameters();

  info->width = width;
  info->height = height;

  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  info->D.resize(5);
  
  info->K.assign(0.0);
  info->K[8] = 1.0;

  // For now use no lens distortion parameters
  info->D[0] = info->D[1] = info->D[2] = info->D[3] = info->D[4] = 0;

  if (is_ir_camera) {
    // info->D.assign(camera_params.l_k, camera_params.l_k + 5);
    info->K[0] = camera_params.l_intr_p[0]; // fx
    info->K[2] = camera_params.l_intr_p[2]; // cx
    info->K[4] = camera_params.l_intr_p[1]; // fy
    info->K[5] = camera_params.l_intr_p[3]; // cy
  } else {
    // info->D.assign(camera_params.r_k, camera_params.r_k + 5);
    info->K[0] = camera_params.r_intr_p[0]; // fx
    info->K[2] = camera_params.r_intr_p[2]; // cx
    info->K[4] = camera_params.r_intr_p[1]; // fy
    info->K[5] = camera_params.r_intr_p[3]; // cy
  }

  info->R[0] = camera_params.r2l_r[0];
  info->R[1] = camera_params.r2l_r[1];
  info->R[2] = camera_params.r2l_r[2];
  info->R[3] = camera_params.r2l_r[3];
  info->R[4] = camera_params.r2l_r[4];
  info->R[5] = camera_params.r2l_r[5];
  info->R[6] = camera_params.r2l_r[6];
  info->R[7] = camera_params.r2l_r[7];
  info->R[8] = camera_params.r2l_r[8];

  info->P.assign(0.0);
  info->P[0] = info->K[0];  // fx
  info->P[2] = info->K[2];  // cx
  info->P[5] = info->K[4];  // fy
  info->P[6] = info->K[5];  // cy
  info->P[10] = 1.0;
  info->P[3] = camera_params.r2l_t[0];    // Tx
  info->P[7] = camera_params.r2l_t[1];    // Ty
  info->P[11] = camera_params.r2l_t[2];   // Tz

  return info;
}


/// @todo Use binning/ROI properly in publishing camera infos
sensor_msgs::CameraInfoPtr AstraAdvancedDriver::getColorCameraInfo(int width, int height, ros::Time time) const
{
  sensor_msgs::CameraInfoPtr info;

  if (color_info_manager_->isCalibrated())
  {
    info = boost::make_shared<sensor_msgs::CameraInfo>(color_info_manager_->getCameraInfo());
    if ( info->width != (unsigned int)width )
    {
      // Use uncalibrated values
      ROS_WARN_ONCE("Image resolution doesn't match calibration of the RGB camera. Using default parameters.");
      info = getDefaultCameraInfo(width, height, false);
    }
  }
  else
  {
    // If uncalibrated, fill in default values
    info = getDefaultCameraInfo(width, height, false);
  }

  // Fill in header
  info->header.stamp    = time;
  info->header.frame_id = color_frame_id_;

  return info;
}


sensor_msgs::CameraInfoPtr AstraAdvancedDriver::getIRCameraInfo(int width, int height, ros::Time time) const
{
  sensor_msgs::CameraInfoPtr info;

  if (ir_info_manager_->isCalibrated())
  {
    info = boost::make_shared<sensor_msgs::CameraInfo>(ir_info_manager_->getCameraInfo());
    if ( info->width != (unsigned int)width )
    {
      // Use uncalibrated values
      ROS_WARN_ONCE("Image resolution doesn't match calibration of the IR camera. Using default parameters.");
      info = getDefaultCameraInfo(width, height, true);
    }
  }
  else
  {
    // If uncalibrated, fill in default values
    info = getDefaultCameraInfo(width, height, true);
  }

  // Fill in header
  info->header.stamp    = time;
  info->header.frame_id = depth_frame_id_;

  return info;
}

sensor_msgs::CameraInfoPtr AstraAdvancedDriver::getDepthCameraInfo(int width, int height, ros::Time time) const
{
  // The depth image has essentially the same intrinsics as the IR image, BUT the
  // principal point is offset by half the size of the hardware correlation window
  // (probably 9x9 or 9x7 in 640x480 mode). See http://www.ros.org/wiki/kinect_calibration/technical

  double scaling = (double)width / 640;

  sensor_msgs::CameraInfoPtr info = getIRCameraInfo(width, height, time);
  info->K[2] -= depth_ir_offset_x_*scaling; // cx
  info->K[5] -= depth_ir_offset_y_*scaling; // cy
  info->P[2] -= depth_ir_offset_x_*scaling; // cx
  info->P[6] -= depth_ir_offset_y_*scaling; // cy

  /// @todo Could put this in projector frame so as to encode the baseline in P[3]
  return info;
}

sensor_msgs::CameraInfoPtr AstraAdvancedDriver::getProjectorCameraInfo(int width, int height, ros::Time time) const
{
  sensor_msgs::CameraInfoPtr info = getDepthCameraInfo(width, height, time);
  if (!IsInit())
  {
    ROS_ERROR_STREAM(GetLogPrefix("getProjectorCameraInfo", ns_) << "device hasn't been initialized");
    return info;
  }

  // The projector info is simply the depth info with the baseline encoded in the P matrix.
  // It's only purpose is to be the "right" camera info to the depth camera's "left" for
  // processing disparity images.
  // Tx = -baseline * fx
  info->P[3] = -device_->getBaseline() * info->P[0];
  return info;
}

void AstraAdvancedDriver::readConfigFromParameterServer()
{
  pnh_.getParam("device_id", device_id_);

  // Camera TF frames
  pnh_.param("/" + ns_ + "/ir_frame_id", ir_frame_id_, std::string("/openni_ir_optical_frame"));
  pnh_.param("/" + ns_ + "/rgb_frame_id", color_frame_id_, std::string("/openni_rgb_optical_frame"));
  pnh_.param("/" + ns_ + "/depth_frame_id", depth_frame_id_, std::string("/openni_depth_optical_frame"));

  ROS_DEBUG("ir_frame_id = '%s' ", ir_frame_id_.c_str());
  ROS_DEBUG("rgb_frame_id = '%s' ", color_frame_id_.c_str());
  ROS_DEBUG("depth_frame_id = '%s' ", depth_frame_id_.c_str());

  pnh_.param("/" + ns_ + "/rgb_camera_info_url", color_info_url_, std::string());
  pnh_.param("/" + ns_ + "/depth_camera_info_url", ir_info_url_, std::string());
  double depth_callback_timeout = 30; // seconds
  pnh_.param("/" + ns_ + "/depth_callback_timeout", depth_callback_timeout, depth_callback_timeout);
  depth_callback_timeout_ = ros::Duration(depth_callback_timeout);
}

std::string AstraAdvancedDriver::resolveDeviceURI(const std::string& device_id) throw(AstraException)
{
  // retrieve available device URIs, they look like this: "1d27/0601@1/5"
  // which is <vendor ID>/<product ID>@<bus number>/<device number>
  boost::shared_ptr<std::vector<std::string> > available_device_URIs =
    device_manager_->getConnectedDeviceURIs();

  //for tes
  #if 0
   for (size_t i = 0; i < available_device_URIs->size(); ++i)
   {
       std::string s = (*available_device_URIs)[i];
  	ROS_WARN("------------id %d, available_device_uri is %s-----------", i, s.c_str());
   }
   #endif
  //end
  // look for '#<number>' format
  if (device_id.size() > 1 && device_id[0] == '#')
  {
    std::istringstream device_number_str(device_id.substr(1));
    int device_number;
    device_number_str >> device_number;
    int device_index = device_number - 1; // #1 refers to first device
    if (device_index >= available_device_URIs->size() || device_index < 0)
    {
      THROW_OPENNI_EXCEPTION(
          "Invalid device number %i, there are %zu devices connected.",
          device_number, available_device_URIs->size());
    }
    else
    {
      return available_device_URIs->at(device_index);
    }
  }
  // look for '<bus>@<number>' format
  //   <bus>    is usb bus id, typically start at 1
  //   <number> is the device number, for consistency with astra_camera, these start at 1
  //               although 0 specifies "any device on this bus"
  else if (device_id.size() > 1 && device_id.find('@') != std::string::npos && device_id.find('/') == std::string::npos)
  {
    // get index of @ character
    size_t index = device_id.find('@');
    if (index <= 0)
    {
      THROW_OPENNI_EXCEPTION(
        "%s is not a valid device URI, you must give the bus number before the @.",
        device_id.c_str());
    }
    if (index >= device_id.size() - 1)
    {
      THROW_OPENNI_EXCEPTION(
        "%s is not a valid device URI, you must give a number after the @, specify 0 for first device",
        device_id.c_str());
    }

    // pull out device number on bus
    std::istringstream device_number_str(device_id.substr(index+1));
    int device_number;
    device_number_str >> device_number;

    // reorder to @<bus>
    std::string bus = device_id.substr(0, index);
    bus.insert(0, "@");

    for (size_t i = 0; i < available_device_URIs->size(); ++i)
    {
      std::string s = (*available_device_URIs)[i];
      if (s.find(bus) != std::string::npos)
      {
        // this matches our bus, check device number
        --device_number;
        if (device_number <= 0)
          return s;
      }
    }

    THROW_OPENNI_EXCEPTION("Device not found %s", device_id.c_str());
  }
  else
  {
    // check if the device id given matches a serial number of a connected device
    for(std::vector<std::string>::const_iterator it = available_device_URIs->begin();
        it != available_device_URIs->end(); ++it)
    {
	#if 0
      	try 
	{
        	std::string serial = device_manager_->getSerial(*it);
        	if (serial.size() > 0 && device_id == serial)
          		return *it;
	}
    	#else
	try 
	{
         	std::set<std::string>::iterator iter;
        	if((iter = alreadyOpen.find(*it)) == alreadyOpen.end())
        	{
              		// ROS_WARN("------------seraial num it is  %s, device_id is %s -----------", (*it).c_str(), device_id_.c_str());
        		std::string serial = device_manager_->getSerial(*it);
            ROS_INFO_STREAM(GetLogPrefix("AstraAdvancedDriver", "") << "serial: " << serial << ", uri: " << it->c_str());
        	 	if (serial.size() > 0 && device_id == serial)
        		{
          			alreadyOpen.insert(*it);
          			return *it;
         		}
        	}
      	}
	#endif
      	catch (const AstraException& exception)
      	{
        	//ROS_WARN("Could not query serial number of device \"%s\":", exception.what());
        	ROS_ERROR("Could not query serial number of device \"%s\":", exception.what());
      	}
    }

    // everything else is treated as part of the device_URI
    bool match_found = false;
    std::string matched_uri;
    for (size_t i = 0; i < available_device_URIs->size(); ++i)
    {
      std::string s = (*available_device_URIs)[i];
      if (s.find(device_id) != std::string::npos)
      {
        if (!match_found)
        {
          matched_uri = s;
          match_found = true;
        }
        else
        {
          // more than one match
          THROW_OPENNI_EXCEPTION("Two devices match the given device id '%s': %s and %s.", device_id.c_str(), matched_uri.c_str(), s.c_str());
        }
      }
    }
    return matched_uri;
  }

  return "INVALID";
}

void AstraAdvancedDriver::initDevice()
{
  ROS_INFO_STREAM(GetLogPrefix("initDevice", ns_) << "STARTED, device_id_: " << device_id_);
  try
  {
    std::string device_URI = resolveDeviceURI(device_id_);
    ROS_INFO_STREAM(GetLogPrefix("initDevice", ns_) << "device_id_: " << device_id_ << ", resolved uri: " << device_URI);
    if (device_URI.length() > 0) 
    {
      namespace bi = boost::interprocess;
      bi::named_mutex usb_mutex{bi::open_or_create, "usb_mutex"};
      bi::scoped_lock<bi::named_mutex> lock(usb_mutex);
      if (device_) device_ = nullptr;
      device_ = device_manager_->getDevice(device_URI, projector_control_, ns_, device_id_);
    }
    else
    {
      ROS_ERROR_STREAM(GetLogPrefix("initDevice", ns_) << "empty uri");
      device_ = nullptr;
    }
  }
  catch (const AstraException& exception)
  {
    ROS_ERROR_STREAM(GetLogPrefix("initDevice", ns_) << "No matching device found.... waiting for devices. Reason: " << exception.what());
  }

  if (!device_)
  {
    ROS_ERROR_STREAM(GetLogPrefix("initDevice", ns_) << "No ui of " << device_id_ << " resolved");
  }
  else if (!device_->isValid())
  {
    device_ = nullptr;
    ROS_ERROR_STREAM(GetLogPrefix("initDevice", ns_) << "Could not retrieve device");
  }
  else
    ROS_INFO_STREAM(GetLogPrefix("initDevice", ns_) << "FINISHED");

}

void AstraAdvancedDriver::genVideoModeTableMap()
{
  /*
   * #  Video modes defined by dynamic reconfigure:
output_mode_enum = gen.enum([  gen.const(  "SXGA_30Hz", int_t, 1,  "1280x1024@30Hz"),
                               gen.const(  "SXGA_15Hz", int_t, 2,  "1280x1024@15Hz"),
                               gen.const(   "XGA_30Hz", int_t, 3,  "1280x720@30Hz"),
                               gen.const(   "XGA_15Hz", int_t, 4,  "1280x720@15Hz"),
                               gen.const(   "VGA_30Hz", int_t, 5,  "640x480@30Hz"),
                               gen.const(   "VGA_25Hz", int_t, 6,  "640x480@25Hz"),
                               gen.const(  "QVGA_25Hz", int_t, 7,  "320x240@25Hz"),
                               gen.const(  "QVGA_30Hz", int_t, 8,  "320x240@30Hz"),
                               gen.const(  "QVGA_60Hz", int_t, 9,  "320x240@60Hz"),
                               gen.const( "QQVGA_25Hz", int_t, 10, "160x120@25Hz"),
                               gen.const( "QQVGA_30Hz", int_t, 11, "160x120@30Hz"),
                               gen.const( "QQVGA_60Hz", int_t, 12, "160x120@60Hz")],
                               "output mode")
  */

  video_modes_lookup_.clear();

  AstraVideoMode video_mode;

  // SXGA_30Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 1024;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[1] = video_mode;

  // SXGA_15Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 1024;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_[2] = video_mode;

  // XGA_30Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 720;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[3] = video_mode;

  // XGA_15Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 720;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_[4] = video_mode;

  // VGA_30Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 480;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[5] = video_mode;

  // VGA_25Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 480;
  video_mode.frame_rate_ = 25;

  video_modes_lookup_[6] = video_mode;

  // QVGA_25Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 25;

  video_modes_lookup_[7] = video_mode;

  // QVGA_30Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[8] = video_mode;

  // QVGA_60Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_[9] = video_mode;

  // QQVGA_25Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 25;

  video_modes_lookup_[10] = video_mode;

  // QQVGA_30Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[11] = video_mode;

  // QQVGA_60Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_[12] = video_mode;

}

int AstraAdvancedDriver::lookupVideoModeFromDynConfig(int mode_nr, AstraVideoMode& video_mode)
{
  int ret = -1;

  std::map<int, AstraVideoMode>::const_iterator it;

  it = video_modes_lookup_.find(mode_nr);

  if (it!=video_modes_lookup_.end())
  {
    video_mode = it->second;
    ret = 0;
  }

  return ret;
}

sensor_msgs::ImageConstPtr AstraAdvancedDriver::rawToFloatingPointConversion(sensor_msgs::ImageConstPtr raw_image)
{
  static const float bad_point = std::numeric_limits<float>::quiet_NaN ();

  sensor_msgs::ImagePtr new_image = boost::make_shared<sensor_msgs::Image>();

  new_image->header = raw_image->header;
  new_image->width = raw_image->width;
  new_image->height = raw_image->height;
  new_image->is_bigendian = 0;
  new_image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  new_image->step = sizeof(float)*raw_image->width;

  std::size_t data_size = new_image->width*new_image->height;
  new_image->data.resize(data_size*sizeof(float));

  const unsigned short* in_ptr = reinterpret_cast<const unsigned short*>(&raw_image->data[0]);
  float* out_ptr = reinterpret_cast<float*>(&new_image->data[0]);

  for (std::size_t i = 0; i<data_size; ++i, ++in_ptr, ++out_ptr)
  {
    if (*in_ptr==0 || *in_ptr==0x7FF)
    {
      *out_ptr = bad_point;
    } else
    {
      *out_ptr = static_cast<float>(*in_ptr)/1000.0f;
    }
  }

  return new_image;
}

}
