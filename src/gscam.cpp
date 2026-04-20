// Copyright 2022 Jonathan Bohren, Clyde McQueen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdlib.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <algorithm>
#include <iostream>
#include <string>

extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
}

#include "camera_info_manager/camera_info_manager.hpp"
#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "gscam/gscam.hpp"

namespace gscam
{

GSCam::GSCam(const rclcpp::NodeOptions & options)
: rclcpp::Node("gscam_publisher", options),
  gsconfig_(""),
  pipeline_(NULL),
  sink_(NULL),
  camera_info_manager_(this),
  stop_signal_(false)
{
  if (!configure()) {
    RCLCPP_FATAL(get_logger(), "Failed to configure gscam!");
    return;
  }
  int period_ms = 1000 / camera_frame_rate_;
  timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&GSCam::publish_stream, this));
  pipeline_thread_ = std::thread(
    [this]()
    {
      run();
    });
}

GSCam::~GSCam()
{
  stop_signal_ = true;
  if (pipeline_thread_.joinable()) {
    pipeline_thread_.join();
  }
}

bool GSCam::configure()
{
  // Get gstreamer configuration
  // (either from environment variable or ROS param)
  bool gsconfig_rosparam_defined = false;
  char * gsconfig_env = NULL;

  const auto gsconfig_rosparam = declare_parameter("gscam_config", "");
  gsconfig_rosparam_defined = !gsconfig_rosparam.empty();
  gsconfig_env = getenv("GSCAM_CONFIG");

  if (!gsconfig_env && !gsconfig_rosparam_defined) {
    RCLCPP_FATAL(
      get_logger(),
      "Problem getting GSCAM_CONFIG environment variable and "
      "'gscam_config' rosparam is not set. This is needed to set up a gstreamer pipeline.");
    return false;
  } else if (gsconfig_env && gsconfig_rosparam_defined) {
    RCLCPP_FATAL(
      get_logger(),
      "Both GSCAM_CONFIG environment variable and 'gscam_config' rosparam are set. "
      "Please only define one.");
    return false;
  } else if (gsconfig_env) {
    gsconfig_ = gsconfig_env;
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Using gstreamer config from env: \"" << gsconfig_env << "\"");
  } else if (gsconfig_rosparam_defined) {
    gsconfig_ = gsconfig_rosparam;
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Using gstreamer config from rosparam: \"" << gsconfig_rosparam << "\"");
  }

  // Get additional gscam configuration
  sync_sink_ = declare_parameter("sync_sink", true);
  preroll_ = declare_parameter("preroll", false);
  use_gst_timestamps_ = declare_parameter("use_gst_timestamps", false);

  reopen_on_eof_ = declare_parameter("reopen_on_eof", false);

  // Get the camera parameters file
  camera_info_url_ = declare_parameter("camera_info_url", "");
  camera_name_ = declare_parameter("camera_name", "");

  // Get the image encoding
  image_encoding_ =
    declare_parameter("image_encoding", std::string(sensor_msgs::image_encodings::RGB8));
  if (image_encoding_ != sensor_msgs::image_encodings::RGB8 &&
    image_encoding_ != sensor_msgs::image_encodings::MONO8 &&
    image_encoding_ != sensor_msgs::image_encodings::YUV422 &&
    image_encoding_ != "jpeg")
  {
    RCLCPP_FATAL_STREAM(get_logger(), "Unsupported image encoding: " + image_encoding_);
  }

  camera_info_manager_.setCameraName(camera_name_);

  if (camera_info_manager_.validateURL(camera_info_url_)) {
    camera_info_manager_.loadCameraInfo(camera_info_url_);
    RCLCPP_INFO_STREAM(get_logger(), "Loaded camera calibration from " << camera_info_url_);
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Camera info at: " << camera_info_url_ << " not found. Using an uncalibrated config.");
  }

  // Get TF Frame
  frame_id_ = declare_parameter("frame_id", "camera_frame");
  if (frame_id_ == "camera_frame") {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "No camera frame_id set, using frame \"" << frame_id_ << "\".");
  }

  use_sensor_data_qos_ = declare_parameter("use_sensor_data_qos", false);
  camera_frame_rate_ = declare_parameter("camera_frame_rate", 15);
  enable_rectifying_ = declare_parameter("enable_rectifying", true);

  return true;
}

bool GSCam::init_stream()
{
  if (!gst_is_initialized()) {
    // Initialize gstreamer pipeline
    RCLCPP_DEBUG_STREAM(get_logger(), "Initializing gstreamer...");
    gst_init(0, 0);
  }

  RCLCPP_DEBUG_STREAM(get_logger(), "Gstreamer Version: " << gst_version_string() );

  GError * error = 0;  // Assignment to zero is a gst requirement

  pipeline_ = gst_parse_launch(gsconfig_.c_str(), &error);
  if (pipeline_ == NULL) {
    RCLCPP_FATAL_STREAM(get_logger(), error->message);
    return false;
  }

  // Create RGB sink
  sink_ = gst_element_factory_make("appsink", NULL);
  GstCaps * caps = gst_app_sink_get_caps(GST_APP_SINK(sink_));

  // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
  if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "RGB",
      NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "GRAY8",
      NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::YUV422) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "UYVY",
      NULL);
  } else if (image_encoding_ == "jpeg") {
    caps = gst_caps_new_simple("image/jpeg", NULL, NULL);
  }

  gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
  gst_caps_unref(caps);

  // Set whether the sink should sync
  // Sometimes setting this to true can cause a large number of frames to be
  // dropped
  gst_base_sink_set_sync(
    GST_BASE_SINK(sink_),
    (sync_sink_) ? TRUE : FALSE);

  if (GST_IS_PIPELINE(pipeline_)) {
    GstPad * outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
    g_assert(outpad);

    GstElement * outelement = gst_pad_get_parent_element(outpad);
    g_assert(outelement);
    gst_object_unref(outpad);

    if (!gst_bin_add(GST_BIN(pipeline_), sink_)) {
      RCLCPP_FATAL(get_logger(), "gst_bin_add() failed");
      gst_object_unref(outelement);
      gst_object_unref(pipeline_);
      return false;
    }

    if (!gst_element_link(outelement, sink_)) {
      RCLCPP_FATAL(
        get_logger(), "GStreamer: cannot link outelement(\"%s\") -> sink\n",
        gst_element_get_name(outelement));
      gst_object_unref(outelement);
      gst_object_unref(pipeline_);
      return false;
    }

    gst_object_unref(outelement);
  } else {
    GstElement * launchpipe = pipeline_;
    pipeline_ = gst_pipeline_new(NULL);
    g_assert(pipeline_);

    gst_object_unparent(GST_OBJECT(launchpipe));

    gst_bin_add_many(GST_BIN(pipeline_), launchpipe, sink_, NULL);

    if (!gst_element_link(launchpipe, sink_)) {
      RCLCPP_FATAL(get_logger(), "GStreamer: cannot link launchpipe -> sink");
      gst_object_unref(pipeline_);
      return false;
    }
  }

  // Calibration between ros::Time and gst timestamps
  GstClock * clock = gst_system_clock_obtain();
  GstClockTime ct = gst_clock_get_time(clock);
  gst_object_unref(clock);
  time_offset_ = now().nanoseconds() - GST_TIME_AS_NSECONDS(ct);
  RCLCPP_INFO(get_logger(), "Time offset: %.6f", rclcpp::Time(time_offset_).seconds());

  gst_element_set_state(pipeline_, GST_STATE_PAUSED);

  if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_FATAL(get_logger(), "Failed to PAUSE stream, check your gstreamer configuration.");
    return false;
  } else {
    RCLCPP_DEBUG_STREAM(get_logger(), "Stream is PAUSED.");
  }

  // Create ROS camera interface
  // const auto qos = use_sensor_data_qos_ ? rclcpp::SensorDataQoS() : rclcpp::QoS{1};
  const auto qos = rclcpp::QoS{100};

  if (image_encoding_ == "jpeg") {
    jpeg_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>(
      "image_raw/compressed", qos);
    cinfo_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", qos);
  } else {
    // Use regular publishers instead of image_transport to avoid compressed variants
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "image_raw", qos);
    cinfo_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", qos);
  }
  camera_pub_rect_ = create_publisher<sensor_msgs::msg::Image>(
    "image_rect", qos);

  return true;
}

void GSCam::update_stream()
{
  RCLCPP_INFO_STREAM(get_logger(), "Publishing stream...");

  // Pre-roll camera if needed
  if (preroll_) {
    RCLCPP_DEBUG(get_logger(), "Performing preroll...");

    // The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
    // I am told this is needed and am erring on the side of caution.
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to PLAY during preroll.");
      return;
    } else {
      RCLCPP_DEBUG(get_logger(), "Stream is PLAYING in preroll.");
    }

    gst_element_set_state(pipeline_, GST_STATE_PAUSED);
    if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to PAUSE.");
      return;
    } else {
      RCLCPP_INFO(get_logger(), "Stream is PAUSED in preroll.");
    }
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "Could not start stream!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Started stream.");

  // Poll the data as fast a spossible
  while (!stop_signal_ && rclcpp::ok()) {
    // This should block until a new frame is awake, this way, we'll run at the
    // actual capture framerate of the device.
    // RCLCPP_DEBUG(get_logger(), "Getting data...");
    GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
    if (!sample) {
      RCLCPP_ERROR(get_logger(), "Could not get gstreamer sample.");
      break;
    }

    GstBuffer * buf = gst_sample_get_buffer(sample);
    if (!buf) {
      RCLCPP_INFO(get_logger(), "Stream ended.");
      gst_sample_unref(sample);
      break;
    }

    GstMapInfo info;
    if (!gst_buffer_map(buf, &info, GST_MAP_READ)) {
      RCLCPP_ERROR(get_logger(), "Could not map gstreamer buffer.");
      gst_sample_unref(sample);
      continue;
    }

    const gsize buf_size = info.size;
    const guint8 * buf_data = info.data;

    GstClockTime bt = gst_element_get_base_time(pipeline_);
    // RCLCPP_INFO(
    //   get_logger(),
    //   "New buffer: timestamp %.6f %lu %lu %.3f",
    //   GST_TIME_AS_USECONDS(buf->timestamp + bt) / 1e6 + time_offset_,
    //   buf->timestamp, bt, time_offset_);


#if 0
    GstFormat fmt = GST_FORMAT_TIME;
    gint64 current = -1;

    Query the current position of the stream
    if (gst_element_query_position(pipeline_, &fmt, &current)) {
      RCLCPP_INFO_STREAM(get_logger(), "Position " << current);
    }
#endif

    // RCLCPP_DEBUG(get_logger(), "Got data.");

    // Get the image width and height
    GstPad * pad = gst_element_get_static_pad(sink_, "sink");
    const GstCaps * caps = gst_pad_get_current_caps(pad);
    if (!caps) {
      RCLCPP_ERROR(get_logger(), "Could not get current caps from sink pad.");
      gst_object_unref(pad);
      gst_buffer_unmap(buf, &info);
      gst_sample_unref(sample);
      continue;
    }

    GstStructure * structure = gst_caps_get_structure(caps, 0);
    if (!structure) {
      RCLCPP_ERROR(get_logger(), "Could not get caps structure.");
      gst_caps_unref(const_cast<GstCaps *>(caps));
      gst_object_unref(pad);
      gst_buffer_unmap(buf, &info);
      gst_sample_unref(sample);
      continue;
    }

    gst_structure_get_int(structure, "width", &width_);
    gst_structure_get_int(structure, "height", &height_);
    gst_caps_unref(const_cast<GstCaps *>(caps));
    gst_object_unref(pad);

    // Update message payload under lock as publish_stream() runs concurrently.
    std::lock_guard<std::mutex> lock(message_mutex_);

    cinfo_msg_ = camera_info_manager_.getCameraInfo();

    if (use_gst_timestamps_) {
      cinfo_msg_.header.stamp = rclcpp::Time(GST_TIME_AS_NSECONDS(buf->pts + bt) + time_offset_);
    } else {
      cinfo_msg_.header.stamp = now();
    }
    // RCLCPP_INFO(get_logger(), "Image time stamp: %.3f",cinfo->header.stamp.toSec());
    cinfo_msg_.header.frame_id = frame_id_;
    if (image_encoding_ == "jpeg") {
      comp_img_msg_.header = cinfo_msg_.header;
      comp_img_msg_.format = "jpeg";
      comp_img_msg_.data.resize(buf_size);
      std::copy_n(buf_data, buf_size, comp_img_msg_.data.begin());
    } else {
      // Complain if the returned buffer is smaller than we expect
      const unsigned int expected_frame_size =
        width_ * height_ * sensor_msgs::image_encodings::numChannels(image_encoding_);

      if (buf_size < expected_frame_size) {
        RCLCPP_WARN_STREAM(
          get_logger(), "GStreamer image buffer underflow: Expected frame to be " <<
            expected_frame_size << " bytes but got only " <<
            buf_size << " bytes. (make sure frames are correctly encoded)");
      } else if (buf_size > expected_frame_size) {
        RCLCPP_WARN_STREAM(
          get_logger(), "GStreamer image buffer overflow: Expected frame to be " <<
            expected_frame_size << " bytes but got " <<
            buf_size << " bytes. Truncating extra bytes.");
      }

      img_msg_.header = cinfo_msg_.header;

      // Image data and metadata
      img_msg_.width = width_;
      img_msg_.height = height_;
      img_msg_.encoding = image_encoding_;
      img_msg_.is_bigendian = false;
      img_msg_.data.resize(expected_frame_size);

      // Copy only the data we received
      // Since we're publishing shared pointers, we need to copy the image so
      // we can free the buffer allocated by gstreamer
      img_msg_.step = width_ * sensor_msgs::image_encodings::numChannels(image_encoding_);
      const auto bytes_to_copy = std::min(static_cast<size_t>(buf_size),
        static_cast<size_t>(expected_frame_size));

      std::copy_n(buf_data, bytes_to_copy, img_msg_.data.begin());
    }

    // Release the buffer
    gst_buffer_unmap(buf, &info);
    gst_sample_unref(sample);
  }
}

void GSCam::publish_stream()
{
  sensor_msgs::msg::Image img_msg;
  sensor_msgs::msg::CompressedImage comp_img_msg;
  sensor_msgs::msg::CameraInfo cinfo_msg;

  {
    std::lock_guard<std::mutex> lock(message_mutex_);
    img_msg = img_msg_;
    comp_img_msg = comp_img_msg_;
    cinfo_msg = cinfo_msg_;
  }

  if (image_encoding_ == "jpeg") {
    jpeg_pub_->publish(comp_img_msg);
    cinfo_pub_->publish(cinfo_msg);
  } else {
    // Publish raw image and camera info
    image_pub_->publish(img_msg);
    cinfo_pub_->publish(cinfo_msg);
  }
  if (enable_rectifying_) {
    sensor_msgs::msg::Image img_rect_msg;
    try {
      cv_bridge::CvImage img_bridge_rect =
        cv_bridge::CvImage(img_msg.header, sensor_msgs::image_encodings::RGB8);
      (void)img_bridge_rect;

      cv_bridge::CvImagePtr cv_img_raw = cv_bridge::toCvCopy(img_msg, img_msg.encoding);
      pinhole_model_.fromCameraInfo(cinfo_msg);
      pinhole_model_.rectifyImage(cv_img_raw->image, img_bridge_rect.image);

      img_bridge_rect.toImageMsg(img_rect_msg);
      camera_pub_rect_->publish(std::move(img_rect_msg));
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Runtime error, publish_rectified_image.");
    }
  }
}

void GSCam::cleanup_stream()
{
  // Clean up
  RCLCPP_INFO(get_logger(), "Stopping gstreamer pipeline...");
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = NULL;
  }
}

void GSCam::run()
{
  while (!stop_signal_ && rclcpp::ok()) {
    if (!this->init_stream()) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize gscam stream!");
      break;
    }

    // Block while publishing
    this->update_stream();

    this->cleanup_stream();

    RCLCPP_INFO(get_logger(), "GStreamer stream stopped!");

    if (reopen_on_eof_) {
      RCLCPP_INFO(get_logger(), "Reopening stream...");
    } else {
      RCLCPP_INFO(get_logger(), "Cleaning up stream and exiting...");
      break;
    }
  }
  rclcpp::shutdown();
}

// Example callbacks for appsink
// TODO(someone): enable callback-based capture
void gst_eos_cb(GstAppSink * appsink, gpointer user_data)
{
}
GstFlowReturn gst_new_preroll_cb(GstAppSink * appsink, gpointer user_data)
{
  return GST_FLOW_OK;
}
GstFlowReturn gst_new_asample_cb(GstAppSink * appsink, gpointer user_data)
{
  return GST_FLOW_OK;
}

}  // namespace gscam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gscam::GSCam)
