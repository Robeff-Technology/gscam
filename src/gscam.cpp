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
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
}

#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "gscam/gscam.hpp"

namespace gscam
{

GSCam::GSCam(const rclcpp::NodeOptions & options)
: rclcpp::Node("gscam_publisher", options),
  gsconfig_(""),
  pipeline_(NULL),
  sink_(NULL),
  camera_info_manager_(this),
  grid_snap_latency_(0.0),
  grid_epoch_ns_(0),
  grid_epoch_set_(false),
  grid_stats_since_ns_(0),
  grid_stat_count_(0),
  grid_stat_dropped_(0),
  grid_stat_outliers_(0),
  grid_stat_resets_(0),
  grid_stat_duplicates_(0),
  grid_stat_sum_(0.0),
  grid_stat_sum_sq_(0.0),
  grid_stat_max_abs_(0.0),
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
  pipeline_thread_.join();
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
    image_encoding_ != sensor_msgs::image_encodings::BGR8 &&
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

  // Frame-sync grid snapping. The NPL54 drives every camera from one hardware
  // frame-sync generator, so true capture instants lie on a uniform grid; this
  // recovers that grid and replaces the jittery host arrival time with it.
  grid_snap_enabled_ = declare_parameter("grid_snap", false);
  grid_snap_latency_ = declare_parameter("grid_snap_latency", 0.0);
  grid_param_cb_ = add_on_set_parameters_callback(
    std::bind(&GSCam::on_set_parameters, this, std::placeholders::_1));
  grid_snap_diagnostics_ = declare_parameter("grid_snap_diagnostics", true);
  grid_snap_report_period_ = declare_parameter("grid_snap_report_period", 10.0);

  FrameGridTracker::Options grid_opt;
  const double grid_fps = declare_parameter("grid_snap_fps", 0.0);
  grid_opt.fps = grid_fps > 0.0 ? grid_fps : static_cast<double>(camera_frame_rate_);
  grid_opt.phase_gain = declare_parameter("grid_snap_phase_gain", 0.05);
  grid_opt.period_gain = declare_parameter("grid_snap_period_gain", 5e-4);
  grid_opt.reset_threshold = declare_parameter("grid_snap_reset_threshold", 0.35);
  grid_opt.max_gap = declare_parameter("grid_snap_max_gap", 2.0);

  if (grid_opt.fps <= 0.0) {
    RCLCPP_FATAL(get_logger(), "grid_snap needs a positive frame rate.");
    return false;
  }

  grid_tracker_ = std::make_unique<FrameGridTracker>(grid_opt);

  if (grid_snap_enabled_) {
    RCLCPP_INFO(
      get_logger(),
      "Frame-sync grid snapping enabled: %.3f fps grid, %.1f ms latency offset.",
      grid_opt.fps, grid_snap_latency_.load() * 1e3);

    if (grid_snap_diagnostics_) {
      // [residual_s, step, measured_fps, flags] with flags bit0=outlier,
      // bit1=reset. Record this topic to histogram the residuals offline.
      grid_diag_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/grid_diagnostics", rclcpp::SensorDataQoS());
    }
  }

  return true;
}

rclcpp::Time GSCam::snap_to_frame_grid(const rclcpp::Time & raw)
{
  if (!grid_snap_enabled_ || !grid_tracker_) {
    return raw;
  }

  const int64_t raw_ns = raw.nanoseconds();
  if (!grid_epoch_set_) {
    // Anchor on whole seconds so the relative value stays small and exact.
    grid_epoch_ns_ = (raw_ns / 1000000000LL) * 1000000000LL;
    grid_epoch_set_ = true;
    grid_stats_since_ns_ = raw_ns;
  }

  const auto result = grid_tracker_->update(static_cast<double>(raw_ns - grid_epoch_ns_) * 1e-9);

  ++grid_stat_count_;
  if (result.reset) {
    ++grid_stat_resets_;
  } else if (result.step == 0) {
    ++grid_stat_duplicates_;
  } else {
    if (result.outlier) {
      ++grid_stat_outliers_;
    }
    grid_stat_dropped_ += static_cast<uint64_t>(result.step - 1);
    grid_stat_sum_ += result.residual;
    grid_stat_sum_sq_ += result.residual * result.residual;
    grid_stat_max_abs_ = std::max(grid_stat_max_abs_, std::fabs(result.residual));
  }

  if (grid_diag_pub_) {
    std_msgs::msg::Float64MultiArray diag;
    diag.data = {
      result.residual,
      static_cast<double>(result.step),
      grid_tracker_->measured_fps(),
      static_cast<double>((result.outlier ? 1 : 0) | (result.reset ? 2 : 0))};
    grid_diag_pub_->publish(diag);
  }

  if (grid_snap_report_period_ > 0.0 &&
    raw_ns - grid_stats_since_ns_ >= static_cast<int64_t>(grid_snap_report_period_ * 1e9))
  {
    report_grid_stats();
    grid_stats_since_ns_ = raw_ns;
  }

  // The tracked grid sits `grid_snap_latency_` after the true start of frame;
  // that offset is constant across all cameras and is calibrated separately.
  const int64_t snapped_ns = grid_epoch_ns_ +
    static_cast<int64_t>(std::llround((result.stamp - grid_snap_latency_.load()) * 1e9));
  return rclcpp::Time(snapped_ns, raw.get_clock_type());
}

rcl_interfaces::msg::SetParametersResult GSCam::on_set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    // Only the latency offset is live-tunable: it is a pure output shift, so
    // changing it cannot disturb the tracking loop. Everything else would need
    // the loop to re-lock and is read once at startup.
    if (parameter.get_name() == "grid_snap_latency") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "grid_snap_latency must be a double (seconds)";
        break;
      }
      grid_snap_latency_ = parameter.as_double();
      RCLCPP_INFO(
        get_logger(), "grid_snap_latency set to %.3f ms",
        grid_snap_latency_.load() * 1e3);
    }
  }

  return result;
}

void GSCam::report_grid_stats()
{
  const uint64_t n = grid_stat_count_ - grid_stat_resets_ - grid_stat_duplicates_;
  if (n == 0) {
    return;
  }

  const double mean = grid_stat_sum_ / static_cast<double>(n);
  const double var = std::max(0.0, grid_stat_sum_sq_ / static_cast<double>(n) - mean * mean);

  RCLCPP_INFO(
    get_logger(),
    "grid_snap: %lu frames, residual mean %.3f ms / sd %.3f ms / max %.3f ms, "
    "fps %.6f (%+.1f ppm), dropped %lu, outliers %lu, duplicates %lu, resets %lu",
    static_cast<unsigned long>(n), mean * 1e3, std::sqrt(var) * 1e3,
    grid_stat_max_abs_ * 1e3, grid_tracker_->measured_fps(), grid_tracker_->ppm(),
    static_cast<unsigned long>(grid_stat_dropped_),
    static_cast<unsigned long>(grid_stat_outliers_),
    static_cast<unsigned long>(grid_stat_duplicates_),
    static_cast<unsigned long>(grid_stat_resets_));

  grid_stat_count_ = 0;
  grid_stat_dropped_ = 0;
  grid_stat_outliers_ = 0;
  grid_stat_resets_ = 0;
  grid_stat_duplicates_ = 0;
  grid_stat_sum_ = 0.0;
  grid_stat_sum_sq_ = 0.0;
  grid_stat_max_abs_ = 0.0;
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
  } else if (image_encoding_ == sensor_msgs::image_encodings::BGR8) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "BGR",
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
  const auto qos = use_sensor_data_qos_ ? rclcpp::SensorDataQoS() : rclcpp::QoS{1};
  if (image_encoding_ == "jpeg") {
    jpeg_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>(
      "image_raw/compressed", qos);
    cinfo_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", qos);
  } else {
    camera_pub_ = image_transport::create_camera_publisher(
      this, "image_raw", qos.get_rmw_qos_profile());
  }
  camera_pub_rect_ = create_publisher<sensor_msgs::msg::Image>(
    "image_rect",qos);

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
    GstMemory * memory = gst_buffer_get_memory(buf, 0);
    GstMapInfo info;

    gst_memory_map(memory, &info, GST_MAP_READ);
    gsize & buf_size = info.size;
    guint8 * & buf_data = info.data;

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

    // Stop on end of stream
    if (!buf) {
      RCLCPP_INFO(get_logger(), "Stream ended.");
      break;
    }

    // RCLCPP_DEBUG(get_logger(), "Got data.");

    // Get the image width and height
    GstPad * pad = gst_element_get_static_pad(sink_, "sink");
    const GstCaps * caps = gst_pad_get_current_caps(pad);
    GstStructure * structure = gst_caps_get_structure(caps, 0);
    gst_structure_get_int(structure, "width", &width_);
    gst_structure_get_int(structure, "height", &height_);

    // Update header information
    cinfo_msg_ = camera_info_manager_.getCameraInfo();

    const rclcpp::Time raw_stamp = use_gst_timestamps_ ?
      rclcpp::Time(GST_TIME_AS_NSECONDS(buf->pts + bt) + time_offset_, now().get_clock_type()) :
      now();
    cinfo_msg_.header.stamp = snap_to_frame_grid(raw_stamp);
    // RCLCPP_INFO(get_logger(), "Image time stamp: %.3f",cinfo->header.stamp.toSec());
    cinfo_msg_.header.frame_id = frame_id_;
    if (image_encoding_ == "jpeg") {
      comp_img_msg_.header = cinfo_msg_.header;
      comp_img_msg_.format = "jpeg";
      comp_img_msg_.data.resize(buf_size);
      std::copy(
        buf_data, (buf_data) + (buf_size),
        comp_img_msg_.data.begin());
    } else {
      // Complain if the returned buffer is smaller than we expect
      const unsigned int expected_frame_size =
        width_ * height_ * sensor_msgs::image_encodings::numChannels(image_encoding_);

      if (buf_size < expected_frame_size) {
        RCLCPP_WARN_STREAM(
          get_logger(), "GStreamer image buffer underflow: Expected frame to be " <<
            expected_frame_size << " bytes but got only " <<
            buf_size << " bytes. (make sure frames are correctly encoded)");
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

      std::copy(
        buf_data,
        (buf_data) + (buf_size),
        img_msg_.data.begin());
    }

    // Release the buffer
    if (buf) {
      gst_memory_unmap(memory, &info);
      gst_memory_unref(memory);
      gst_sample_unref(sample);
    }
  }
}

void GSCam::publish_stream()
{
  if (image_encoding_ == "jpeg") {
    jpeg_pub_->publish(comp_img_msg_);
    cinfo_pub_->publish(cinfo_msg_);
  } else {
    // Publish the image/info
    camera_pub_.publish(img_msg_, cinfo_msg_);
  }
  if (enable_rectifying_) {
    sensor_msgs::msg::Image img_rect_msg;
    try {
      // Match the rectified output encoding to the source encoding so that
      // whatever `image_encoding` is configured (e.g. bgr8) propagates through
      // to image_rect without an implicit channel-order mismatch.
      cv_bridge::CvImage img_bridge_rect =
        cv_bridge::CvImage(img_msg_.header, img_msg_.encoding);

      cv_bridge::CvImagePtr cv_img_raw = cv_bridge::toCvCopy(img_msg_, img_msg_.encoding);
      pinhole_model_.fromCameraInfo(cinfo_msg_);
      // INTER_NEAREST is cheaper than the default INTER_LINEAR and is adequate
      // for a downstream detection network.
      pinhole_model_.rectifyImage(cv_img_raw->image, img_bridge_rect.image, cv::INTER_NEAREST);

      img_bridge_rect.toImageMsg(img_rect_msg);
      camera_pub_rect_->publish(std::move(img_rect_msg));
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Runtime error, publish_rectified_image.");
    }
  }
}

void GSCam::cleanup_stream()
{
  // The grid phase is meaningless across a pipeline restart.
  if (grid_tracker_) {
    grid_tracker_->reset();
    grid_epoch_set_ = false;
  }

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
