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
#include <chrono>
#include <cinttypes>
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
  grid_snap_pps_(false),
  pps_arrival_phase_(0.0),
  pps_frame_delay_(0),
  pps_period_(0.0),
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
  grid_stat_sin_(0.0),
  grid_stat_cos_(0.0),
  grid_stat_max_abs_(0.0),
  stop_signal_(false)
{
  if (!configure()) {
    RCLCPP_FATAL(get_logger(), "Failed to configure gscam!");
    return;
  }
  // Publishers must exist before the pipeline thread can deliver a frame.
  // Frames are published from that thread as they arrive, so no timer is
  // needed and publish never races with the frame being written.
  init_publishers();
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
  // Unused since frames are published on arrival; kept so existing launch
  // files that set it still load.
  camera_frame_rate_ = declare_parameter("camera_frame_rate", 15);
  enable_rectifying_ = declare_parameter("enable_rectifying", true);
  frame_timeout_ms_ = declare_parameter("frame_timeout_ms", 2000);
  reconnect_delay_ms_ = declare_parameter("reconnect_delay_ms", 1000);

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

  // "track" recovers the grid from arrival times; "pps" computes it as
  // T_pps + phase + n / fps, valid when the card's frame sync is PPS-locked.
  const std::string grid_mode = declare_parameter("grid_snap_mode", std::string("track"));
  const double pps_phase = declare_parameter("grid_snap_pps_phase", 0.0);
  pps_arrival_phase_ = declare_parameter("grid_snap_pps_arrival_phase", 0.0);
  pps_frame_delay_ = static_cast<int>(declare_parameter("grid_snap_pps_frame_delay", 0));
  if (grid_mode == "pps") {
    grid_snap_pps_ = true;
    const int pps_fps = static_cast<int>(std::lround(grid_opt.fps));
    if (std::fabs(grid_opt.fps - pps_fps) > 1e-6) {
      RCLCPP_FATAL(
        get_logger(), "grid_snap_mode pps needs a whole number of frames per second, got %.6f.",
        grid_opt.fps);
      return false;
    }
    pps_grid_ = std::make_unique<PpsFrameGrid>(
      pps_fps, static_cast<int64_t>(std::llround(pps_phase * 1e9)));
    pps_period_ = 1.0 / pps_fps;
  } else if (grid_mode != "track") {
    RCLCPP_FATAL(
      get_logger(), "Unknown grid_snap_mode '%s', expected 'track' or 'pps'.", grid_mode.c_str());
    return false;
  }

  if (grid_snap_enabled_) {
    if (grid_snap_pps_) {
      RCLCPP_INFO(
        get_logger(),
        "PPS frame grid enabled: t = T_pps + %.3f ms + n / %.0f, arrival latency "
        "%.3f ms phase + %d frames = %.3f ms.",
        pps_phase * 1e3, grid_opt.fps, pps_arrival_phase_.load() * 1e3,
        pps_frame_delay_.load(), pps_latency() * 1e3);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Frame-sync grid snapping enabled: %.3f fps grid, %.1f ms latency offset.",
        grid_opt.fps, grid_snap_latency_.load() * 1e3);
    }

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
  if (grid_snap_pps_) {
    return snap_to_pps_grid(raw);
  }
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

rclcpp::Time GSCam::snap_to_pps_grid(const rclcpp::Time & raw)
{
  const int64_t raw_ns = raw.nanoseconds();
  if (grid_stats_since_ns_ == 0) {
    grid_stats_since_ns_ = raw_ns;
  }

  const auto result = pps_grid_->update(
    raw_ns, static_cast<int64_t>(std::llround(pps_latency() * 1e9)));

  ++grid_stat_count_;
  if (result.step <= 0) {
    ++grid_stat_duplicates_;
  } else {
    grid_stat_dropped_ += static_cast<uint64_t>(result.step - 1);
    grid_stat_sum_ += result.residual;
    grid_stat_sum_sq_ += result.residual * result.residual;
    grid_stat_max_abs_ = std::max(grid_stat_max_abs_, std::fabs(result.residual));
    const double angle = 2.0 * M_PI * result.residual / pps_period_;
    grid_stat_sin_ += std::sin(angle);
    grid_stat_cos_ += std::cos(angle);
  }

  if (grid_diag_pub_) {
    // Same layout as track mode; the third field is the frame index within
    // the second instead of a measured rate.
    std_msgs::msg::Float64MultiArray diag;
    diag.data = {
      result.residual,
      static_cast<double>(result.step),
      static_cast<double>(result.index),
      0.0};
    grid_diag_pub_->publish(diag);
  }

  if (grid_snap_report_period_ > 0.0 &&
    raw_ns - grid_stats_since_ns_ >= static_cast<int64_t>(grid_snap_report_period_ * 1e9))
  {
    report_grid_stats();
    grid_stats_since_ns_ = raw_ns;
  }

  return rclcpp::Time(result.stamp_ns, raw.get_clock_type());
}

double GSCam::pps_latency() const
{
  return pps_arrival_phase_.load() + pps_frame_delay_.load() * pps_period_;
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
    } else if (parameter.get_name() == "grid_snap_pps_arrival_phase") {
      // Like the latency, both pps latency terms only choose which slot a
      // frame lands in, so they can change without disturbing anything.
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "grid_snap_pps_arrival_phase must be a double (seconds)";
        break;
      }
      pps_arrival_phase_ = parameter.as_double();
      RCLCPP_INFO(
        get_logger(), "grid_snap_pps_arrival_phase set to %.3f ms",
        pps_arrival_phase_.load() * 1e3);
    } else if (parameter.get_name() == "grid_snap_pps_frame_delay") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "grid_snap_pps_frame_delay must be an integer (frames)";
        break;
      }
      pps_frame_delay_ = static_cast<int>(parameter.as_int());
      RCLCPP_INFO(
        get_logger(), "grid_snap_pps_frame_delay set to %d frames", pps_frame_delay_.load());
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

  if (grid_snap_pps_) {
    // The circular mean residual is the error in the configured arrival phase,
    // so configured + mean is the phase this setup actually shows. Residuals
    // approaching half a frame period risk assigning frames to the wrong slot.
    const double circ_mean = std::atan2(grid_stat_sin_, grid_stat_cos_) * pps_period_ /
      (2.0 * M_PI);
    double measured_phase = std::fmod(pps_arrival_phase_.load() + circ_mean, pps_period_);
    if (measured_phase < 0.0) {
      measured_phase += pps_period_;
    }
    RCLCPP_INFO(
      get_logger(),
      "pps grid: %" PRIu64 " frames, residual mean %.3f ms / sd %.3f ms / max %.3f ms, "
      "dropped %" PRIu64 ", duplicates %" PRIu64 ", measured arrival phase %.3f ms",
      n, mean * 1e3, std::sqrt(var) * 1e3, grid_stat_max_abs_ * 1e3,
      grid_stat_dropped_, grid_stat_duplicates_, measured_phase * 1e3);
  } else {
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
  }

  grid_stat_count_ = 0;
  grid_stat_dropped_ = 0;
  grid_stat_outliers_ = 0;
  grid_stat_resets_ = 0;
  grid_stat_duplicates_ = 0;
  grid_stat_sum_ = 0.0;
  grid_stat_sum_sq_ = 0.0;
  grid_stat_sin_ = 0.0;
  grid_stat_cos_ = 0.0;
  grid_stat_max_abs_ = 0.0;
}

int64_t GSCam::gst_to_ros_time_offset_ns()
{
  GstClock * clock = gst_system_clock_obtain();
  const GstClockTime gst_now = gst_clock_get_time(clock);
  gst_object_unref(clock);
  return static_cast<int64_t>(now().nanoseconds()) -
         static_cast<int64_t>(GST_TIME_AS_NSECONDS(gst_now));
}

bool GSCam::log_bus_errors()
{
  if (!pipeline_) {
    return false;
  }
  bool had_error = false;
  GstBus * bus = gst_element_get_bus(pipeline_);
  GstMessage * msg;
  while ((msg = gst_bus_pop_filtered(
      bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING))) != NULL)
  {
    GError * err = NULL;
    gchar * debug = NULL;
    const bool is_error = GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR;
    if (is_error) {
      gst_message_parse_error(msg, &err, &debug);
      had_error = true;
    } else {
      gst_message_parse_warning(msg, &err, &debug);
    }
    const std::string text = "GStreamer " + std::string(is_error ? "error" : "warning") +
      " from " + GST_OBJECT_NAME(GST_MESSAGE_SRC(msg)) + ": " + (err ? err->message : "") +
      " (" + (debug ? debug : "no debug info") + ")";
    if (is_error) {
      RCLCPP_ERROR(get_logger(), "%s", text.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", text.c_str());
    }
    g_clear_error(&err);
    g_free(debug);
    gst_message_unref(msg);
  }
  gst_object_unref(bus);
  return had_error;
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
      pipeline_ = NULL;
      return false;
    }

    if (!gst_element_link(outelement, sink_)) {
      RCLCPP_FATAL(
        get_logger(), "GStreamer: cannot link outelement(\"%s\") -> sink\n",
        gst_element_get_name(outelement));
      gst_object_unref(outelement);
      gst_object_unref(pipeline_);
      pipeline_ = NULL;
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
      pipeline_ = NULL;
      return false;
    }
  }

  // Calibration between ros::Time and gst timestamps. Kept as a reference value
  // for logging only; the stamp of every frame re-reads this offset (see
  // gst_to_ros_time_offset_ns) so that drift between the two clocks cannot
  // accumulate into the published timestamps.
  time_offset_ = gst_to_ros_time_offset_ns();
  RCLCPP_INFO(get_logger(), "Time offset: %.6f", rclcpp::Time(time_offset_).seconds());

  gst_element_set_state(pipeline_, GST_STATE_PAUSED);

  if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_FATAL(get_logger(), "Failed to PAUSE stream, check your gstreamer configuration.");
    log_bus_errors();
    return false;
  } else {
    RCLCPP_DEBUG_STREAM(get_logger(), "Stream is PAUSED.");
  }

  return true;
}

void GSCam::init_publishers()
{
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
    "image_rect", qos);
}

void GSCam::update_stream()
{
  RCLCPP_INFO_STREAM(get_logger(), "Publishing stream...");
  stream_ended_ = false;

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
    // actual capture framerate of the device. A v4l2 source can stop
    // delivering frames without ever reaching EOS, so the wait is bounded and
    // a stalled pipeline is handed back to run() to be reopened.
    // RCLCPP_DEBUG(get_logger(), "Getting data...");
    const GstClockTime timeout = frame_timeout_ms_ > 0 ?
      static_cast<GstClockTime>(frame_timeout_ms_) * GST_MSECOND : GST_CLOCK_TIME_NONE;
    GstSample * sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink_), timeout);
    if (!sample) {
      if (gst_app_sink_is_eos(GST_APP_SINK(sink_))) {
        stream_ended_ = true;
        RCLCPP_ERROR(get_logger(), "Could not get gstreamer sample (end of stream).");
      } else {
        RCLCPP_ERROR(
          get_logger(), "No frame received for %d ms, restarting the stream.", frame_timeout_ms_);
      }
      break;
    }
    GstBuffer * buf = gst_sample_get_buffer(sample);
    // Stop on end of stream
    if (!buf) {
      RCLCPP_INFO(get_logger(), "Stream ended.");
      stream_ended_ = true;
      gst_sample_unref(sample);
      break;
    }
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

    // RCLCPP_DEBUG(get_logger(), "Got data.");

    // Get the image width and height
    GstCaps * caps = gst_sample_get_caps(sample);
    if (caps) {
      const GstStructure * structure = gst_caps_get_structure(caps, 0);
      gst_structure_get_int(structure, "width", &width_);
      gst_structure_get_int(structure, "height", &height_);
    }

    // Update header information
    cinfo_msg_ = camera_info_manager_.getCameraInfo();

    rclcpp::Time raw_stamp = now();
    if (use_gst_timestamps_) {
      // GstSystemClock is CLOCK_MONOTONIC while now() reads the ROS clock, and the
      // two do not advance at the same rate once the system clock is steered by
      // NTP/PTP. Re-anchoring per frame keeps the precision of buf->pts (the v4l2
      // dequeue instant) without inheriting the drift a startup-only offset would.
      const int64_t frame_time_offset = gst_to_ros_time_offset_ns();
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 10000, "Clock offset drift since startup: %.3f ms",
        static_cast<double>(frame_time_offset - static_cast<int64_t>(time_offset_)) / 1e6);
      raw_stamp = rclcpp::Time(
        static_cast<int64_t>(GST_TIME_AS_NSECONDS(buf->pts + bt)) + frame_time_offset,
        raw_stamp.get_clock_type());
    }
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
        RCLCPP_WARN_STREAM_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "GStreamer image buffer underflow: Expected frame to be " <<
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
        (buf_data) + std::min<gsize>(buf_size, expected_frame_size),
        img_msg_.data.begin());
    }

    // Release the buffer
    gst_memory_unmap(memory, &info);
    gst_memory_unref(memory);
    gst_sample_unref(sample);

    // Publish every frame exactly once, as it arrives. Publishing from a timer
    // instead duplicated or dropped frames whenever its rate did not match
    // the camera, and raced with this thread on the message buffers.
    publish_stream();
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
  // jpeg frames are never decoded, so there is no raw image to rectify
  if (enable_rectifying_ && image_encoding_ != "jpeg") {
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
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Failed to publish rectified image: %s", e.what());
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
  if (pps_grid_) {
    pps_grid_->reset();
  }

  // Clean up
  RCLCPP_INFO(get_logger(), "Stopping gstreamer pipeline...");
  // A source that fails posts an ERROR and then pushes EOS, so an "end of
  // stream" is only a real one when no error came with it.
  if (log_bus_errors()) {
    stream_ended_ = false;
  }
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = NULL;
  }
}

void GSCam::run()
{
  const auto wait_before_retry = [this]() {
      const auto until = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(reconnect_delay_ms_);
      while (!stop_signal_ && rclcpp::ok() && std::chrono::steady_clock::now() < until) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    };

  while (!stop_signal_ && rclcpp::ok()) {
    if (!this->init_stream()) {
      // The device can be briefly busy or not yet enumerated; retry instead of
      // shutting down the whole container.
      RCLCPP_ERROR(
        get_logger(), "Failed to initialize gscam stream, retrying in %d ms.",
        reconnect_delay_ms_);
      this->cleanup_stream();
      wait_before_retry();
      continue;
    }

    // Block while publishing
    this->update_stream();

    this->cleanup_stream();

    RCLCPP_INFO(get_logger(), "GStreamer stream stopped!");

    if (stop_signal_ || !rclcpp::ok()) {
      break;
    }
    // A stall (no frame within frame_timeout_ms) is always recovered from;
    // a real end of stream only when reopen_on_eof is set.
    if (reopen_on_eof_ || !stream_ended_) {
      RCLCPP_INFO(get_logger(), "Reopening stream...");
      wait_before_retry();
    } else {
      RCLCPP_INFO(get_logger(), "Cleaning up stream and exiting...");
      rclcpp::shutdown();
      break;
    }
  }
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
