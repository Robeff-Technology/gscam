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

#ifndef GSCAM__GSCAM_HPP_
#define GSCAM__GSCAM_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>
#include <stdexcept>
#include <string>
#include <thread>

extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
}

#include "rclcpp/rclcpp.hpp"
#include "image_geometry/pinhole_camera_model.h"


#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "gscam/frame_grid_tracker.hpp"
#include "gscam/pps_frame_grid.hpp"

namespace gscam
{

class GSCam : public rclcpp::Node
{
public:
  explicit GSCam(const rclcpp::NodeOptions & options);
  ~GSCam();

private:
  bool configure();
  void init_publishers();
  bool init_stream();
  // Log pending ERROR/WARNING bus messages; returns true if any was an error
  bool log_bus_errors();

  // Offset to add to a GStreamer clock timestamp to express it in ROS time.
  int64_t gst_to_ros_time_offset_ns();

  void update_stream();
  void publish_stream();
  void cleanup_stream();

  void run();

  /// Snaps a raw arrival time onto the hardware frame-sync grid. Returns the
  /// input unchanged when grid snapping is disabled.
  rclcpp::Time snap_to_frame_grid(const rclcpp::Time & raw);
  /// Places a raw arrival time at T_pps + phase + n / fps.
  rclcpp::Time snap_to_pps_grid(const rclcpp::Time & raw);
  /// Expected arrival delay after the PPS grid slot, in seconds.
  double pps_latency() const;
  void report_grid_stats();
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  // General gstreamer configuration
  std::string gsconfig_;

  // Gstreamer structures
  GstElement * pipeline_;
  GstElement * sink_;

  // Appsink configuration
  bool sync_sink_;
  bool preroll_;
  bool reopen_on_eof_;
  bool use_gst_timestamps_;

  // Camera publisher configuration
  std::string frame_id_;
  int width_, height_;
  std::string image_encoding_;
  std::string camera_name_;
  std::string camera_info_url_;
  bool use_sensor_data_qos_;
  int camera_frame_rate_;
  bool enable_rectifying_;
  // Restart the pipeline when no frame arrives within this time (0 disables)
  int frame_timeout_ms_;
  // Delay before retrying a pipeline that failed to start or stalled
  int reconnect_delay_ms_;
  // Set by update_stream() when the stream reached EOS rather than stalling
  bool stream_ended_ = false;

  // Frame-sync grid snapping (see frame_grid_tracker.hpp)
  bool grid_snap_enabled_;
  // Tunable at runtime while calibrating, and read from the pipeline thread.
  std::atomic<double> grid_snap_latency_;
  bool grid_snap_diagnostics_;
  double grid_snap_report_period_;
  std::unique_ptr<FrameGridTracker> grid_tracker_;
  // grid_snap_mode "pps": frames are placed at T_pps + phase + n / fps instead
  // of on a grid tracked from arrival times (see pps_frame_grid.hpp).
  bool grid_snap_pps_;
  std::unique_ptr<PpsFrameGrid> pps_grid_;
  // pps mode arrival latency = arrival_phase + frame_delay / fps. The phase is
  // measured (and logged) per setup; frame_delay is the whole frames the
  // phase measurement cannot see. Both are tunable at runtime.
  std::atomic<double> pps_arrival_phase_;
  std::atomic<int> pps_frame_delay_;
  double pps_period_;
  // Arrival times are tracked relative to this epoch so that a double keeps
  // sub-nanosecond resolution (absolute epoch nanoseconds would not).
  int64_t grid_epoch_ns_;
  bool grid_epoch_set_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr grid_diag_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr grid_param_cb_;
  int64_t grid_stats_since_ns_;
  uint64_t grid_stat_count_;
  uint64_t grid_stat_dropped_;
  uint64_t grid_stat_outliers_;
  uint64_t grid_stat_resets_;
  uint64_t grid_stat_duplicates_;
  double grid_stat_sum_;
  double grid_stat_sum_sq_;
  // Circular sums of the pps residual, so the measured arrival phase stays
  // meaningful when the configured one is off by up to half a period.
  double grid_stat_sin_;
  double grid_stat_cos_;
  double grid_stat_max_abs_;

  // Camera model
  image_geometry::PinholeCameraModel pinhole_model_;

  // ROS Inteface
  // Calibration between ros::Time and gst timestamps
  uint64_t time_offset_;
  camera_info_manager::CameraInfoManager camera_info_manager_;
  image_transport::CameraPublisher camera_pub_;

  // Camera message
  sensor_msgs::msg::Image img_msg_;
  sensor_msgs::msg::CompressedImage comp_img_msg_;

  // Camera Info message
  sensor_msgs::msg::CameraInfo cinfo_msg_;

  // Case of a jpeg only publisher
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr jpeg_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_pub_;

  // Rectifier publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_rect_;

  // Poll gstreamer on a separate thread
  std::thread pipeline_thread_;
  std::atomic<bool> stop_signal_;
};

}  // namespace gscam

#endif  // GSCAM__GSCAM_HPP_
