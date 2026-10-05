// Copyright 2026 Leo Drive
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

#ifndef GSCAM__FRAME_GRID_TRACKER_HPP_
#define GSCAM__FRAME_GRID_TRACKER_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace gscam
{

/// Recovers the hardware frame-sync grid from jittery host-side arrival times.
///
/// The NPL54 drives all four cameras from one MCU frame-sync generator, so the
/// true start-of-frame instants sit on a uniform grid (33.33 ms apart at 30
/// fps). What the host observes is that grid plus a mostly-constant pipeline
/// latency (sensor readout + DMA + IRQ + GStreamer) plus a small amount of
/// scheduling jitter. Because the jitter is orders of magnitude smaller than
/// the grid spacing, every arrival can be snapped back onto the grid without
/// ambiguity, which removes the jitter from the published timestamps.
///
/// Each sample is snapped independently rather than by incrementing a frame
/// counter: if frames are dropped, the next arrival simply lands several grid
/// steps later instead of silently shifting every subsequent timestamp.
///
/// The grid phase and period are tracked with an alpha-beta loop, so the
/// estimate follows the card's oscillator even when it free-runs (~50 ppm)
/// instead of being locked to an external PPS.
class FrameGridTracker
{
public:
  struct Result
  {
    /// Snapped grid position, in the same time base as the input.
    double stamp = 0.0;
    /// Measured minus predicted arrival, in seconds. Feed this to a histogram
    /// to check that the jitter really is small compared to the grid spacing.
    double residual = 0.0;
    /// Grid steps advanced since the previous accepted sample. 1 is nominal,
    /// >1 means that many frames were dropped, 0 means a duplicate arrival.
    int step = 0;
    /// The residual was too large to trust, so the loop state was not updated
    /// and `stamp` is the prediction rather than a measurement.
    bool outlier = false;
    /// The loop re-anchored on this sample; `stamp` is the raw arrival time.
    bool reset = false;
  };

  struct Options
  {
    /// Nominal frame rate the hardware is configured for.
    double fps = 30.0;
    /// Phase loop gain. Higher tracks faster, lower filters harder.
    double phase_gain = 0.05;
    /// Period (frequency) loop gain. Zero freezes the period at nominal.
    double period_gain = 5e-4;
    /// Residuals beyond this fraction of the grid spacing are rejected.
    double reset_threshold = 0.35;
    /// Gap after which the loop gives up and re-anchors, in seconds.
    double max_gap = 2.0;
    /// Consecutive rejected samples tolerated before re-anchoring.
    int max_consecutive_outliers = 5;
    /// Samples to observe before the period loop is allowed to move.
    int period_warmup = 100;
    /// Period is never allowed further than this fraction from nominal.
    double max_period_deviation = 0.01;
  };

  explicit FrameGridTracker(const Options & options)
  : opt_(options),
    nominal_period_(1.0 / options.fps),
    period_(1.0 / options.fps)
  {
  }

  /// Feeds one arrival time and returns the snapped estimate.
  Result update(double t)
  {
    if (!locked_) {
      return anchor(t);
    }

    const double dt = t - grid_;
    if (dt > opt_.max_gap || dt < -opt_.max_gap) {
      return anchor(t);
    }

    const int64_t k = static_cast<int64_t>(std::llround(dt / period_));
    if (k <= 0) {
      // Two arrivals landed on the same grid slot. Report the duplicate rather
      // than forcing the grid forward, which would corrupt the phase estimate.
      Result r;
      r.stamp = grid_;
      r.residual = dt;
      r.step = 0;
      return r;
    }

    const double predicted = grid_ + static_cast<double>(k) * period_;
    const double e = t - predicted;

    Result r;
    r.residual = e;
    r.step = static_cast<int>(k);

    if (std::fabs(e) > opt_.reset_threshold * period_) {
      if (++consecutive_outliers_ > opt_.max_consecutive_outliers) {
        return anchor(t);
      }
      // Hold the loop and publish the prediction: a single late arrival should
      // not drag the phase estimate with it.
      r.stamp = predicted;
      r.outlier = true;
      return r;
    }
    consecutive_outliers_ = 0;

    grid_ = predicted + opt_.phase_gain * e;
    if (++samples_ > opt_.period_warmup && opt_.period_gain > 0.0) {
      period_ += opt_.period_gain * e / static_cast<double>(k);
      const double lo = nominal_period_ * (1.0 - opt_.max_period_deviation);
      const double hi = nominal_period_ * (1.0 + opt_.max_period_deviation);
      period_ = std::min(std::max(period_, lo), hi);
    }

    r.stamp = grid_;
    return r;
  }

  /// Frame rate implied by the tracked grid period.
  double measured_fps() const {return 1.0 / period_;}

  /// Deviation of the tracked rate from nominal, in parts per million. Around
  /// -50 ppm is the card free-running on its local oscillator; near zero means
  /// the grid is holding the configured rate.
  double ppm() const {return (nominal_period_ / period_ - 1.0) * 1e6;}

  bool locked() const {return locked_;}

  void reset()
  {
    locked_ = false;
    samples_ = 0;
    consecutive_outliers_ = 0;
    period_ = nominal_period_;
  }

private:
  Result anchor(double t)
  {
    grid_ = t;
    locked_ = true;
    samples_ = 0;
    consecutive_outliers_ = 0;
    period_ = nominal_period_;

    Result r;
    r.stamp = t;
    r.reset = true;
    return r;
  }

  Options opt_;
  double nominal_period_;
  double period_;
  double grid_ = 0.0;
  bool locked_ = false;
  int samples_ = 0;
  int consecutive_outliers_ = 0;
};

}  // namespace gscam

#endif  // GSCAM__FRAME_GRID_TRACKER_HPP_
