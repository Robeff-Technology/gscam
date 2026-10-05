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

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "gscam/frame_grid_tracker.hpp"

using gscam::FrameGridTracker;

namespace
{

constexpr double kFps = 30.0;
constexpr double kPeriod = 1.0 / kFps;

FrameGridTracker::Options default_options()
{
  FrameGridTracker::Options opt;
  opt.fps = kFps;
  return opt;
}

}  // namespace

TEST(FrameGridTracker, FirstSampleAnchors)
{
  FrameGridTracker tracker(default_options());
  const auto r = tracker.update(12.345);
  EXPECT_TRUE(r.reset);
  EXPECT_DOUBLE_EQ(12.345, r.stamp);
  EXPECT_TRUE(tracker.locked());
}

TEST(FrameGridTracker, JitterIsRemovedFromTimestamps)
{
  FrameGridTracker tracker(default_options());
  std::mt19937 rng(1234);
  // 300 us of jitter is far more than the driver shows in practice and still
  // two orders of magnitude below the 33 ms grid spacing.
  std::normal_distribution<double> jitter(0.0, 300e-6);

  double worst = 0.0;
  double sum_sq = 0.0;
  int n_measured = 0;
  for (int n = 0; n < 2000; ++n) {
    const double truth = n * kPeriod;
    const auto r = tracker.update(truth + jitter(rng));
    if (n > 200) {
      const double err = r.stamp - truth;
      worst = std::max(worst, std::fabs(err));
      sum_sq += err * err;
      ++n_measured;
      EXPECT_EQ(1, r.step);
      EXPECT_FALSE(r.reset);
    }
  }

  // A first-order loop of gain a attenuates white input noise by
  // sqrt(a / (2 - a)), so 300 us in should come out near 48 us rms.
  const double rms = std::sqrt(sum_sq / n_measured);
  EXPECT_LT(rms, 80e-6);
  EXPECT_LT(worst, 250e-6);
}

TEST(FrameGridTracker, DroppedFramesDoNotShiftLaterTimestamps)
{
  FrameGridTracker tracker(default_options());
  for (int n = 0; n < 200; ++n) {
    tracker.update(n * kPeriod);
  }

  // The last frame accepted above was 199. Frames 200..203 never arrive, so
  // frame 204 lands five grid steps on, reporting four dropped frames.
  const auto r = tracker.update(204 * kPeriod);
  EXPECT_EQ(5, r.step);
  EXPECT_NEAR(204 * kPeriod, r.stamp, 1e-9);

  // And the grid is still intact afterwards.
  const auto next = tracker.update(205 * kPeriod);
  EXPECT_EQ(1, next.step);
  EXPECT_NEAR(205 * kPeriod, next.stamp, 1e-9);
}

TEST(FrameGridTracker, SingleLateArrivalDoesNotDragThePhase)
{
  FrameGridTracker tracker(default_options());
  for (int n = 0; n < 300; ++n) {
    tracker.update(n * kPeriod);
  }

  // One frame is delivered 15 ms late, e.g. the host scheduler stalled.
  const auto outlier = tracker.update(300 * kPeriod + 0.015);
  EXPECT_TRUE(outlier.outlier);
  EXPECT_NEAR(300 * kPeriod, outlier.stamp, 1e-6);

  const auto recovered = tracker.update(301 * kPeriod);
  EXPECT_FALSE(recovered.outlier);
  EXPECT_NEAR(301 * kPeriod, recovered.stamp, 1e-5);
}

TEST(FrameGridTracker, TracksAFreeRunningOscillator)
{
  FrameGridTracker tracker(default_options());
  // The card running 50 ppm slow on its local oscillator, as the vendor
  // describes for the un-locked case.
  const double actual_period = kPeriod * (1.0 + 50e-6);

  for (int n = 0; n < 20000; ++n) {
    tracker.update(n * actual_period);
  }

  EXPECT_NEAR(-50.0, tracker.ppm(), 10.0);
  EXPECT_NEAR(1.0 / actual_period, tracker.measured_fps(), 1e-3);
}

TEST(FrameGridTracker, ReAnchorsAfterALongGap)
{
  auto opt = default_options();
  opt.max_gap = 2.0;
  FrameGridTracker tracker(opt);

  for (int n = 0; n < 100; ++n) {
    tracker.update(n * kPeriod);
  }

  const auto r = tracker.update(100 * kPeriod + 5.0);
  EXPECT_TRUE(r.reset);
  EXPECT_DOUBLE_EQ(100 * kPeriod + 5.0, r.stamp);
}

TEST(FrameGridTracker, ReportsDuplicateArrivals)
{
  FrameGridTracker tracker(default_options());
  for (int n = 0; n < 100; ++n) {
    tracker.update(n * kPeriod);
  }

  // Same frame handed to us twice.
  const auto r = tracker.update(99 * kPeriod + 1e-5);
  EXPECT_EQ(0, r.step);
}

TEST(FrameGridTracker, SustainedDisruptionForcesReAnchor)
{
  auto opt = default_options();
  opt.max_consecutive_outliers = 5;
  FrameGridTracker tracker(opt);

  for (int n = 0; n < 200; ++n) {
    tracker.update(n * kPeriod);
  }

  // The grid jumps half a frame and stays there, e.g. the card restarted its
  // frame-sync generator. The loop must give up and re-anchor rather than
  // rejecting frames forever.
  bool saw_reset = false;
  for (int n = 200; n < 220 && !saw_reset; ++n) {
    saw_reset = tracker.update(n * kPeriod + 0.5 * kPeriod).reset;
  }
  EXPECT_TRUE(saw_reset);
}
