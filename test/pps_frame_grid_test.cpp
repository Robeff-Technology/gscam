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

#include <cstdint>
#include <random>

#include "gscam/pps_frame_grid.hpp"

using gscam::PpsFrameGrid;

namespace
{

constexpr int64_t kSec = 1000000000LL;
constexpr int kFps = 30;
// An arbitrary epoch second, large enough to catch overflow in the arithmetic.
constexpr int64_t kEpoch = 1790000000LL * kSec;
constexpr int64_t kLatency = 233000000LL;

}  // namespace

TEST(PpsFrameGrid, SlotsMatchFormula)
{
  PpsFrameGrid grid(kFps, 0);
  EXPECT_EQ(kEpoch, grid.slot_time(kEpoch, 0));
  EXPECT_EQ(kEpoch + 33333333, grid.slot_time(kEpoch, 1));
  EXPECT_EQ(kEpoch + 500000000, grid.slot_time(kEpoch, 15));
  EXPECT_EQ(kEpoch + 966666667, grid.slot_time(kEpoch, 29));
  // Slot fps is exactly the next PPS edge, with no accumulated rounding.
  EXPECT_EQ(kEpoch + kSec, grid.slot_time(kEpoch, kFps));
}

TEST(PpsFrameGrid, JitteredArrivalsLandExactlyOnGrid)
{
  PpsFrameGrid grid(kFps, 0);
  std::mt19937 rng(42);
  // 5 ms of jitter, far beyond what the pipeline shows, is still well inside
  // the 16.7 ms half period.
  std::uniform_int_distribution<int64_t> jitter(-5000000, 5000000);

  for (int64_t s = 0; s < 3; ++s) {
    for (int64_t n = 0; n < kFps; ++n) {
      const int64_t truth = grid.slot_time(kEpoch + s * kSec, n);
      const auto r = grid.update(truth + kLatency + jitter(rng), kLatency);
      EXPECT_EQ(truth, r.stamp_ns);
      EXPECT_EQ(n, r.index);
      if (s + n > 0) {
        EXPECT_EQ(1, r.step);
      }
    }
  }
}

TEST(PpsFrameGrid, LastFrameAndSecondRolloverAreNotConfused)
{
  PpsFrameGrid grid(kFps, 0);
  // Arrival just before the PPS edge belongs to frame 0 of the next second.
  const auto r0 = grid.update(kEpoch + kSec - 2000000 + kLatency, kLatency);
  EXPECT_EQ(kEpoch + kSec, r0.stamp_ns);
  EXPECT_EQ(0, r0.index);
  EXPECT_NEAR(-0.002, r0.residual, 1e-9);

  // And one that is closer to frame 29 stays in the old second.
  PpsFrameGrid grid2(kFps, 0);
  const auto r29 = grid2.update(kEpoch + kSec - 30000000 + kLatency, kLatency);
  EXPECT_EQ(kEpoch + 966666667, r29.stamp_ns);
  EXPECT_EQ(29, r29.index);
}

TEST(PpsFrameGrid, PhaseShiftsTheWholeGrid)
{
  // A +25% port phase shift at 20 fps, as in the vendor's four-port layout.
  const int64_t phase = 12500000;
  PpsFrameGrid grid(20, phase);
  const auto r = grid.update(kEpoch + phase + 3 * 50000000 + kLatency + 1000000, kLatency);
  EXPECT_EQ(kEpoch + phase + 150000000, r.stamp_ns);
  EXPECT_EQ(3, r.index);
}

TEST(PpsFrameGrid, PhaseLongerThanAPeriodStillMapsToTheRightSecond)
{
  // A stamp offset can exceed one frame period; frame 0 of second s is then
  // still T_pps(s) + phase, not a frame of the previous second.
  const int64_t phase = 40000000;
  PpsFrameGrid grid(kFps, phase);
  const auto r = grid.update(kEpoch + phase + kLatency, kLatency);
  EXPECT_EQ(kEpoch + phase, r.stamp_ns);
  EXPECT_EQ(0, r.index);
}

TEST(PpsFrameGrid, DroppedFramesAndDuplicatesAreCounted)
{
  PpsFrameGrid grid(kFps, 0);
  grid.update(grid.slot_time(kEpoch, 28) + kLatency, kLatency);

  // Frames 29 and 0 dropped across the PPS edge.
  const auto r = grid.update(grid.slot_time(kEpoch + kSec, 1) + kLatency, kLatency);
  EXPECT_EQ(3, r.step);
  EXPECT_EQ(grid.slot_time(kEpoch + kSec, 1), r.stamp_ns);

  const auto dup = grid.update(grid.slot_time(kEpoch + kSec, 1) + kLatency + 1000000, kLatency);
  EXPECT_EQ(0, dup.step);
}

TEST(PpsFrameGrid, WrongLatencyShowsUpAsResidualMean)
{
  PpsFrameGrid grid(kFps, 0);
  // Configured latency 3 ms too short: stamps stay correct, residual reports it.
  const auto r = grid.update(grid.slot_time(kEpoch, 7) + kLatency, kLatency - 3000000);
  EXPECT_EQ(grid.slot_time(kEpoch, 7), r.stamp_ns);
  EXPECT_NEAR(0.003, r.residual, 1e-9);
}
