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

#ifndef GSCAM__PPS_FRAME_GRID_HPP_
#define GSCAM__PPS_FRAME_GRID_HPP_

#include <cmath>
#include <cstdint>

namespace gscam
{

/// Computes frame times from the PPS-locked frame-sync grid.
///
/// When the NPL54 runs in "PPS input and frame-sync" mode, the card restarts
/// its frame-sync generator on every PPS edge, so frame n of each second is
/// triggered at
///
///   t(n) = T_pps + phase + n / fps,    n = 0 .. fps-1
///
/// by construction. The host clock is disciplined by the same grandmaster that
/// drives the PPS, so T_pps is simply the whole second. Nothing is tracked or
/// filtered: the host arrival time is only used to decide which (second, n)
/// slot a frame belongs to, which is unambiguous as long as the arrival time
/// minus `latency` is within half a frame period of the true slot.
///
/// All arithmetic is done in integer nanoseconds so that the published
/// timestamps are exact and identical across cameras and nodes.
class PpsFrameGrid
{
public:
  struct Result
  {
    /// Grid slot time in nanoseconds, in the same time base as the input.
    int64_t stamp_ns = 0;
    /// Arrival minus latency minus `stamp_ns`, in seconds. Its spread is the
    /// pipeline jitter; its mean is the error in the configured latency.
    double residual = 0.0;
    /// Frame index within the second, 0 .. fps-1.
    int64_t index = 0;
    /// Grid slots advanced since the previous frame. 1 is nominal, >1 means
    /// that many frames were dropped, 0 means two arrivals hit the same slot.
    int64_t step = 0;
  };

  /// `fps` must divide one second into whole frames for the grid to restart
  /// cleanly on every PPS edge; `valid()` reports whether it does.
  /// `phase_ns` is the offset of frame 0 from the PPS edge (port phase shift
  /// plus wherever in the exposure the stamp should sit).
  PpsFrameGrid(int fps, int64_t phase_ns)
  : fps_(fps), phase_ns_(phase_ns) {}

  bool valid() const {return fps_ > 0;}

  /// Time of frame `index` within the second starting at `second_ns`.
  int64_t slot_time(int64_t second_ns, int64_t index) const
  {
    // Round each slot independently instead of accumulating a rounded period,
    // so 1/30 s never drifts and slot fps lands exactly on the next second.
    return second_ns + phase_ns_ + (index * kNsPerSec + fps_ / 2) / fps_;
  }

  /// Maps one arrival time (nanoseconds) onto the PPS grid. `latency_ns` is the
  /// expected arrival delay after the grid slot.
  Result update(int64_t arrival_ns, int64_t latency_ns)
  {
    const int64_t t = arrival_ns - latency_ns - phase_ns_;
    int64_t second = floor_div(t, kNsPerSec);
    const int64_t within = t - second * kNsPerSec;
    int64_t index = (within * fps_ + kNsPerSec / 2) / kNsPerSec;
    if (index >= fps_) {
      // Closer to frame 0 of the next second than to the last frame.
      index -= fps_;
      ++second;
    }
    second *= kNsPerSec;

    Result r;
    r.stamp_ns = slot_time(second, index);
    r.index = index;
    r.residual = static_cast<double>(arrival_ns - latency_ns - r.stamp_ns) * 1e-9;

    const int64_t slot = (second / kNsPerSec) * fps_ + index;
    r.step = has_last_ ? slot - last_slot_ : 1;
    last_slot_ = slot;
    has_last_ = true;
    return r;
  }

  void reset() {has_last_ = false;}

private:
  static constexpr int64_t kNsPerSec = 1000000000LL;

  static int64_t floor_div(int64_t a, int64_t b)
  {
    const int64_t q = a / b;
    return (a % b != 0 && (a < 0) != (b < 0)) ? q - 1 : q;
  }

  int64_t fps_;
  int64_t phase_ns_;
  int64_t last_slot_ = 0;
  bool has_last_ = false;
};

}  // namespace gscam

#endif  // GSCAM__PPS_FRAME_GRID_HPP_
