// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// planner_sandbox stub: the subset of ROS 2 tf2/transform_datatypes.h used by
// the world API (tf2::Stamped<T>), without depending on ROS 2.

#ifndef TF2__TRANSFORM_DATATYPES_H_
#define TF2__TRANSFORM_DATATYPES_H_

#include <chrono>
#include <string>

namespace tf2 {

using Duration = std::chrono::nanoseconds;
using TimePoint = std::chrono::time_point<std::chrono::system_clock, Duration>;

template <typename T>
class Stamped : public T {
 public:
  TimePoint stamp_;
  std::string frame_id_;

  Stamped() = default;
  Stamped(const T& input, const TimePoint& timestamp,
          const std::string& frame_id)
      : T(input), stamp_(timestamp), frame_id_(frame_id) {}

  void setData(const T& input) { *static_cast<T*>(this) = input; }
};

}  // namespace tf2

#endif  // TF2__TRANSFORM_DATATYPES_H_
