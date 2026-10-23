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

// planner_sandbox stub: replaces the OpenTelemetry/gRPC backed ScopedSpan with
// a wall-clock timer. Span durations are logged at VLOG(1).

#ifndef INTRINSIC_STATS_SCOPED_SPAN_H_
#define INTRINSIC_STATS_SCOPED_SPAN_H_

#include <string>

#include "absl/strings/string_view.h"
#include "absl/time/time.h"

namespace intrinsic::stats {

class ScopedSpan {
 public:
  explicit ScopedSpan(absl::string_view name);
  ~ScopedSpan();

  // Attributes are dropped.
  template <typename T>
  void AddAttribute(absl::string_view key, const T& value) const {}

  ScopedSpan(const ScopedSpan&) = delete;
  ScopedSpan& operator=(const ScopedSpan&) = delete;

 private:
  std::string name_;
  absl::Time start_;
};

}  // namespace intrinsic::stats

#endif  // INTRINSIC_STATS_SCOPED_SPAN_H_
