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

// planner_sandbox stub: Robotics Library dynamics are not built. Torque-limited
// trajectory parameterization therefore returns UnimplementedError.

#ifndef INTRINSIC_ICON_DYNAMICS_ROBOTICS_LIBRARY_DYNAMICS_CREATOR_H_
#define INTRINSIC_ICON_DYNAMICS_ROBOTICS_LIBRARY_DYNAMICS_CREATOR_H_

#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/kinematics/skeleton.h"

namespace intrinsic::icon {
absl::StatusOr<std::unique_ptr<icon::RigidBodyInterface>>
CreateRoboticsLibraryDynamics(
    std::unique_ptr<intrinsic::kinematics::Skeleton> model);
}

#endif  // INTRINSIC_ICON_DYNAMICS_ROBOTICS_LIBRARY_DYNAMICS_CREATOR_H_
