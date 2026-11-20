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

#include "intrinsic/icon/dynamics/robotics_library_dynamics_creator.h"

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/kinematics/skeleton.h"

namespace intrinsic::icon {

absl::StatusOr<std::unique_ptr<icon::RigidBodyInterface>>
CreateRoboticsLibraryDynamics(
    std::unique_ptr<intrinsic::kinematics::Skeleton> model) {
  return absl::UnimplementedError(
      "Rigid body dynamics (Robotics Library) are not available in "
      "planner_sandbox; torque limits are not supported.");
}

}  // namespace intrinsic::icon
