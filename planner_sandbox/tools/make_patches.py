#!/usr/bin/env python3
# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Regenerates planner_sandbox/patches/*.patch from scripted edits.

Each patch removes a dependency that the in-process planner does not need. The
edits are expressed as (old, new) string replacements against the current
sources so the patches can be regenerated after upstream changes:

  planner_sandbox/tools/make_patches.py
"""

import difflib
import os
import re

SANDBOX = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_ROOT = os.path.dirname(SANDBOX)

# unified path -> path in this checkout
SRC = {
    "intrinsic/world/world.cc": "intrinsic/world/world.cc",
    "intrinsic/world/internal/inner_world.h": "intrinsic/world/internal/inner_world.h",
    "intrinsic/world/internal/inner_world.cc": "intrinsic/world/internal/inner_world.cc",
    "intrinsic/kinematics/utils/compute_ik_util.h": "intrinsic_kinematics/intrinsic/kinematics/utils/compute_ik_util.h",
    "intrinsic/kinematics/utils/compute_ik_util.cc": "intrinsic_kinematics/intrinsic/kinematics/utils/compute_ik_util.cc",
    "intrinsic/util/status/get_extended_status.h": "intrinsic_sdk/intrinsic/util/status/get_extended_status.h",
    "intrinsic/util/status/get_extended_status.cc": "intrinsic_sdk/intrinsic/util/status/get_extended_status.cc",
    "intrinsic/icon/utils/log_internal.cc": "intrinsic_control/intrinsic/icon/utils/log_internal.cc",
    "intrinsic/icon/utils/realtime_log_sink.cc": "intrinsic_control/intrinsic/icon/utils/realtime_log_sink.cc",
    "intrinsic/icon/interprocess/binary_futex.cc": "intrinsic_control/intrinsic/icon/interprocess/binary_futex.cc",
    "intrinsic/util/path_resolver/path_resolver.cc": "intrinsic_sdk/intrinsic/util/path_resolver/path_resolver.cc",
    "intrinsic/world/gzfile/gzfile.cc": "intrinsic_sdk/intrinsic/world/gzfile/gzfile.cc",
}


def drop_lines(text, *needles):
  out = [l for l in text.splitlines(keepends=True)
         if not any(n in l for n in needles)]
  return "".join(out)


def replace_function(text, signature_start, new_body):
  """Replaces a top-level function definition starting at signature_start."""
  i = text.index(signature_start)
  j = text.index("{", i)
  depth = 0
  for k in range(j, len(text)):
    if text[k] == "{":
      depth += 1
    elif text[k] == "}":
      depth -= 1
      if depth == 0:
        return text[:j] + new_body + text[k + 1:]
  raise ValueError(signature_start)


def remove_function(text, signature_start, is_decl=False):
  i = text.index(signature_start)
  # Include the preceding comment block.
  lines_before = text[:i].splitlines(keepends=True)
  while lines_before and lines_before[-1].lstrip().startswith("//"):
    lines_before.pop()
  start = len("".join(lines_before))
  if is_decl:
    end = text.index(";", i) + 1
  else:
    j = text.index("{", i)
    depth = 0
    for k in range(j, len(text)):
      if text[k] == "{":
        depth += 1
      elif text[k] == "}":
        depth -= 1
        if depth == 0:
          end = k + 1
          break
  while end < len(text) and text[end] == "\n":
    end += 1
  return text[:start] + text[end:]


def no_ros_tf2(f):
  """World keeps tf2::Stamped (stubbed) but drops the ROS tf2 buffer."""
  s = f["intrinsic/world/world.cc"]
  s = drop_lines(s, '#include "geometry_msgs/msg/transform_stamped.hpp"',
                 '#include "intrinsic/math/tf2_convert_intrinsic.h"',
                 '#include "rclcpp/rclcpp.hpp"',
                 '#include "tf2/include/tf2/exceptions.h"',
                 '#include "tf2_ros/buffer_interface.h"')
  s = replace_function(
      s, "absl::Status World::GetTransform(AttachmentEntityId a_handle,\n"
      "                                 AttachmentEntityId b_handle, absl::Time t,",
      """{
  // planner_sandbox: the time-indexed tf2 buffer (ROS 2) is not built.
  if (t != absl::UnixEpoch()) {
    return absl::UnimplementedError(
        "Time-indexed transform lookups require tf2 (not in planner_sandbox)");
  }
  output.setData(this->GetTransform(a_handle, b_handle));
  return absl::OkStatus();
}""")
  s = replace_function(
      s, "absl::Status World::UpdateTransformBuffer(",
      """{
  // planner_sandbox: the time-indexed tf2 buffer (ROS 2) is not built.
  return absl::UnimplementedError(
      "UpdateTransformBuffer requires tf2 (not in planner_sandbox)");
}""")
  f["intrinsic/world/world.cc"] = s

  h = f["intrinsic/world/internal/inner_world.h"]
  h = drop_lines(h, '#include "tf2/include/tf2/buffer_core.h"')
  h = h.replace("""  // Get the tf2 Buffer associated with this world
  tf2::BufferCore& GetBufferCore();

""", "")
  h = h.replace("""
  // Storage for tf2 transform data
  ::tf2::BufferCore tf2_buffer_;
""", "")
  f["intrinsic/world/internal/inner_world.h"] = h

  c = f["intrinsic/world/internal/inner_world.cc"]
  c = drop_lines(c, '#include "tf2/include/tf2/buffer_core.h"',
                 "tf2::BufferCore& InnerWorld::GetBufferCore()")
  f["intrinsic/world/internal/inner_world.cc"] = c


def no_object_world_client(f):
  """Drops the gRPC ObjectWorldClient overload from compute_ik_util."""
  for p, is_decl in (("intrinsic/kinematics/utils/compute_ik_util.h", True),
                     ("intrinsic/kinematics/utils/compute_ik_util.cc", False)):
    s = f[p]
    s = drop_lines(s, '#include "intrinsic/world/objects/object_world_client.h"')
    s = remove_function(s, "absl::StatusOr<Pose3d> CartesianMotionTargetToPose3d(\n"
                        "    const world::ObjectWorldClient& object_world,",
                        is_decl=is_decl)
    f[p] = s


def no_grpc_status(f):
  """Drops the grpc::Status overload of GetExtendedStatus."""
  h = f["intrinsic/util/status/get_extended_status.h"]
  h = drop_lines(h, '#include "grpcpp/support/status.h"')
  h = remove_function(h, "std::optional<intrinsic_proto::status::ExtendedStatus> GetExtendedStatus(\n"
                      "    const grpc::Status& status);", is_decl=True)
  f["intrinsic/util/status/get_extended_status.h"] = h
  c = f["intrinsic/util/status/get_extended_status.cc"]
  c = drop_lines(c, '#include "grpcpp/support/status.h"')
  c = remove_function(c, "std::optional<intrinsic_proto::status::ExtendedStatus> GetExtendedStatus(\n"
                      "    const grpc::Status& status) {")
  f["intrinsic/util/status/get_extended_status.cc"] = c


def sub(text, old, new):
  if old not in text:
    raise ValueError(f"not found: {old!r}")
  return text.replace(old, new, 1)


def macos_portability(f):
  """Linux-only APIs behind #ifdef, with macOS equivalents."""
  p = "intrinsic/icon/utils/log_internal.cc"
  f[p] = sub(f[p], "#include <bits/time.h>\n", "#include <time.h>\n")

  p = "intrinsic/icon/utils/realtime_log_sink.cc"
  f[p] = sub(f[p], """    pthread_setname_np(reader_thread_.native_handle(), "GlobalLogSink");
""", """#if !defined(__APPLE__)  // macOS can only name the calling thread.
    pthread_setname_np(reader_thread_.native_handle(), "GlobalLogSink");
#endif
""")

  p = "intrinsic/icon/interprocess/binary_futex.cc"
  f[p] = sub(f[p], """#include <linux/futex.h>
#include <sys/syscall.h>
""", """#if defined(__APPLE__)
#include <os/os_sync_wait_on_address.h>
#else
#include <linux/futex.h>
#include <sys/syscall.h>
#endif
""")
  f[p] = sub(f[p], """inline int64_t futex(std::atomic<uint32_t>* uaddr, int futex_op, uint32_t val,""",
             """#if defined(__APPLE__)
// Minimal emulation of the two futex operations used below on top of macOS'
// os_sync_wait_on_address (macOS 14.4+). `timeout` is an absolute
// CLOCK_REALTIME deadline, as for FUTEX_WAIT_BITSET | FUTEX_CLOCK_REALTIME.
constexpr int FUTEX_WAKE = 1;
constexpr int FUTEX_WAIT_BITSET = 9;
constexpr int FUTEX_CLOCK_REALTIME = 256;

inline int64_t futex(std::atomic<uint32_t>* uaddr, int futex_op, uint32_t val,
                     bool private_futex,
                     const struct timespec* timeout = nullptr) {
  void* addr = static_cast<void*>(uaddr);
  if ((futex_op & ~FUTEX_CLOCK_REALTIME) == FUTEX_WAKE) {
    const os_sync_wake_by_address_flags_t flags =
        private_futex ? OS_SYNC_WAKE_BY_ADDRESS_NONE
                      : OS_SYNC_WAKE_BY_ADDRESS_SHARED;
    int ret = val == 1 ? os_sync_wake_by_address_any(addr, sizeof(*uaddr), flags)
                       : os_sync_wake_by_address_all(addr, sizeof(*uaddr), flags);
    // No waiter is not an error for futex(FUTEX_WAKE).
    return (ret == -1 && errno == ENOENT) ? 0 : ret;
  }
  const os_sync_wait_on_address_flags_t flags =
      private_futex ? OS_SYNC_WAIT_ON_ADDRESS_NONE
                    : OS_SYNC_WAIT_ON_ADDRESS_SHARED;
  if (timeout == nullptr) {
    return os_sync_wait_on_address(addr, val, sizeof(*uaddr), flags);
  }
  const absl::Duration remaining =
      absl::TimeFromTimespec(*timeout) - absl::Now();
  if (remaining <= absl::ZeroDuration()) {
    errno = ETIMEDOUT;
    return -1;
  }
  return os_sync_wait_on_address_with_timeout(
      addr, val, sizeof(*uaddr), flags, OS_CLOCK_MACH_ABSOLUTE_TIME,
      static_cast<uint64_t>(absl::ToInt64Nanoseconds(remaining)));
}
#else
inline int64_t futex(std::atomic<uint32_t>* uaddr, int futex_op, uint32_t val,""")
  f[p] = sub(f[p], """  return syscall(SYS_futex, uaddr, futex_op, val, timeout, uaddr2,
                 FUTEX_BITSET_MATCH_ANY);
}
""", """  return syscall(SYS_futex, uaddr, futex_op, val, timeout, uaddr2,
                 FUTEX_BITSET_MATCH_ANY);
}
#endif
""")

  p = "intrinsic/util/path_resolver/path_resolver.cc"
  f[p] = sub(f[p], """namespace intrinsic {

using bazel::tools::cpp::runfiles::Runfiles;
""", """#if defined(__APPLE__)
#include <mach-o/dyld.h>

namespace {
// glibc's program_invocation_name: the path this binary was invoked with.
const char* const program_invocation_name = [] {
  static char path[4096];
  uint32_t size = sizeof(path);
  return _NSGetExecutablePath(path, &size) == 0 ? path : getprogname();
}();
}  // namespace
#endif

namespace intrinsic {

using bazel::tools::cpp::runfiles::Runfiles;
""")

  p = "intrinsic/world/gzfile/gzfile.cc"
  f[p] = sub(f[p], """#include "zipconf.h"
""", """#include "zipconf.h"

#ifndef ENOKEY  // Linux-only errno.
#define ENOKEY EACCES
#endif
""")


PATCHES = {
    "macos_portability.patch": macos_portability,
    "no_ros_tf2.patch": no_ros_tf2,
    "no_object_world_client.patch": no_object_world_client,
    "no_grpc_status.patch": no_grpc_status,
}


def main():
  os.makedirs(os.path.join(SANDBOX, "patches"), exist_ok=True)
  for name, fn in PATCHES.items():
    orig = {u: open(os.path.join(REPO_ROOT, p)).read() for u, p in SRC.items()}
    new = dict(orig)
    fn(new)
    diff = []
    for u in SRC:
      if new[u] != orig[u]:
        diff += difflib.unified_diff(
            orig[u].splitlines(keepends=True), new[u].splitlines(keepends=True),
            fromfile=f"a/{u}", tofile=f"b/{u}")
    with open(os.path.join(SANDBOX, "patches", name), "w") as out:
      out.writelines(diff)
    print(name, sum(1 for l in diff if l[:1] in "+-") , "changed lines")


if __name__ == "__main__":
  main()
