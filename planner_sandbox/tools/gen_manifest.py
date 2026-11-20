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
"""Generates the source manifest for the standalone planner sandbox.

The main intrinsic-core workspace spreads the `intrinsic/...` include tree over
several top-level directories (intrinsic_sdk/, intrinsic_kinematics/, ...) and
pulls in a large amount of infrastructure (gRPC, ROS 2, OpenTelemetry,
simulation, cloud APIs) that the motion planner itself does not need.

This script takes the real dependency graph of the planner (as produced by
`bazel query --output=xml` in the main workspace), cuts it at the points listed
in CUTS / STUBS below, and writes `gen/manifest.json`. The `intrinsic_src`
repository rule (bazel/intrinsic_src.bzl) turns that manifest into a single
unified `intrinsic/...` source tree made of symlinks into this checkout, plus
generated BUILD files.

Usage (from the repository root, i.e. the parent of planner_sandbox/):

  planner_sandbox/tools/gen_manifest.py            # runs bazel query itself
  planner_sandbox/tools/gen_manifest.py --xml deps.xml
"""

import argparse
import collections
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

SANDBOX = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_ROOT = os.path.dirname(SANDBOX)

# Targets (in main-workspace labels) whose closure makes up the sandbox.
ROOTS = [
    "//intrinsic/world/conversion/sdf:world_from_sdf",
    "//intrinsic/world:update_world",
    "//intrinsic_motion_planning/intrinsic/motion_planning/motion_planner:motion_planner_oss_factory",
    "//intrinsic/world:world",
    "//intrinsic/world:kinematic_world_views",
    "//intrinsic/world/objects:object_world",
    "//intrinsic_sdk/intrinsic/world/gzfile:gzfile",
    "//intrinsic_sdk/intrinsic/util/proto:parse_text_proto",
    "//intrinsic_motion_planning/intrinsic/motion_planning/proto:motion_planner_service_proto_utils",
]

# Dependency edges that are removed everywhere. Code that used them is either
# dead for in-process planning or patched (see PATCHES).
CUTS = {
    # ROS 2 / tf2: the time-indexed tf2 buffer in World/InnerWorld. Patched out
    # by no_ros_tf2.patch; tf2::Stamped itself is stubbed below.
    "@ros2_geometry2//:tf2_ros",
    "@ros2_rclcpp//:rclcpp",
    "@ros2_common_interfaces//:cpp_geometry_msgs",
    "//intrinsic/math:tf2_convert_intrinsic",
    # gRPC world client (and with it equipment/skills/connect libraries). Only
    # used by an ObjectWorldClient overload in compute_ik_util.
    # Patched out by no_object_world_client.patch.
    "//intrinsic_sdk/intrinsic/world/objects:object_world_client",
    # grpc::Status overload of GetExtendedStatus. Patched out by
    # no_grpc_status.patch.
    "@com_github_grpc_grpc//:grpc++",
}

# Targets replaced by local stubs. Stub files live under planner_sandbox/stubs/
# at the same path they have in the unified tree, so includes are unchanged.
# Stubs of external targets get a package in the unified tree.
STUBS = {
    # Tracing: OpenTelemetry/gRPC spans replaced by a timer logged at --v=1.
    "//intrinsic/stats:scoped_span": dict(
        kind="cc_library",
        hdrs=["intrinsic/stats/scoped_span.h"],
        srcs=["intrinsic/stats/scoped_span.cc"],
        deps=["@abseil-cpp//absl/log", "@abseil-cpp//absl/strings",
              "@abseil-cpp//absl/time"],
    ),
    # Rigid body dynamics via Robotics Library: only needed for torque-limited
    # TOPP. The stub reports Unimplemented if dynamics are requested.
    "//intrinsic_control/intrinsic/icon/dynamics:robotics_library_dynamics_creator": dict(
        kind="cc_library",
        hdrs=["intrinsic/icon/dynamics/robotics_library_dynamics_creator.h"],
        srcs=["intrinsic/icon/dynamics/robotics_library_dynamics_creator.cc"],
        deps=[
            "//intrinsic_control/intrinsic/icon/dynamics:rigid_body_interface",
            "//intrinsic_kinematics/intrinsic/kinematics:skeleton",
            "@abseil-cpp//absl/status",
            "@abseil-cpp//absl/status:statusor",
        ],
    ),
    # tf2::Stamped<T> is part of the TransformNode API; only that template is
    # needed once the tf2 buffer is gone.
    "@ros2_geometry2//:tf2": dict(
        kind="cc_library",
        package="tf2",
        hdrs=["tf2/include/tf2/transform_datatypes.h"],
    ),
}

# Patches (planner_sandbox/patches/*.patch, -p1 relative to the unified tree).
# Regenerate with tools/make_patches.py.
PATCHES = [
    "macos_portability.patch",
    "no_grpc_status.patch",
    "no_object_world_client.patch",
    "no_ros_tf2.patch",
]

# Rule attributes that hold label lists we follow / emit.
LABEL_LIST_ATTRS = ("srcs", "hdrs", "textual_hdrs", "deps",
                    "implementation_deps", "data")


def run_query(out_path):
  expr = " + ".join(ROOTS)
  cmd = ["bazel", "query", f"kind('rule', deps({expr}))", "--output=xml"]
  print("Running:", " ".join(cmd), file=sys.stderr)
  with open(out_path, "wb") as f:
    subprocess.run(cmd, cwd=REPO_ROOT, stdout=f, check=True)


def load_rules(xml_path):
  rules = {}
  for r in ET.parse(xml_path).getroot().findall("rule"):
    attrs = {}
    for el in r:
      n = el.get("name")
      if el.tag == "list":
        attrs[n] = [x.get("value") for x in el]
      elif el.tag in ("string", "label", "boolean", "int"):
        attrs[n] = el.get("value")
    rules[r.get("name")] = (r.get("class"), attrs)
  return rules


def is_internal(label):
  return label.startswith("//") or label.startswith("@intrinsic_apis//")


def split_label(label):
  if label.startswith("@intrinsic_apis//"):
    repo_prefix, rest = "intrinsic_apis/", label[len("@intrinsic_apis//"):]
  else:
    repo_prefix, rest = "", label[2:]
  pkg, _, name = rest.partition(":")
  return repo_prefix, pkg, name


def unify_pkg(pkg):
  """intrinsic_sdk/intrinsic/foo -> intrinsic/foo."""
  first, _, rest = pkg.partition("/")
  if first.startswith("intrinsic_") and rest.startswith("intrinsic"):
    return rest
  return pkg


def unify(label):
  if not is_internal(label):
    return label
  _, pkg, name = split_label(label)
  return f"//{unify_pkg(pkg)}:{name}"


def orig_file_path(label):
  repo_prefix, pkg, name = split_label(label)
  return os.path.join(repo_prefix + pkg, name)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--xml", help="Output of bazel query --output=xml.")
  args = ap.parse_args()
  xml_path = args.xml
  if not xml_path:
    xml_path = os.path.join(SANDBOX, "gen", "deps.xml")
    os.makedirs(os.path.dirname(xml_path), exist_ok=True)
    run_query(xml_path)
  rules = load_rules(xml_path)

  # Resolve aliases (the main workspace has many migration aliases).
  def resolve(label):
    seen = set()
    while (is_internal(label) and label in rules and
           rules[label][0] == "alias"):
      if label in seen:
        raise ValueError(f"alias cycle at {label}")
      seen.add(label)
      label = rules[label][1]["actual"]
    return label

  def rule_deps(label):
    if label in STUBS:
      return [resolve(d) for d in STUBS[label].get("deps", [])]
    _, attrs = rules[label]
    out = []
    for a in LABEL_LIST_ATTRS:
      out += [resolve(d) for d in attrs.get(a) or [] if d in rules]
    return out

  # Walk the internal closure, honoring cuts and stubs.
  closure, external = set(), set()
  todo = [resolve(r) for r in ROOTS]
  while todo:
    t = todo.pop()
    if t in closure or t in CUTS:
      continue
    if not is_internal(t) and t not in STUBS:
      external.add(t)
      continue
    closure.add(t)
    todo += rule_deps(t)

  files = {}  # unified path -> path relative to repo root (or stub marker)
  stub_files = {}  # unified path -> path relative to planner_sandbox/stubs
  packages = collections.defaultdict(list)
  cross_pkg_files = collections.defaultdict(set)

  def add_file(file_label, owner_pkg):
    upath = os.path.join(*filter(None, [unify(file_label)[2:].replace(":", "/")]))
    orig = orig_file_path(file_label)
    if upath in files and files[upath] != orig:
      raise ValueError(f"file collision: {upath}: {files[upath]} vs {orig}")
    files[upath] = orig
    fpkg, _, fname = unify(file_label)[2:].partition(":")
    if fpkg != owner_pkg:
      cross_pkg_files[fpkg].add(fname)
      return f"//{fpkg}:{fname}"
    return fname

  def emit_label(dep, owner_pkg):
    dep = resolve(dep)
    if dep in STUBS and not is_internal(dep):
      # External target stubbed inside the unified tree.
      return f"//{STUBS[dep]['package']}:{dep.split(':')[-1]}"
    if not is_internal(dep):
      return dep
    u = unify(dep)
    pkg, _, name = u[2:].partition(":")
    return f":{name}" if pkg == owner_pkg else u

  stubbed_external = {s for s in STUBS if not is_internal(s)}
  for t in sorted(closure | (stubbed_external & external)):
    if t in STUBS:
      spec = STUBS[t]
      if is_internal(t):
        upkg, name = unify(t)[2:].split(":")
      else:
        upkg, name = spec["package"], t.split(":")[-1]
      entry = dict(kind=spec["kind"], name=name)
      for a in ("srcs", "hdrs"):
        vals = []
        for p in spec.get(a, []):
          stub_files[p] = p
          vals.append(os.path.relpath(p, upkg))
        if vals:
          entry[a] = vals
      entry["deps"] = sorted({emit_label(d, upkg)
                              for d in spec.get("deps", []) if d not in CUTS})
      packages[upkg].append(entry)
      continue

    cls, attrs = rules[t]
    if cls in ("alias", "_license"):
      continue
    upkg, name = unify(t)[2:].split(":")
    entry = dict(kind=cls, name=name)
    if cls in ("cc_library", "proto_library", "cc_proto_library", "filegroup"):
      for a in LABEL_LIST_ATTRS:
        vals = []
        for v in attrs.get(a) or []:
          rv = resolve(v)
          if rv in CUTS:
            continue
          if v in rules or not is_internal(v):
            vals.append(emit_label(v, upkg))
          else:
            vals.append(add_file(v, upkg))
        if vals:
          entry[a] = sorted(set(vals)) if a in ("deps", "implementation_deps") else vals
      for a in ("copts", "defines", "local_defines", "linkopts"):
        if attrs.get(a):
          entry[a] = list(dict.fromkeys(attrs[a]))
      if attrs.get("alwayslink") == "true":
        entry["alwayslink"] = True
    else:
      raise ValueError(f"unsupported rule class {cls} for {t}")
    packages[upkg].append(entry)

  # Stub files override the originals at the same unified path.
  for p in stub_files:
    files.pop(p, None)

  ext_repos = sorted({e.split("//")[0] for e in external
                      if e not in STUBS})
  manifest = dict(
      files=dict(sorted(files.items())),
      stub_files=sorted(stub_files),
      patches=PATCHES,
      packages={
          p: dict(targets=sorted(ts, key=lambda e: e["name"]),
                  exports=sorted(cross_pkg_files.get(p, ())))
          for p, ts in sorted(packages.items())
      },
      external_repos=ext_repos,
  )
  # Packages that only exist because other packages reference their files.
  for p, fs in cross_pkg_files.items():
    manifest["packages"].setdefault(p, dict(targets=[], exports=sorted(fs)))

  out = os.path.join(SANDBOX, "gen", "manifest.json")
  os.makedirs(os.path.dirname(out), exist_ok=True)
  with open(out, "w") as f:
    json.dump(manifest, f, indent=1, sort_keys=True)
    f.write("\n")
  n_cc = sum(1 for p in packages.values() for e in p if e["kind"] == "cc_library")
  print(f"{len(closure)} targets ({n_cc} cc_library) in {len(packages)} "
        f"packages, {len(files)} source files", file=sys.stderr)
  print("external repos:", " ".join(ext_repos), file=sys.stderr)


if __name__ == "__main__":
  main()
