#!/usr/bin/env bash
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
#
# Fetches the Git LFS objects (robot/scene meshes) used by the sandbox
# scenarios into .git/lfs/objects. Nothing in the working tree changes:
# plan_scenario resolves LFS pointer files to the fetched objects itself.
#
#   planner_sandbox/tools/fetch_lfs.sh [extra --include patterns...]

set -euo pipefail
cd "$(dirname "$0")/../.."

if ! git lfs version >/dev/null 2>&1; then
  if command -v nix >/dev/null 2>&1; then
    PATH="$(nix build nixpkgs#git-lfs --no-link --print-out-paths)/bin:$PATH"
  else
    echo "git-lfs is required (brew install git-lfs)" >&2
    exit 1
  fi
fi

includes="third_party/universal_robots/**,intrinsic/models/**,intrinsic_control/intrinsic/models/**,third_party/kuka/**"
for extra in "$@"; do includes="$includes,$extra"; done
git lfs fetch --include="$includes"
