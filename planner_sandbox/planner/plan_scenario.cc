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

// Runs the Intrinsic motion planner in-process on a scenario file and writes
// the scene geometry plus the planned motions for the viser viewer
// (viz/view.py).
//
//   bazel run //planner:plan_scenario -- \
//       --scenario=scenarios/ur5e_joint_move.textproto --out=/tmp/ur5e

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/flags/usage.h"
#include "absl/hash/hash.h"
#include "absl/log/globals.h"
#include "absl/log/initialize.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/strings/strip.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "google/protobuf/text_format.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/geometry/api/renderable_generation.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/motion_planner/motion_planner.h"
#include "intrinsic/motion_planning/motion_planner/motion_planner_flags.h"
#include "intrinsic/motion_planning/proto/motion_planner_service_proto_utils.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/conversion/sdf/world_from_sdf.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/geometry_types.h"
#include "intrinsic/world/gzfile/gzfile.h"
#include "intrinsic/world/objects/frame_internal.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/proto/world_updates.pb.h"
#include "intrinsic/world/update_world.h"
#include "intrinsic/world/world.h"
#include "nlohmann/json.hpp"
#include "planner/scenario.pb.h"

ABSL_FLAG(std::string, scenario, "", "Scenario text proto (planner_sandbox.Scenario).");
ABSL_FLAG(std::string, out, "",
          "Output directory for the viewer (result.json + meshes/). "
          "Nothing is written if empty.");
ABSL_FLAG(std::vector<std::string>, asset_root, {},
          "Additional directories to resolve model:// URIs against. The "
          "intrinsic-core checkout is always searched.");
ABSL_FLAG(std::string, intrinsic_root, "",
          "intrinsic-core checkout. Defaults to the parent of the Bazel "
          "workspace when started with `bazel run`.");
ABSL_FLAG(double, viz_dt, 1.0 / 30.0,
          "Sampling period [s] of the exported animation frames.");
ABSL_FLAG(bool, export_collision_geometry, true,
          "Also export collision geometry (toggleable in the viewer).");

// Planner flags. Defaults follow the production motion planner service
// (intrinsic/motion_planning/service/motion_planner_service_prod_config.textproto).
ABSL_FLAG(bool, path_refinement_validation, true,
          "MotionPlannerFlags.enable_path_refinement_validation_step");
ABSL_FLAG(bool, fallback_trajectory, true,
          "MotionPlannerFlags.enable_fallback_trajectory");
ABSL_FLAG(int, collision_threads, 4,
          "Threads for concurrent collision checking (0 disables it).");

namespace planner_sandbox {
namespace {

namespace fs = std::filesystem;
using ::intrinsic::Pose3d;
using ::intrinsic::World;
using ::intrinsic::eigenmath::VectorXd;
using ::nlohmann::json;

// `bazel run` changes the working directory; resolve user paths against the
// directory the command was started from.
fs::path UserPath(const std::string& p) {
  fs::path path(p);
  if (path.is_relative()) {
    if (const char* wd = std::getenv("BUILD_WORKING_DIRECTORY")) {
      return fs::path(wd) / path;
    }
  }
  return path;
}

absl::StatusOr<std::string> ReadFile(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return absl::NotFoundError(absl::StrCat("Cannot read ", path.string()));
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

template <typename Proto>
absl::StatusOr<Proto> ReadTextProto(const fs::path& path) {
  INTR_ASSIGN_OR_RETURN(std::string text, ReadFile(path));
  Proto proto;
  if (!google::protobuf::TextFormat::ParseFromString(text, &proto)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Failed to parse ", path.string(), " as ", Proto::descriptor()->full_name()));
  }
  return proto;
}

fs::path CheckoutRoot() {
  if (!absl::GetFlag(FLAGS_intrinsic_root).empty()) {
    return UserPath(absl::GetFlag(FLAGS_intrinsic_root));
  }
  if (const char* ws = std::getenv("BUILD_WORKSPACE_DIRECTORY")) {
    return fs::path(ws).parent_path();
  }
  return {};
}

// Meshes in intrinsic-core are stored with Git LFS. If `path` is an LFS
// pointer file whose object has been fetched (`git lfs fetch`), returns a path
// with the same file name that links to the object, so the working tree does
// not need an LFS checkout.
std::string MaybeResolveLfsPointer(const fs::path& path) {
  std::ifstream in(path);
  std::string line;
  if (!std::getline(in, line) || !absl::StartsWith(line, "version https://git-lfs")) {
    return path.string();
  }
  std::string oid;
  while (std::getline(in, line)) {
    absl::string_view v = line;
    if (absl::ConsumePrefix(&v, "oid sha256:")) oid = std::string(v);
  }
  const fs::path checkout = CheckoutRoot();
  const fs::path object =
      checkout / ".git/lfs/objects" / oid.substr(0, 2) / oid.substr(2, 2) / oid;
  if (oid.size() < 4 || !fs::exists(object)) {
    LOG(WARNING) << path.string() << " is a Git LFS pointer, but its object is "
                 << "not available. Run planner_sandbox/tools/fetch_lfs.sh.";
    return path.string();
  }
  const fs::path link =
      fs::temp_directory_path() / "planner_sandbox_lfs" / oid / path.filename();
  std::error_code ec;
  fs::create_directories(link.parent_path(), ec);
  if (!fs::exists(link)) fs::create_symlink(object, link, ec);
  return link.string();
}

std::vector<fs::path> AssetRoots(const fs::path& scenario_dir) {
  std::vector<fs::path> roots = {scenario_dir};
  for (const std::string& r : absl::GetFlag(FLAGS_asset_root)) {
    roots.push_back(UserPath(r));
  }
  if (const fs::path checkout = CheckoutRoot(); !checkout.empty()) {
    // The `intrinsic/...` tree is split over several top-level directories.
    roots.push_back(checkout);
    for (const auto& entry : fs::directory_iterator(checkout)) {
      if (entry.is_directory() &&
          absl::StartsWith(entry.path().filename().string(), "intrinsic_")) {
        roots.push_back(entry.path());
      }
    }
  }
  return roots;
}

// Resolves model:// and plain file URIs used in SDF files.
intrinsic::sdf::UriResolver MakeUriResolver(std::vector<fs::path> roots) {
  return [roots = std::move(roots)](
             const std::string& uri) -> absl::StatusOr<std::string> {
    absl::string_view path = uri;
    if (!absl::ConsumePrefix(&path, "model://")) {
      absl::ConsumePrefix(&path, "file://");
    }
    std::string p(path);
    // Built-in Gazebo models.
    if (p == "sun" || p == "ground_plane") p = "intrinsic/models/gazebo/" + p;
    if (fs::path(p).is_absolute() && fs::exists(p)) {
      return MaybeResolveLfsPointer(p);
    }
    for (const fs::path& root : roots) {
      if (fs::path candidate = root / p; fs::exists(candidate)) {
        return fs::is_regular_file(candidate) ? MaybeResolveLfsPointer(candidate)
                                              : candidate.string();
      }
    }
    return absl::NotFoundError(absl::StrCat(
        "Cannot resolve '", uri, "'. Searched: ",
        absl::StrJoin(roots, ", ", [](std::string* out, const fs::path& r) {
          absl::StrAppend(out, r.string());
        }),
        ". Pass --asset_root or --intrinsic_root."));
  };
}

absl::StatusOr<World> LoadScene(const Scene& scene, const fs::path& dir) {
  World world = World::CreateEmptyWorld();
  if (scene.has_gzf()) {
    INTR_ASSIGN_OR_RETURN(auto gz_file,
                          intrinsic::GZFile::Open((dir / scene.gzf()).string()));
    INTR_ASSIGN_OR_RETURN(world, World::FromFile(*gz_file));
  } else if (scene.has_sdf()) {
    INTR_ASSIGN_OR_RETURN(std::string sdf_text, ReadFile(dir / scene.sdf()));
    intrinsic::sdf::WorldFromSdf world_from_sdf;
    world_from_sdf.SetUriResolver(MakeUriResolver(AssetRoots(dir)));
    world_from_sdf.SetGroupIdGeneration(scene.generate_group_ids());
    INTR_RETURN_IF_ERROR(world_from_sdf.Parse(sdf_text))
        << "while parsing " << scene.sdf();
    INTR_ASSIGN_OR_RETURN(std::unique_ptr<World> parsed,
                          world_from_sdf.GetWorld());
    world = std::move(*parsed);
  } else {
    return absl::InvalidArgumentError("scene needs `gzf` or `sdf`");
  }

  for (const std::string& f : scene.world_updates_files()) {
    INTR_ASSIGN_OR_RETURN(
        auto updates, ReadTextProto<intrinsic_proto::world::WorldUpdates>(dir / f));
    INTR_RETURN_IF_ERROR(intrinsic::UpdateWorld(updates, &world))
        << "while applying " << f;
  }
  if (scene.has_world_updates()) {
    INTR_RETURN_IF_ERROR(intrinsic::UpdateWorld(scene.world_updates(), &world))
        << "while applying inline world_updates";
  }
  return world;
}

json PoseJson(const Pose3d& pose) {
  const Eigen::Matrix4d m = pose.matrix();
  Eigen::Quaterniond q(Eigen::Matrix3d(m.block<3, 3>(0, 0)));
  q.normalize();
  return {{"p", {m(0, 3), m(1, 3), m(2, 3)}},
          {"q", {q.w(), q.x(), q.y(), q.z()}}};
}

json MatrixJson(const Eigen::Matrix4d& m) {
  json rows = json::array();
  for (int r = 0; r < 4; ++r) rows.push_back({m(r, 0), m(r, 1), m(r, 2), m(r, 3)});
  return rows;
}

json VectorJson(const VectorXd& v) {
  return std::vector<double>(v.data(), v.data() + v.size());
}

// An entity that carries geometry, with the world pose we track over time.
struct Node {
  intrinsic::AttachmentEntityId id;
  json def;
};

absl::StatusOr<std::string> WriteGlb(const std::string& glb, const fs::path& out) {
  const std::string name =
      absl::StrFormat("meshes/%016x.glb", absl::HashOf(glb));
  if (const fs::path path = out / name; !fs::exists(path)) {
    std::ofstream f(path, std::ios::binary);
    f << glb;
    if (!f) return absl::InternalError(absl::StrCat("Cannot write ", path.string()));
  }
  return name;
}

// Exports the visual (and optionally collision) geometry of every entity.
absl::StatusOr<std::vector<Node>> ExportGeometry(World& world, const fs::path& out) {
  fs::create_directories(out / "meshes");
  std::vector<Node> nodes;
  for (intrinsic::GeometryEntityId gid :
       world.GetTypedEntityIds<intrinsic::GeometryEntityId>()) {
    auto attachment_id = world.ValidateEntity<intrinsic::AttachmentEntityId>(gid);
    if (!attachment_id.ok()) continue;
    INTR_ASSIGN_OR_RETURN(intrinsic::WorldEntity * entity, world.GetEntityById(gid));
    INTR_ASSIGN_OR_RETURN(const intrinsic::GeometryComponent* geo_component,
                          entity->GetComponent<intrinsic::GeometryComponent>());
    json geometries = json::array();
    for (const auto& [kind, kind_name] :
         {std::pair{intrinsic::kKindVisualGeometry, "visual"},
          std::pair{intrinsic::kKindCollisionGeometry, "collision"}}) {
      if (std::string(kind_name) == "collision" &&
          !absl::GetFlag(FLAGS_export_collision_geometry)) {
        continue;
      }
      if (!geo_component->HasGeometry(kind)) continue;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
      auto geometry_set = geo_component->GetGeometry(kind);
#pragma clang diagnostic pop
      if (!geometry_set.ok()) {
        LOG(WARNING) << "Skipping " << kind_name << " geometry of "
                     << entity->GetLocalName() << ": " << geometry_set.status();
        continue;
      }
      for (const auto& [geo_name, tg] : *geometry_set) {
        auto renderable =
            std::string(kind_name) == "visual"
                ? intrinsic::geo::GenerateRenderableWithMaterialOverrides(tg.shape())
                : intrinsic::geo::GenerateRenderableForExactGeometry(
                      tg.shape().GetExactGeometry(), std::nullopt);
        if (!renderable.ok() || *renderable == nullptr) {
          LOG(WARNING) << "No renderable for " << entity->GetLocalName() << "/"
                       << geo_name << ": " << renderable.status();
          continue;
        }
        INTR_ASSIGN_OR_RETURN(std::string glb_path,
                              WriteGlb((*renderable)->GetGLBString(), out));
        geometries.push_back({{"name", geo_name},
                              {"kind", kind_name},
                              {"glb", glb_path},
                              {"ref_t_shape", MatrixJson(tg.ref_t_shape())}});
      }
    }
    if (geometries.empty()) continue;
    nodes.push_back(
        {*attachment_id,
         {{"id", absl::StrCat(gid.value())},
          {"name", world.GetLocalNameForEntityById(gid)},
          {"geometry", std::move(geometries)},
          {"world_t_node",
           PoseJson(world.GetTransform(intrinsic::kRootEntityId, *attachment_id))}}});
  }
  return nodes;
}

intrinsic::MotionPlannerFlags PlannerFlags() {
  intrinsic::MotionPlannerFlags flags;
  flags.enable_path_refinement_validation_step =
      absl::GetFlag(FLAGS_path_refinement_validation);
  flags.path_refinement_validation_margin_relative_factor = 0.9;
  flags.path_refinement_validation_margin_absolute_factor = 0.005;
  flags.enable_concurrent_collision_checking =
      absl::GetFlag(FLAGS_collision_threads) > 0;
  flags.concurrent_collision_checking_thread_count =
      std::max(1, absl::GetFlag(FLAGS_collision_threads));
  flags.enable_fallback_trajectory = absl::GetFlag(FLAGS_fallback_trajectory);
  flags.enable_fallback_trajectory_local_adjust = true;
  return flags;
}

// Moves the robot through `configs` and records the poses of all nodes and of
// the robot flange.
absl::StatusOr<json> RecordFrames(
    World& world, intrinsic::object_world::KinematicObject& robot,
    const std::vector<Node>& nodes, const std::vector<double>& times,
    const std::vector<VectorXd>& configs) {
  std::optional<intrinsic::AttachmentEntityId> flange_id;
  if (auto flange = robot.GetSingleIsoFlangeFrame(); flange.ok()) {
    if (auto id = (*flange)->GetTransformOriginEntityId(); id.ok()) flange_id = *id;
  }
  json poses = json::object();
  json flange = json::array();
  for (const Node& n : nodes) poses[n.def["id"].get<std::string>()] = json::array();
  for (const VectorXd& q : configs) {
    INTR_RETURN_IF_ERROR(robot.SetJointPositions(q, /*enforce_limits=*/false));
    for (const Node& n : nodes) {
      poses[n.def["id"].get<std::string>()].push_back(
          PoseJson(world.GetTransform(intrinsic::kRootEntityId, n.id)));
    }
    if (flange_id) {
      flange.push_back(
          PoseJson(world.GetTransform(intrinsic::kRootEntityId, *flange_id))["p"]);
    }
  }
  // Only keep nodes that are not at their exported pose (`world_t_node`).
  json moving = json::object();
  for (const Node& n : nodes) {
    const std::string id = n.def["id"].get<std::string>();
    for (const json& p : poses[id]) {
      if (p != n.def["world_t_node"]) {
        moving[id] = std::move(poses[id]);
        break;
      }
    }
  }
  return json{{"t", times}, {"poses", std::move(moving)}, {"flange", std::move(flange)}};
}

absl::Status Run() {
  const fs::path scenario_path = UserPath(absl::GetFlag(FLAGS_scenario));
  if (scenario_path.empty()) return absl::InvalidArgumentError("--scenario is required");
  INTR_ASSIGN_OR_RETURN(Scenario scenario, ReadTextProto<Scenario>(scenario_path));
  const fs::path dir = scenario_path.parent_path();

  absl::Time t0 = absl::Now();
  INTR_ASSIGN_OR_RETURN(World world, LoadScene(scenario.scene(), dir));
  LOG(INFO) << "Loaded scene in " << absl::Now() - t0;

  INTR_ASSIGN_OR_RETURN(auto object_world,
                        intrinsic::object_world::ObjectWorld::CreateView(world));
  INTR_ASSIGN_OR_RETURN(
      intrinsic::object_world::KinematicObject * robot,
      object_world->GetKinematicObject(intrinsic::WorldObjectName(scenario.robot())));

  intrinsic_proto::motion_planning::v1::RobotSpecification robot_spec =
      scenario.robot_specification();
  if (!robot_spec.has_robot_reference()) {
    *robot_spec.mutable_robot_reference() =
        intrinsic::CreateRobotReferenceByName(scenario.robot());
  }
  if (!scenario.start_joint_positions().empty()) {
    const auto& q = scenario.start_joint_positions();
    INTR_RETURN_IF_ERROR(robot->SetJointPositions(
        Eigen::Map<const VectorXd>(q.data(), q.size())));
  }
  INTR_ASSIGN_OR_RETURN(VectorXd q_current, robot->GetJointPositions());

  const fs::path out =
      absl::GetFlag(FLAGS_out).empty() ? fs::path() : UserPath(absl::GetFlag(FLAGS_out));
  std::vector<Node> nodes;
  if (!out.empty()) {
    INTR_ASSIGN_OR_RETURN(nodes, ExportGeometry(world, out));
  }

  INTR_ASSIGN_OR_RETURN(auto planner, intrinsic::MotionPlanner::Create(PlannerFlags()));

  json motions = json::array();
  int failures = 0;
  for (int i = 0; i < scenario.motions_size(); ++i) {
    const Motion& motion = scenario.motions(i);
    const std::string name = motion.name().empty() ? absl::StrCat("motion_", i) : motion.name();
    INTR_RETURN_IF_ERROR(robot->SetJointPositions(q_current));
    json m = {{"name", name}, {"start", VectorJson(q_current)}};

    std::vector<double> times;
    std::vector<VectorXd> configs;
    absl::Time start = absl::Now();
    if (motion.path_only()) {
      auto result = planner->PlanPath(*object_world, robot_spec,
                                      motion.motion_specification(),
                                      motion.motion_planner_config(), std::nullopt);
      m["planning_time_s"] = absl::ToDoubleSeconds(absl::Now() - start);
      m["ok"] = result.ok();
      if (!result.ok()) {
        m["status"] = result.status().ToString();
      } else {
        json waypoints = json::array();
        for (const VectorXd& q : result->path) waypoints.push_back(VectorJson(q));
        m["waypoints"] = waypoints;
        // Animate the joint-space polyline at unit speed (1 rad/s).
        double t = 0;
        for (size_t k = 0; k < result->path.size(); ++k) {
          if (k > 0) {
            const VectorXd& a = result->path[k - 1];
            const VectorXd& b = result->path[k];
            const int steps = std::max(1, static_cast<int>((b - a).norm() / 0.02));
            for (int s = 1; s <= steps; ++s) {
              t += (b - a).norm() / steps;
              times.push_back(t);
              configs.push_back(a + (b - a) * (static_cast<double>(s) / steps));
            }
          } else {
            times.push_back(0);
            configs.push_back(result->path[0]);
          }
        }
      }
    } else {
      auto result = planner->PlanTrajectory(*object_world, robot_spec,
                                            motion.motion_specification(),
                                            motion.motion_planner_config(), std::nullopt);
      m["planning_time_s"] = absl::ToDoubleSeconds(absl::Now() - start);
      m["ok"] = result.ok();
      if (!result.ok()) {
        m["status"] = result.status().ToString();
      } else {
        const auto& traj = result->trajectory;
        json t_all = json::array(), q_all = json::array(), qd_all = json::array(),
             qdd_all = json::array();
        for (size_t k = 0; k < traj.size(); ++k) {
          t_all.push_back(absl::ToDoubleSeconds(traj.time_stamps()[k]));
          q_all.push_back(VectorJson(traj.data()[k].position));
          qd_all.push_back(VectorJson(traj.data()[k].velocity));
          qdd_all.push_back(VectorJson(traj.data()[k].acceleration));
        }
        m["duration_s"] = absl::ToDoubleSeconds(traj.Duration());
        m["fallback_stage"] = static_cast<int>(result->fallback_stage);
        m["num_path_segments"] = result->path_segments.size();
        m["trajectory"] = {{"t", t_all}, {"q", q_all}, {"qd", qd_all}, {"qdd", qdd_all}};
        const double dt = absl::GetFlag(FLAGS_viz_dt);
        for (double t = 0;; t += dt) {
          const bool last = t >= absl::ToDoubleSeconds(traj.Duration());
          const absl::Duration td = last ? traj.Duration() : absl::Seconds(t);
          const int k = traj.GetLowerIndexForTime(td);
          times.push_back(absl::ToDoubleSeconds(traj.time_stamps()[k]));
          configs.push_back(traj.data()[k].position);
          if (last) break;
        }
      }
    }

    if (m["ok"].get<bool>()) {
      std::cout << absl::StrFormat("[ OK ] %-28s planned in %7.3f s", name,
                                   m["planning_time_s"].get<double>());
      if (m.contains("duration_s")) {
        std::cout << absl::StrFormat(", duration %6.3f s", m["duration_s"].get<double>());
      }
      std::cout << std::endl;
      if (!nodes.empty()) {
        INTR_ASSIGN_OR_RETURN(m["frames"],
                              RecordFrames(world, *robot, nodes, times, configs));
      }
      q_current = configs.back();
    } else {
      ++failures;
      if (!nodes.empty()) {
        // Show the robot at the start of the failed motion.
        INTR_ASSIGN_OR_RETURN(m["frames"], RecordFrames(world, *robot, nodes, {0.0},
                                                        {q_current}));
      }
      std::cout << absl::StrFormat("[FAIL] %-28s after %7.3f s: %s", name,
                                   m["planning_time_s"].get<double>(),
                                   m["status"].get<std::string>())
                << std::endl;
    }
    motions.push_back(std::move(m));
  }

  if (!out.empty()) {
    INTR_ASSIGN_OR_RETURN(auto joint_ids, robot->GetJointEntityIds());
    json joint_names = json::array();
    for (auto id : joint_ids) joint_names.push_back(world.GetLocalNameForEntityById(id));
    json doc = {{"scenario", scenario_path.string()},
                {"robot", scenario.robot()},
                {"joint_names", joint_names},
                {"nodes", json::array()},
                {"motions", motions}};
    for (const Node& n : nodes) doc["nodes"].push_back(n.def);
    std::ofstream f(out / "result.json");
    f << doc.dump();
    std::cout << "Wrote " << (out / "result.json").string() << std::endl;
  }
  return failures == 0 ? absl::OkStatus()
                       : absl::AbortedError(absl::StrCat(failures, " motion(s) failed"));
}

}  // namespace
}  // namespace planner_sandbox

int main(int argc, char** argv) {
  absl::SetProgramUsageMessage(
      "Plans the motions of a planner_sandbox.Scenario and exports them for "
      "viz/view.py.");
  // Planner progress is logged at INFO; pass --stderrthreshold=0 to see it.
  absl::SetStderrThreshold(absl::LogSeverityAtLeast::kWarning);
  absl::ParseCommandLine(argc, argv);
  absl::InitializeLog();
  if (absl::Status s = planner_sandbox::Run(); !s.ok()) {
    std::cerr << s << std::endl;
    return 1;
  }
  return 0;
}
