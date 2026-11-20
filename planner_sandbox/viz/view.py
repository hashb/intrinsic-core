# /// script
# requires-python = ">=3.10"
# dependencies = [
#   "numpy",
#   "plotly==7.1.0",
#   "trimesh==5.1.1",
#   "viser==1.1.1",
# ]
# ///
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
"""Viser viewer for planner_sandbox results.

  uv run planner_sandbox/viz/view.py /tmp/ur5e        # dir written by --out

Reloads automatically whenever plan_scenario rewrites result.json, so you can
keep the browser tab open while iterating on a scenario or the planner.
"""

import argparse
import dataclasses
import functools
import json
from pathlib import Path
import threading
import time

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import trimesh
import viser

ALL_MOTIONS = "All (in sequence)"
PATH_COLORS = [
    (31, 119, 180),
    (255, 127, 14),
    (44, 160, 44),
    (214, 39, 40),
    (148, 103, 189),
    (140, 86, 75),
    (227, 119, 194),
]


@functools.lru_cache(maxsize=None)
def _load_scene(path: Path) -> trimesh.Scene:
  return trimesh.load(path, force="scene")


def visual_glb(path: Path, ref_t_shape: np.ndarray) -> bytes:
  scene = _load_scene(path).copy()
  scene.apply_transform(ref_t_shape)
  return scene.export(file_type="glb")


def collision_mesh(path: Path, ref_t_shape: np.ndarray) -> trimesh.Trimesh:
  mesh = _load_scene(path).to_mesh()
  mesh.apply_transform(ref_t_shape)
  return mesh


@dataclasses.dataclass
class Timeline:
  """Frames of one motion (or several motions back to back)."""

  t: np.ndarray  # [N]
  poses: dict[str, list[dict]]  # node id -> N poses {"p": [3], "q": [4]}
  flange: np.ndarray  # [N, 3]

  @property
  def duration(self) -> float:
    return float(self.t[-1]) if len(self.t) else 0.0

  @staticmethod
  def of(motion: dict) -> "Timeline":
    f = motion.get("frames") or {"t": [0.0], "poses": {}, "flange": []}
    return Timeline(
        np.asarray(f["t"], dtype=float),
        f["poses"],
        np.asarray(f["flange"], dtype=float).reshape(-1, 3),
    )

  @staticmethod
  def concat(timelines: list["Timeline"]) -> "Timeline":
    ts, flange, poses, offset = [], [], {}, 0.0
    ids = {i for tl in timelines for i in tl.poses}
    for tl in timelines:
      if not len(tl.t):
        continue
      ts.append(tl.t + offset)
      flange.append(tl.flange)
      for i in ids:
        # Nodes that do not move in a motion keep their last pose.
        default = poses[i][-1] if poses.get(i) else None
        poses.setdefault(i, []).extend(
            tl.poses.get(i) or [default] * len(tl.t)
        )
      offset = ts[-1][-1] + 0.25
    return Timeline(
        np.concatenate(ts) if ts else np.zeros(1),
        poses,
        np.concatenate(flange) if flange else np.zeros((0, 3)),
    )

  def index(self, t: float) -> int:
    return int(
        np.clip(np.searchsorted(self.t, t, side="right") - 1, 0, len(self.t) - 1)
    )


class Viewer:

  def __init__(self, out_dir: Path, server: viser.ViserServer):
    self.out_dir = out_dir
    self.server = server
    self.lock = threading.Lock()
    self.playing = True
    self.t = 0.0
    self.node_frames: dict[str, viser.FrameHandle] = {}
    self.visual: list = []
    self.collision: list = []
    self.paths: list = []

  # --------------------------------------------------------------- building

  def build(self):
    doc = json.loads((self.out_dir / "result.json").read_text())
    self.doc = doc
    self.motions = doc["motions"]
    self.timelines = {m["name"]: Timeline.of(m) for m in self.motions}
    ok = [self.timelines[m["name"]] for m in self.motions if m["ok"]]
    self.timelines[ALL_MOTIONS] = Timeline.concat(ok)
    self.node_rest = {n["id"]: n["world_t_node"] for n in doc["nodes"]}

    scene, gui = self.server.scene, self.server.gui
    scene.reset()
    gui.reset()
    self.node_frames.clear()
    self.visual.clear()
    self.collision.clear()
    self.paths.clear()

    scene.add_grid("/grid", width=4, height=4, cell_size=0.1, section_size=0.5)
    for n in doc["nodes"]:
      pose = n["world_t_node"]
      frame = scene.add_frame(
          f"/nodes/n{n['id']}",
          show_axes=False,
          axes_length=0.08,
          axes_radius=0.003,
          position=pose["p"],
          wxyz=pose["q"],
      )
      self.node_frames[n["id"]] = frame
      for k, g in enumerate(n["geometry"]):
        path = self.out_dir / g["glb"]
        m = np.asarray(g["ref_t_shape"], dtype=float)
        if g["kind"] == "visual":
          self.visual.append(
              scene.add_glb(f"/nodes/n{n['id']}/visual_{k}", visual_glb(path, m))
          )
        else:
          mesh = collision_mesh(path, m)
          self.collision.append(
              scene.add_mesh_simple(
                  f"/nodes/n{n['id']}/collision_{k}",
                  vertices=mesh.vertices.astype(np.float32),
                  faces=mesh.faces.astype(np.uint32),
                  color=(255, 140, 0),
                  opacity=0.45,
                  visible=False,
              )
          )

    for i, m in enumerate(self.motions):
      tl = self.timelines[m["name"]]
      if len(tl.flange) < 2:
        continue
      color = PATH_COLORS[i % len(PATH_COLORS)]
      self.paths.append(
          scene.add_spline_catmull_rom(
              f"/paths/{i}_{m['name']}",
              points=tl.flange,
              color=color,
              thickness=0.004,
          )
      )
      self.paths.append(
          scene.add_icosphere(
              f"/paths/{i}_{m['name']}_goal",
              radius=0.012,
              color=color,
              position=tuple(tl.flange[-1]),
          )
      )

    self._build_gui()
    self.select(self.motion_dropdown.value)

  def _build_gui(self):
    gui = self.server.gui
    gui.add_markdown(
        f"**{Path(self.doc['scenario']).name}** — robot `{self.doc['robot']}`"
    )
    names = [m["name"] for m in self.motions]
    self.motion_dropdown = gui.add_dropdown(
        "Motion", [ALL_MOTIONS] + names, initial_value=ALL_MOTIONS
    )
    self.motion_dropdown.on_update(lambda _: self.select(self.motion_dropdown.value))

    self.play_button = gui.add_button("Pause", icon=viser.Icon.PLAYER_PAUSE)
    self.play_button.on_click(lambda _: self.toggle_play())
    self.time_slider = gui.add_slider("Time [s]", 0.0, 1.0, 0.001, 0.0)
    self.time_slider.on_update(self._on_slider)
    self.speed = gui.add_slider("Speed", 0.1, 4.0, 0.1, 1.0)
    self.loop = gui.add_checkbox("Loop", True)

    with gui.add_folder("Display"):
      show_visual = gui.add_checkbox("Visual geometry", True)
      show_collision = gui.add_checkbox("Collision geometry", False)
      show_paths = gui.add_checkbox("Flange paths", True)
      show_axes = gui.add_checkbox("Link frames", False)

    def set_visible(handles, value):
      for h in handles:
        h.visible = value

    show_visual.on_update(lambda _: set_visible(self.visual, show_visual.value))
    show_collision.on_update(
        lambda _: set_visible(self.collision, show_collision.value)
    )
    show_paths.on_update(lambda _: set_visible(self.paths, show_paths.value))

    def toggle_axes(_):
      for f in self.node_frames.values():
        f.show_axes = show_axes.value

    show_axes.on_update(toggle_axes)

    with gui.add_folder("Result"):
      self.info = gui.add_markdown("")
      self.joints = gui.add_markdown("")
    with gui.add_folder("Joint trajectory", expand_by_default=False):
      self.plot = gui.add_plotly(go.Figure(), aspect=1.0)

  # ------------------------------------------------------------- interaction

  def select(self, name: str):
    with self.lock:
      self.timeline = self.timelines[name]
      self.t = 0.0
      self._updating_slider = True
      self.time_slider.max = max(self.timeline.duration, 1e-3)
      self.time_slider.value = 0.0
      self._updating_slider = False
    self.info.content = self._info_markdown(name)
    self.plot.figure = self._joint_figure(name)
    self.render(0.0)

  def toggle_play(self):
    self.playing = not self.playing
    self.play_button.label = "Pause" if self.playing else "Play"
    self.play_button.icon = (
        viser.Icon.PLAYER_PAUSE if self.playing else viser.Icon.PLAYER_PLAY
    )

  def _on_slider(self, _):
    if getattr(self, "_updating_slider", False):
      return
    with self.lock:
      self.t = self.time_slider.value
    self.render(self.t)

  def _info_markdown(self, name: str) -> str:
    rows = ["| motion | result | plan [ms] | duration [s] |", "|---|---|---|---|"]
    for m in self.motions:
      if name not in (ALL_MOTIONS, m["name"]):
        continue
      result = "ok" if m["ok"] else "**failed**"
      if m.get("fallback_stage"):
        result += f" (fallback {m['fallback_stage']})"
      if "action" in m:
        rows.append(f"| {m['name']} | {m['action']} `{m['object']}` | | |")
        continue
      rows.append(
          f"| {m['name']} | {result} | {1e3 * m['planning_time_s']:.1f} |"
          f" {m.get('duration_s', float('nan')):.3f} |"
      )
    errors = [
        f"`{m['name']}`: {m['status']}"
        for m in self.motions
        if not m["ok"] and name in (ALL_MOTIONS, m["name"])
    ]
    return "\n".join(rows + [""] + errors)

  def _joint_figure(self, name: str) -> go.Figure:
    fig = make_subplots(rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.06)
    offset = 0.0
    joint_names = self.doc["joint_names"]
    for m in self.motions:
      traj = m.get("trajectory")
      if not traj or name not in (ALL_MOTIONS, m["name"]):
        continue
      t = np.asarray(traj["t"]) + offset
      q, qd = np.asarray(traj["q"]), np.asarray(traj["qd"])
      for j in range(q.shape[1]):
        color = f"hsl({int(360 * j / q.shape[1])},60%,45%)"
        label = joint_names[j] if j < len(joint_names) else f"j{j}"
        first = bool(offset == 0.0)
        fig.add_trace(
            go.Scatter(x=t, y=q[:, j], name=label, legendgroup=label,
                       showlegend=first, line=dict(color=color, width=1.5)),
            row=1, col=1)
        fig.add_trace(
            go.Scatter(x=t, y=qd[:, j], name=label, legendgroup=label,
                       showlegend=False, line=dict(color=color, width=1.5)),
            row=2, col=1)
      offset = float(t[-1]) + 0.25
    fig.update_yaxes(title_text="q [rad]", row=1, col=1)
    fig.update_yaxes(title_text="qd [rad/s]", row=2, col=1)
    fig.update_xaxes(title_text="t [s]", row=2, col=1)
    fig.update_layout(margin=dict(l=10, r=10, t=10, b=10), height=420,
                      legend=dict(orientation="h", y=-0.15))
    return fig

  def render(self, t: float):
    tl = self.timeline
    k = tl.index(t)
    for node_id, frame in self.node_frames.items():
      poses = tl.poses.get(node_id)
      pose = poses[k] if poses and poses[k] else self.node_rest[node_id]
      frame.position = pose["p"]
      frame.wxyz = pose["q"]
    self.joints.content = f"t = {t:.3f} s (frame {k + 1}/{len(tl.t)})"

  # -------------------------------------------------------------------- loop

  def step(self, dt: float):
    if not self.playing:
      return
    with self.lock:
      duration = self.timeline.duration
      self.t += dt * self.speed.value
      if self.t > duration:
        self.t = 0.0 if self.loop.value else duration
      t = self.t
      self._updating_slider = True
      self.time_slider.value = min(t, self.time_slider.max)
      self._updating_slider = False
    self.render(t)


def main():
  parser = argparse.ArgumentParser(description=__doc__,
                                   formatter_class=argparse.RawTextHelpFormatter)
  parser.add_argument("out_dir", type=Path, help="Directory passed to --out.")
  parser.add_argument("--port", type=int, default=8080)
  args = parser.parse_args()

  result = args.out_dir / "result.json"
  while not result.exists():
    print(f"Waiting for {result} ...", flush=True)
    time.sleep(1.0)

  server = viser.ViserServer(port=args.port)
  server.initial_camera.position = (1.4, -1.4, 1.1)
  server.initial_camera.look_at = (0.2, 0.0, 0.3)
  viewer = Viewer(args.out_dir, server)
  viewer.build()
  mtime = result.stat().st_mtime

  period = 1.0 / 30.0
  last_check = time.time()
  while True:
    time.sleep(period)
    viewer.step(period)
    if time.time() - last_check > 1.0:
      last_check = time.time()
      try:
        new_mtime = result.stat().st_mtime
      except FileNotFoundError:
        continue
      if new_mtime != mtime:
        time.sleep(0.2)  # Let the planner finish writing.
        mtime = result.stat().st_mtime
        print("result.json changed, reloading", flush=True)
        try:
          viewer.build()
        except Exception as e:  # Keep the server alive on a bad result.
          print(f"Reload failed: {e}", flush=True)


if __name__ == "__main__":
  main()
