"""Compare causal raw/bias/global-EKF paths and local two-state EKF plus odometry.

The two-state EKF stays unchanged. An external pose integrator connects its
straight-passage estimates with wheel/gyro odometry across turns. Cardinal
passage orientation is a labelled geometric assumption, not an XY/route fit.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

from wall_heading_ekf import wrap_angle


class LocalPosePath:
    """External pose bookkeeping, without adding states/covariance to the EKF.

    Without local corrections this exactly reproduces bias-only propagation.
    On trusted straight samples psi corrects heading; consecutive samples use
    delta-e for the lateral increment. The first trusted e anchors an offset,
    not an absolute world coordinate. No grid coordinates are assigned to XY.
    """
    def __init__(self, use_local=True, heading_std_limit_deg=3., axis_limit_deg=25., psi_limit_deg=15.):
        self.use_local = use_local
        self.heading_std_limit_deg = heading_std_limit_deg
        self.axis_limit = math.radians(axis_limit_deg)
        self.psi_limit_deg = psi_limit_deg
        self.position = np.zeros(2)
        self.heading = 0.
        self.previous = None
        self.segment = None
        self.axis = None
        self.previous_local = None
        self.used_samples = 0
        self.replaced_lateral_steps = 0

    def update(self, sample):
        raw_position = np.array([sample["raw.east_mm"], sample["raw.north_mm"]], dtype=float) / 1000
        raw_heading = float(sample["raw.yaw_rad"])
        dt = float(sample["dt_ms"]) / 1000
        bias = math.radians(float(sample["initial_bias_deg_s"]))
        if not np.isfinite([*raw_position, raw_heading, dt, bias]).all() or not 0 < dt < .1:
            raise ValueError("Invalid causal path sample or time units")
        if self.previous is None:
            self.previous = (raw_position, raw_heading, float(sample["time_ms"]), int(sample["sequence"]))
            return self._result(False, False, "initial_origin")
        old_raw, old_heading, old_time, old_sequence = self.previous
        if int(sample["sequence"]) != old_sequence + 1 or float(sample["time_ms"]) <= old_time:
            raise ValueError("Discontinuous path samples")
        delta = raw_position - old_raw
        angle = self.heading - old_heading - .5*bias*dt
        c, s = math.cos(angle), math.sin(angle)
        step = np.array([c*delta[0]-s*delta[1], s*delta[0]+c*delta[1]])
        predicted_heading = self.heading + raw_heading - old_heading - bias*dt
        current_segment = int(sample["segment"])
        if current_segment != self.segment:
            self.segment = current_segment
            self.axis = None
            self.previous_local = None
        reason = "disabled" if not self.use_local else "not_trusted_straight"
        used, replaced = False, False
        local_values = [sample.get("local.e_mm"), sample.get("local.psi_deg"), sample.get("local.std_psi_deg")]
        finite_local = all(value is not None and math.isfinite(float(value)) for value in local_values)
        trusted = (self.use_local and bool(sample["local_active"]) and bool(sample["local_observed"])
                   and finite_local and float(local_values[2]) <= self.heading_std_limit_deg
                   and abs(float(local_values[1])) <= self.psi_limit_deg)
        if trusted:
            e = float(local_values[0])/1000
            psi = math.radians(float(local_values[1]))
            if self.axis is None:
                candidate = round(predicted_heading/(math.pi/2))*(math.pi/2)
                if abs(wrap_angle(predicted_heading-candidate)) <= self.axis_limit:
                    self.axis = candidate
            if self.axis is not None:
                if self.previous_local is not None:
                    old_e, old_psi = self.previous_local
                    forward = np.array([-math.sin(self.axis), math.cos(self.axis)])
                    left = np.array([-math.cos(self.axis), -math.sin(self.axis)])
                    # Current straight candidates have positive target velocity.
                    distance = float(np.linalg.norm(delta))
                    step = forward*distance*math.cos(.5*(old_psi+psi)) + left*(e-old_e)
                    replaced = True
                    self.replaced_lateral_steps += 1
                predicted_heading = self.axis+psi
                self.previous_local = (e, psi)
                self.used_samples += 1
                used = True
                reason = "two_state_heading_and_relative_lateral"
            else:
                reason = "cardinal_axis_ambiguous"
                self.previous_local = None
        else:
            self.previous_local = None
        self.position += step
        self.heading = predicted_heading
        self.previous = (raw_position, raw_heading, float(sample["time_ms"]), int(sample["sequence"]))
        return self._result(used, replaced, reason)

    def _result(self, used, replaced, reason):
        return {"east_mm": float(self.position[0]*1000), "north_mm": float(self.position[1]*1000),
                "heading_rad": self.heading, "used_local": used, "replaced_lateral_step": replaced,
                "passage_axis_rad": self.axis, "reason": reason}


def build_comparison(states):
    needed = ["sequence", "time_ms", "raw.east_mm", "raw.north_mm", "raw.yaw_rad", "dt_ms",
              "initial_bias_deg_s", "bias_only.east_mm", "bias_only.north_mm", "global.east_mm",
              "global.north_mm", "segment", "local_active", "local_observed", "local.e_mm",
              "local.psi_deg", "local.std_psi_deg"]
    if not set(needed).issubset(states.columns) or len(states) < 2:
        raise ValueError("Missing streaming state columns or too few samples")
    numeric = [key for key in needed if not key.startswith("local.")]
    if not np.isfinite(states[numeric].to_numpy(float)).all():
        raise ValueError("Nonfinite global path field")
    if not states.local_active.isin([True, False]).all() or not states.local_observed.isin([True, False]).all():
        raise ValueError("Local validity flags must be booleans")
    comparison = states[["sequence", "time_ms"]].copy()
    names = {"raw": "raw", "bias_only": "bias_only", "global_ekf": "global"}
    for name, prefix in names.items():
        for axis in ("east", "north"):
            comparison[name+"."+axis+"_mm"] = states[prefix+"."+axis+"_mm"]
    integrator, no_update = LocalPosePath(), LocalPosePath(use_local=False)
    projected = []
    control = []
    for row in states.to_dict("records"):
        projected.append(integrator.update(row))
        control.append(no_update.update(row))
    for axis in ("east", "north"):
        comparison["local_ekf_path."+axis+"_mm"] = [row[axis+"_mm"] for row in projected]
        if not np.allclose([row[axis+"_mm"] for row in control], states["bias_only."+axis+"_mm"], atol=1e-7, rtol=1e-10):
            raise ValueError("Disabled local wrapper does not reproduce bias-only path")
    comparison["local_path.used_local"] = [row["used_local"] for row in projected]
    comparison["local_path.replaced_lateral_step"] = [row["replaced_lateral_step"] for row in projected]
    comparison["local_path.heading_rad"] = [row["heading_rad"] for row in projected]
    return comparison, integrator


LABELS = {"raw": "補正なし", "bias_only": "ゼロ点補正のみ", "global_ekf": "逐次4状態EKF",
          "local_ekf_path": "2状態EKF＋外側の経路積分"}
COLORS = {"raw": "#888888", "bias_only": "#2366b4", "global_ekf": "#bd5b16", "local_ekf_path": "#298452"}


def draw(output, comparison, route, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.lines import Line2D
    fonts = {font.name for font in font_manager.fontManager.ttflist}
    plt.rcParams["font.family"] = next((name for name in ("Yu Gothic", "Meiryo", "MS Gothic") if name in fonts), "DejaVu Sans")
    plt.rcParams["axes.unicode_minus"] = False
    paths = {name: comparison[[name+".east_mm", name+".north_mm"]].to_numpy(float) for name in LABELS}
    points = [np.vstack(list(paths.values()))]
    reference = []
    if route:
        reference = [np.asarray(route[phase], dtype=float)*90 for phase in ("outward", "return")]
        points.extend(reference)
    points = np.vstack(points)
    low, high = np.floor((points.min(axis=0)-45)/90)*90, np.ceil((points.max(axis=0)+45)/90)*90

    def base(axis):
        for index, part in enumerate(reference):
            axis.plot(part[:, 0], part[:, 1], "--", color="#333333", alpha=.4, lw=1.,
                      label="推定route（表示用）" if index == 0 else None, zorder=0)
        axis.scatter(0, 0, s=45, facecolor="white", edgecolor="#222222", zorder=5)
        axis.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), xlabel="東 [mm]", ylabel="北 [mm]")
        axis.set_xticks(np.arange(low[0], high[0]+1, 180))
        axis.set_yticks(np.arange(low[1], high[1]+1, 180))
        axis.set_aspect("equal"); axis.grid(alpha=.25)

    fig, axes = plt.subplots(1, 2, figsize=(14, 8), constrained_layout=True,
                             gridspec_kw={"width_ratios": [1.5, 1.]})
    base(axes[0])
    for name, xy in paths.items():
        axes[0].plot(xy[:, 0], xy[:, 1], color=COLORS[name], lw=1.6, label=LABELS[name])
        axes[0].scatter(*xy[-1], marker="x", color=COLORS[name], s=45)
        # Show only the final 900mm of measured travel in the return-area inset.
        remaining = np.r_[np.cumsum(np.linalg.norm(np.diff(paths["raw"], axis=0), axis=1)[::-1])[::-1], 0.]
        tail = remaining <= 900
        axes[1].plot(xy[tail, 0], xy[tail, 1], color=COLORS[name], alpha=.65, lw=1.)
        axes[1].scatter(*xy[-1], marker="x", color=COLORS[name], s=65)
        endpoint = xy[-1]
    axes[0].set_title("補正ごとの全体経路 · 始点・初期北向きのみ共通")
    axes[0].legend(fontsize=8, loc="upper center", bbox_to_anchor=(.5, -.11), ncol=2)
    endpoints = np.vstack([np.zeros(2), *[xy[-1] for xy in paths.values()]])
    zoom_low, zoom_high = endpoints.min(axis=0)-35, endpoints.max(axis=0)+55
    axes[1].set(xlim=(zoom_low[0], zoom_high[0]), ylim=(zoom_low[1], zoom_high[1]),
                xlabel="東 [mm]", ylabel="北 [mm]", title="推定帰還点の拡大\n数値は始点からの距離・実位置誤差ではない")
    axes[1].scatter(0, 0, facecolor="white", edgecolor="#222222", s=60, zorder=5)
    axes[1].axhline(0, color="#777777", lw=.6); axes[1].axvline(0, color="#777777", lw=.6)
    axes[1].set_aspect("equal"); axes[1].grid(alpha=.25)
    short_labels = {"raw": "補正なし", "bias_only": "ゼロ点補正", "global_ekf": "4状態EKF", "local_ekf_path": "2状態EKF＋積分"}
    axes[1].legend(handles=[Line2D([], [], color=COLORS[name], marker="x", ls="",
                          label=f'{short_labels[name]}：{np.linalg.norm(xy[-1]):.1f} mm')
                          for name, xy in paths.items()], loc="upper center", bbox_to_anchor=(.5, -.15), fontsize=9)
    fig.suptitle(title+"\n2状態版は直進中のe・ψを使用し、前進・旋回と通路軸の対応を外側で補う。位置の終点合わせなし。", fontsize=12)
    fig.savefig(output/"paths_overlay.png", dpi=170, bbox_inches="tight"); plt.close(fig)

    fig, axes = plt.subplots(1, 4, figsize=(18, 7), constrained_layout=True)
    for axis, (name, xy) in zip(axes, paths.items()):
        base(axis)
        axis.plot(xy[:, 0], xy[:, 1], color=COLORS[name], lw=1.7)
        axis.scatter(*xy[-1], marker="x", color=COLORS[name], s=55)
        axis.set_title(LABELS[name]+f"\n推定帰還点 {np.linalg.norm(xy[-1]):.1f} mm", fontsize=10)
    fig.suptitle(title+" · 全条件同じ縮尺\n推定routeも同じログからの復元値。外部測位の正解値は未取得。", fontsize=12)
    fig.savefig(output/"paths_separate.png", dpi=170, bbox_inches="tight"); plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--states", type=Path, required=True)
    parser.add_argument("--route", type=Path, help="Optional inferred route for display only")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", default="補正ごとの逐次推定経路")
    args = parser.parse_args(argv)
    states = pd.read_csv(args.states)
    comparison, integrator = build_comparison(states)
    source_report = args.states.parent/"report.json"
    source_metadata = json.loads(source_report.read_text(encoding="utf-8")) if source_report.exists() else {}
    if "motion_rows" in source_metadata and source_metadata["motion_rows"] != len(states):
        raise ValueError("State row count does not match the source estimation report")
    if not np.isfinite(comparison.select_dtypes(include=["number"])).all().all():
        raise ValueError("Nonfinite output path")
    args.output.mkdir(parents=True, exist_ok=True)
    comparison.to_csv(args.output/"compared_paths.csv", index=False)
    report = {"states": str(args.states.resolve()), "route_display_only": str(args.route.resolve()) if args.route else None,
              "machine": source_metadata.get("machine", "not recorded"),
              "capture_settings": source_metadata.get("settings", {}),
              "source_estimation_report": str(source_report.resolve()) if source_report.exists() else None,
              "source_estimation_implementation_sha256": source_metadata.get("implementation_sha256"),
              "samples": len(comparison), "first_sequence": int(comparison.sequence.iloc[0]),
              "last_sequence": int(comparison.sequence.iloc[-1]), "initial_alignment": "Shared origin and initial north; no later translation, rotation, scale or endpoint fit",
              "local_ekf_dimension": 2, "local_path_external_integrator": True,
              "local_samples_used": integrator.used_samples, "local_lateral_steps_replaced": integrator.replaced_lateral_steps,
              "local_path_conditions": {"heading_std_max_deg": 3., "psi_abs_max_deg": 15., "cardinal_axis_association_max_deg": 25.},
              "endpoints": {}, "limitations": [
                  "These are estimates from the same recorded physical run; no correction changes the recorded motion.",
                  "Two-state output alone is not a global path. The green path additionally uses external wheel/gyro integration and cardinal straight-axis association.",
                  "First usable lateral state establishes a relative offset; no absolute XY coordinate or cell centre is supplied to the two-state wrapper.",
                  "Local updates can move the estimated lateral position; they are state corrections, not directly measured physical displacements.",
                  "Local-coordinate resets/timing are reconstructed heuristically; endpoint distance is a loop-closure consistency indicator, not physical position error.",
                  "Route, when shown, was inferred from the same log and is used only for display; matching it is not independent accuracy validation."],
              "sha256": {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest()
                         for path in [args.states]+([args.route] if args.route else [])+([source_report] if source_report.exists() else [])},
              "implementation_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    for name in LABELS:
        end = comparison[[name+".east_mm", name+".north_mm"]].iloc[-1].to_numpy(float)
        report["endpoints"][name] = {"east_mm": float(end[0]), "north_mm": float(end[1]),
                                    "distance_from_start_mm_not_ground_truth": float(np.linalg.norm(end))}
    (args.output/"report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    route = json.loads(args.route.read_text(encoding="utf-8")) if args.route else None
    draw(args.output, comparison, route, args.title)
    print(json.dumps({"local_samples_used": integrator.used_samples, "endpoints": report["endpoints"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
