#!/usr/bin/env python3
import argparse
from pathlib import Path
from typing import Iterable, Tuple, List

import pandas as pd
import matplotlib.pyplot as plt


REQUIRED_BASE_COLUMNS = [
    "time_s",
    "err_x",
    "err_y",
    "err_z",
    "cmd_vx",
    "cmd_vy",
    "cmd_vz",
    "dob_ff_x",
    "dob_ff_y",
    "dob_ff_z",
    "target_x",
    "target_y",
    "uav_x",
    "uav_y",
]

RAW_FILTERED_COLUMNS = [
    "raw_x",
    "raw_y",
    "raw_z",
    "filtered_x",
    "filtered_y",
    "filtered_z",
]

STATE_COLUMN_CANDIDATES = ["state", "state_name", "fsm_state"]


def validate_columns(df: pd.DataFrame, columns: Iterable[str], context: str) -> List[str]:
    missing = [c for c in columns if c not in df.columns]
    if missing:
        print(f"[WARN] Missing columns for {context}: {', '.join(missing)}")
    return missing


def add_xy_error(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df["err_xy"] = (df["err_x"] ** 2 + df["err_y"] ** 2) ** 0.5
    return df


def get_time_axis(df: pd.DataFrame) -> pd.Series:
    return df["time_s"] - df["time_s"].iloc[0]


def get_state_column(df: pd.DataFrame) -> str | None:
    for name in STATE_COLUMN_CANDIDATES:
        if name in df.columns:
            return name
    return None


def find_state_segments(df: pd.DataFrame) -> List[Tuple[float, float, str]]:
    state_col = get_state_column(df)
    if state_col is None or df.empty:
        return []

    t = get_time_axis(df)
    states = df[state_col].astype(str).fillna("UNKNOWN")
    segments: List[Tuple[float, float, str]] = []

    start_idx = 0
    current_state = states.iloc[0]
    for i in range(1, len(df)):
        if states.iloc[i] != current_state:
            segments.append((float(t.iloc[start_idx]), float(t.iloc[i - 1]), current_state))
            start_idx = i
            current_state = states.iloc[i]
    segments.append((float(t.iloc[start_idx]), float(t.iloc[len(df) - 1]), current_state))
    return segments


def annotate_state_boundaries(ax: plt.Axes, segments: List[Tuple[float, float, str]]) -> None:
    if not segments:
        return

    ymin, ymax = ax.get_ylim()
    y_text = ymax - 0.08 * (ymax - ymin) if ymax > ymin else ymax

    for i, (t0, t1, state) in enumerate(segments):
        if i > 0:
            ax.axvline(t0, color="gray", linestyle="--", linewidth=0.8, alpha=0.6)
        t_mid = 0.5 * (t0 + t1)
        ax.text(t_mid, y_text, state, ha="center", va="top", fontsize=8,
                bbox={"boxstyle": "round,pad=0.2", "facecolor": "white", "alpha": 0.65, "edgecolor": "none"})


def plot_tracking_error(df: pd.DataFrame, outdir: Path) -> None:
    t = get_time_axis(df)
    segments = find_state_segments(df)

    plt.figure(figsize=(10, 5))
    plt.plot(t, df["err_x"], label="err_x")
    plt.plot(t, df["err_y"], label="err_y")
    plt.plot(t, df["err_z"], label="err_z")
    plt.plot(t, df["err_xy"], label="err_xy", linewidth=2.0)
    ax = plt.gca()
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error [m]")
    ax.set_title("Tracking Error")
    ax.legend()
    ax.grid(True)
    annotate_state_boundaries(ax, segments)
    plt.tight_layout()
    plt.savefig(outdir / "tracking_error.png", dpi=200)
    plt.close()


def plot_cmd_velocity(df: pd.DataFrame, outdir: Path) -> None:
    t = get_time_axis(df)
    plt.figure(figsize=(10, 5))
    plt.plot(t, df["cmd_vx"], label="cmd_vx")
    plt.plot(t, df["cmd_vy"], label="cmd_vy")
    plt.plot(t, df["cmd_vz"], label="cmd_vz")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity Command [m/s]")
    plt.title("Velocity Commands")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(outdir / "cmd_velocity.png", dpi=200)
    plt.close()


def plot_dob_ff(df: pd.DataFrame, outdir: Path) -> None:
    t = get_time_axis(df)
    plt.figure(figsize=(10, 5))
    plt.plot(t, df["dob_ff_x"], label="dob_ff_x")
    plt.plot(t, df["dob_ff_y"], label="dob_ff_y")
    plt.plot(t, df["dob_ff_z"], label="dob_ff_z")
    plt.xlabel("Time [s]")
    plt.ylabel("DOB Feedforward [m/s^2]")
    plt.title("DOB Feedforward")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(outdir / "dob_feedforward.png", dpi=200)
    plt.close()


def plot_top_view(df: pd.DataFrame, outdir: Path) -> None:
    plt.figure(figsize=(6, 6))
    plt.plot(df["target_x"], df["target_y"], label="target")
    plt.plot(df["uav_x"], df["uav_y"], label="uav")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Top View Trajectory")
    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(outdir / "top_view_trajectory.png", dpi=200)
    plt.close()


def plot_raw_vs_filtered_pose(df: pd.DataFrame, outdir: Path) -> bool:
    missing = validate_columns(df, RAW_FILTERED_COLUMNS, "raw vs filtered pose plot")
    if missing:
        return False

    t = get_time_axis(df)
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    axes[0].plot(t, df["raw_x"], label="raw_x", alpha=0.75)
    axes[0].plot(t, df["filtered_x"], label="filtered_x", linewidth=1.8)
    axes[0].set_ylabel("X [m]")
    axes[0].set_title("Raw vs Filtered Pose")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(t, df["raw_y"], label="raw_y", alpha=0.75)
    axes[1].plot(t, df["filtered_y"], label="filtered_y", linewidth=1.8)
    axes[1].set_ylabel("Y [m]")
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(t, df["raw_z"], label="raw_z", alpha=0.75)
    axes[2].plot(t, df["filtered_z"], label="filtered_z", linewidth=1.8)
    axes[2].set_ylabel("Z [m]")
    axes[2].set_xlabel("Time [s]")
    axes[2].legend()
    axes[2].grid(True)

    fig.tight_layout()
    fig.savefig(outdir / "raw_vs_filtered_pose.png", dpi=200)
    plt.close(fig)
    return True


def plot_target_velocity_estimate(df: pd.DataFrame, outdir: Path) -> bool:
    needed = ["target_vx_ff", "target_vy_ff"]
    missing = validate_columns(df, needed, "target velocity plot")
    if missing:
        return False

    t = get_time_axis(df)
    plt.figure(figsize=(10, 5))
    plt.plot(t, df["target_vx_ff"], label="target_vx_ff")
    plt.plot(t, df["target_vy_ff"], label="target_vy_ff")
    if "estimated_vel_x" in df.columns:
        plt.plot(t, df["estimated_vel_x"], label="estimated_vel_x", linestyle="--")
    if "estimated_vel_y" in df.columns:
        plt.plot(t, df["estimated_vel_y"], label="estimated_vel_y", linestyle="--")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.title("Target Velocity Estimate / Feedforward")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(outdir / "target_velocity_estimate.png", dpi=200)
    plt.close()
    return True


def plot_basic(df: pd.DataFrame, outdir: Path) -> dict:
    outdir.mkdir(parents=True, exist_ok=True)
    plot_tracking_error(df, outdir)
    plot_cmd_velocity(df, outdir)
    plot_dob_ff(df, outdir)
    plot_top_view(df, outdir)

    produced = {
        "raw_vs_filtered_pose": plot_raw_vs_filtered_pose(df, outdir),
        "target_velocity_estimate": plot_target_velocity_estimate(df, outdir),
    }
    return produced


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", help="Path to logged CSV file")
    parser.add_argument("--outdir", default="analysis_plots", help="Directory to save plots")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)
    validate_columns(df, REQUIRED_BASE_COLUMNS, "base plots")
    df = add_xy_error(df)
    outdir = Path(args.outdir)
    produced = plot_basic(df, outdir)

    summary = {
        "rows": len(df),
        "mean_err_xy_m": df["err_xy"].mean(),
        "rmse_err_xy_m": (df["err_xy"] ** 2).mean() ** 0.5,
        "max_err_xy_m": df["err_xy"].max(),
        "mean_abs_err_z_m": df["err_z"].abs().mean(),
        "raw_vs_filtered_pose_plot": produced["raw_vs_filtered_pose"],
        "target_velocity_estimate_plot": produced["target_velocity_estimate"],
    }
    summary_path = outdir / "summary.txt"
    with summary_path.open("w", encoding="utf-8") as f:
        for k, v in summary.items():
            f.write(f"{k}: {v}\n")
    print(f"Plots and summary saved to {outdir}")


if __name__ == "__main__":
    main()