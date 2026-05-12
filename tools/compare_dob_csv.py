import argparse
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def load_csv(path):
    # 兼容坏表头
    with open(path, "r", encoding="utf-8") as f:
        header = f.readline().strip().split(",")

    with open(path, "r", encoding="utf-8") as f:
        f.readline()
        first_data = f.readline().strip().split(",")

    n_header = len(header)
    n_data = len(first_data)

    if n_data == n_header + 1:
        if header[-2:] == ["offboard_enablorce_flag", "battery_v"]:
            header = header[:-2] + ["offboard_enabled", "send_force_flag", "battery_v"]
        else:
            header = header + [f"extra_col_{n_data - n_header}"]
        df = pd.read_csv(path, names=header, header=0)
    else:
        df = pd.read_csv(path)

    df["time_s"] = pd.to_numeric(df["time_s"], errors="coerce")
    df = df[np.isfinite(df["time_s"])].copy()

    if len(df) == 0:
        raise ValueError(f"{path} has no valid numeric time_s data")

    df = df.sort_values("time_s").reset_index(drop=True)
    df["t_rel"] = df["time_s"] - df["time_s"].iloc[0]
    return df


def trim_valid_segment(df):
    if "target_valid" in df.columns:
        mask = pd.to_numeric(df["target_valid"], errors="coerce").fillna(0) > 0.5
        if mask.any():
            df = df[mask].copy().reset_index(drop=True)
            df["t_rel"] = df["time_s"] - df["time_s"].iloc[0]
            return df

    if "offboard_enabled" in df.columns:
        mask = pd.to_numeric(df["offboard_enabled"], errors="coerce").fillna(0) > 0.5
        if mask.any():
            df = df[mask].copy().reset_index(drop=True)
            df["t_rel"] = df["time_s"] - df["time_s"].iloc[0]
            return df

    return df


def first_valid_col(df, candidates):
    for c in candidates:
        if c in df.columns:
            return c
    return None


def detect_event_time_from_target(df, pos_threshold=0.15, vel_threshold=0.05, min_hold=5):
    """
    自动寻找目标轨迹开始明显变化的时刻：
    1) 目标位置偏离初值超过阈值
    或
    2) 目标速度超过阈值
    """
    pos_cols = [c for c in ["target_x", "target_y", "target_z"] if c in df.columns]
    vel_cols = [c for c in ["target_vx", "target_vy", "target_vz"] if c in df.columns]

    t = pd.to_numeric(df["t_rel"], errors="coerce").to_numpy()

    if len(t) < 2:
        return 0.0

    pos_metric = None
    if pos_cols:
        pos_arr = []
        for c in pos_cols:
            y = pd.to_numeric(df[c], errors="coerce").to_numpy()
            pos_arr.append(y)
        pos_arr = np.vstack(pos_arr).T

        # 用前 5% 数据估计初值
        n0 = max(3, int(0.05 * len(pos_arr)))
        base = np.nanmedian(pos_arr[:n0, :], axis=0)
        pos_metric = np.linalg.norm(pos_arr - base, axis=1)

    vel_metric = None
    if vel_cols:
        vel_arr = []
        for c in vel_cols:
            y = pd.to_numeric(df[c], errors="coerce").to_numpy()
            vel_arr.append(y)
        vel_arr = np.vstack(vel_arr).T
        vel_metric = np.linalg.norm(vel_arr, axis=1)

    cond = np.zeros(len(df), dtype=bool)
    if pos_metric is not None:
        cond |= np.isfinite(pos_metric) & (pos_metric > pos_threshold)
    if vel_metric is not None:
        cond |= np.isfinite(vel_metric) & (vel_metric > vel_threshold)

    # 要求连续 min_hold 个点满足，防止误触发
    if cond.any():
        count = 0
        for i, flag in enumerate(cond):
            if flag:
                count += 1
                if count >= min_hold:
                    idx = i - min_hold + 1
                    return float(t[idx])
            else:
                count = 0

    return 0.0


def align_by_target_event(df):
    t0 = detect_event_time_from_target(df)
    df = df.copy()
    df["t_align"] = df["t_rel"] - t0
    return df, t0


def interp_series(df, t_new, col, time_col="t_align"):
    if col not in df.columns:
        return np.full_like(t_new, np.nan, dtype=float)

    t = pd.to_numeric(df[time_col], errors="coerce").to_numpy()
    y = pd.to_numeric(df[col], errors="coerce").to_numpy()

    valid = np.isfinite(t) & np.isfinite(y)
    if valid.sum() < 2:
        return np.full_like(t_new, np.nan, dtype=float)

    return np.interp(t_new, t[valid], y[valid])


def common_time_base(df1, df2, dt=None, time_col="t_align"):
    t1 = pd.to_numeric(df1[time_col], errors="coerce").to_numpy()
    t2 = pd.to_numeric(df2[time_col], errors="coerce").to_numpy()

    t1 = t1[np.isfinite(t1)]
    t2 = t2[np.isfinite(t2)]

    if len(t1) == 0 or len(t2) == 0:
        raise ValueError("One CSV has no valid aligned time data.")

    t_start = max(np.min(t1), np.min(t2))
    t_end = min(np.max(t1), np.max(t2))

    if dt is None:
        dts = []
        if len(t1) > 1:
            dts.append(np.median(np.diff(t1)))
        if len(t2) > 1:
            dts.append(np.median(np.diff(t2)))
        dt = min(dts) if dts else 0.02

    if dt <= 0:
        dt = 0.02

    return np.arange(t_start, t_end + dt * 0.5, dt)


def plot_tracking_error_compare(df_no, df_dob, outdir):
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    axis_names = [("x", "X Error [m]"), ("y", "Y Error [m]"), ("z", "Z Error [m]")]

    for ax, (axis, ylabel) in zip(axes, axis_names):
        col = f"err_{axis}"
        if col in df_no.columns:
            ax.plot(df_no["t_align"], df_no[col], color="tab:red", linewidth=1.8, label="Without DOB")
        if col in df_dob.columns:
            ax.plot(df_dob["t_align"], df_dob[col], color="tab:blue", linewidth=1.8, label="With DOB")

        ax.axhline(0.0, color="k", linestyle="--", linewidth=1)
        ax.axvline(0.0, color="gray", linestyle=":", linewidth=1)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend()

    axes[-1].set_xlabel("Aligned Time [s]")
    fig.suptitle("Tracking Error Comparison (Auto-aligned)", fontsize=15)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "tracking_error_comparison_aligned.png"), dpi=200)
    plt.close(fig)


def plot_tracking_position_compare(df_no, df_dob, outdir):
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    axis_names = [("x", "X Position [m]"), ("y", "Y Position [m]"), ("z", "Z Position [m]")]

    for ax, (axis, ylabel) in zip(axes, axis_names):
        uav_col = f"uav_{axis}"
        target_col = f"target_{axis}"

        if target_col in df_no.columns:
            ax.plot(df_no["t_align"], df_no[target_col], "k--", linewidth=2.0, label="Target")
        elif target_col in df_dob.columns:
            ax.plot(df_dob["t_align"], df_dob[target_col], "k--", linewidth=2.0, label="Target")

        if uav_col in df_no.columns:
            ax.plot(df_no["t_align"], df_no[uav_col], color="tab:red", linewidth=1.8, label="Without DOB")
        if uav_col in df_dob.columns:
            ax.plot(df_dob["t_align"], df_dob[uav_col], color="tab:blue", linewidth=1.8, label="With DOB")

        ax.axvline(0.0, color="gray", linestyle=":", linewidth=1)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend()

    axes[-1].set_xlabel("Aligned Time [s]")
    fig.suptitle("Tracking Position Comparison (Auto-aligned)", fontsize=15)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "tracking_position_comparison_aligned.png"), dpi=200)
    plt.close(fig)


def plot_xy_compare(df_no, df_dob, outdir):
    fig, ax = plt.subplots(figsize=(8, 7))

    if "target_x" in df_no.columns and "target_y" in df_no.columns:
        ax.plot(df_no["target_x"], df_no["target_y"], "k--", linewidth=2, label="Target")
    elif "target_x" in df_dob.columns and "target_y" in df_dob.columns:
        ax.plot(df_dob["target_x"], df_dob["target_y"], "k--", linewidth=2, label="Target")

    if "uav_x" in df_no.columns and "uav_y" in df_no.columns:
        ax.plot(df_no["uav_x"], df_no["uav_y"], color="tab:red", linewidth=1.8, label="Without DOB")
    if "uav_x" in df_dob.columns and "uav_y" in df_dob.columns:
        ax.plot(df_dob["uav_x"], df_dob["uav_y"], color="tab:blue", linewidth=1.8, label="With DOB")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("XY Trajectory Comparison")
    ax.grid(True, alpha=0.3)
    ax.axis("equal")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "xy_trajectory_comparison.png"), dpi=200)
    plt.close(fig)


def plot_dob_ff(df_dob, outdir):
    cols = ["dob_ff_x", "dob_ff_y", "dob_ff_z"]
    if not all(c in df_dob.columns for c in cols):
        return

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(df_dob["t_align"], df_dob["dob_ff_x"], label="dob_ff_x", linewidth=1.8)
    ax.plot(df_dob["t_align"], df_dob["dob_ff_y"], label="dob_ff_y", linewidth=1.8)
    ax.plot(df_dob["t_align"], df_dob["dob_ff_z"], label="dob_ff_z", linewidth=1.8)
    ax.axvline(0.0, color="gray", linestyle=":", linewidth=1)
    ax.set_xlabel("Aligned Time [s]")
    ax.set_ylabel("DOB Feedforward [m/s^2]")
    ax.set_title("DOB Feedforward (Auto-aligned)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "dob_feedforward.png"), dpi=200)
    plt.close(fig)


def calc_metrics(err):
    err = np.asarray(err, dtype=float)
    err = err[np.isfinite(err)]
    if len(err) == 0:
        return np.nan, np.nan, np.nan
    rmse = np.sqrt(np.mean(err ** 2))
    mae = np.mean(np.abs(err))
    maxe = np.max(np.abs(err))
    return rmse, mae, maxe


def save_summary(df_no, df_dob, outdir):
    t_common = common_time_base(df_no, df_dob, time_col="t_align")

    lines = []
    lines.append("Tracking Error Metrics Comparison (Auto-aligned)\n")
    lines.append("================================================\n\n")

    for axis in ["x", "y", "z"]:
        col = f"err_{axis}"
        if col not in df_no.columns or col not in df_dob.columns:
            continue

        err_no = interp_series(df_no, t_common, col, time_col="t_align")
        err_dob = interp_series(df_dob, t_common, col, time_col="t_align")

        rmse_no, mae_no, max_no = calc_metrics(err_no)
        rmse_dob, mae_dob, max_dob = calc_metrics(err_dob)

        lines.append(f"[{axis.upper()} axis]\n")
        lines.append(f"Without DOB: RMSE={rmse_no:.4f}, MAE={mae_no:.4f}, MAX={max_no:.4f}\n")
        lines.append(f"With DOB   : RMSE={rmse_dob:.4f}, MAE={mae_dob:.4f}, MAX={max_dob:.4f}\n")
        if np.isfinite(rmse_no) and rmse_no > 1e-12:
            improve = (rmse_no - rmse_dob) / rmse_no * 100.0
            lines.append(f"RMSE Improvement: {improve:.2f}%\n")
        lines.append("\n")

    with open(os.path.join(outdir, "summary_metrics.txt"), "w", encoding="utf-8") as f:
        f.writelines(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no_dob", required=True, help="CSV without DOB")
    parser.add_argument("--dob", required=True, help="CSV with DOB")
    parser.add_argument("--outdir", required=True, help="output dir")
    args = parser.parse_args()

    ensure_dir(args.outdir)

    df_no = load_csv(args.no_dob)
    df_dob = load_csv(args.dob)

    df_no = trim_valid_segment(df_no)
    df_dob = trim_valid_segment(df_dob)

    df_no, t0_no = align_by_target_event(df_no)
    df_dob, t0_dob = align_by_target_event(df_dob)

    print(f"[INFO] no_dob target event time: {t0_no:.3f}s")
    print(f"[INFO] dob    target event time: {t0_dob:.3f}s")

    plot_tracking_error_compare(df_no, df_dob, args.outdir)
    plot_tracking_position_compare(df_no, df_dob, args.outdir)
    plot_xy_compare(df_no, df_dob, args.outdir)
    plot_dob_ff(df_dob, args.outdir)
    save_summary(df_no, df_dob, args.outdir)

    print(f"Saved plots to {args.outdir}")


if __name__ == "__main__":
    main()