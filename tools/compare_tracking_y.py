#!/usr/bin/env python3

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def compute_metrics(df, settle_window=5.0, start_ignore=0.0):
    df = df.copy()
    df = df[df["t"] >= start_ignore]

    if len(df) == 0:
        return {"steady_bias": np.nan, "rmse": np.nan}

    e = df["e_y"].to_numpy()
    rmse = np.sqrt(np.mean(e ** 2))

    t_end = df["t"].iloc[-1]
    df_tail = df[df["t"] >= (t_end - settle_window)]
    steady_bias = df_tail["e_y"].mean() if len(df_tail) > 0 else np.nan

    return {
        "steady_bias": steady_bias,
        "rmse": rmse
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no_dob_csv", required=True)
    parser.add_argument("--dob_csv", required=True)
    parser.add_argument("--settle_window", type=float, default=5.0)
    parser.add_argument("--start_ignore", type=float, default=3.0)
    args = parser.parse_args()

    df_no = pd.read_csv(args.no_dob_csv)
    df_do = pd.read_csv(args.dob_csv)

    m_no = compute_metrics(df_no, args.settle_window, args.start_ignore)
    m_do = compute_metrics(df_do, args.settle_window, args.start_ignore)

    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)

    # 图1：y_ref / y_meas
    axes[0].plot(df_no["t"], df_no["y_ref"], label="y_ref (no DOB)", linestyle="--")
    axes[0].plot(df_no["t"], df_no["y_meas"], label="y_meas (no DOB)")

    axes[0].plot(df_do["t"], df_do["y_ref"], label="y_ref (DOB)", linestyle="--")
    axes[0].plot(df_do["t"], df_do["y_meas"], label="y_meas (DOB)")

    axes[0].set_ylabel("Y Position (m)")
    axes[0].set_title("Y Tracking Comparison Under Wind")
    axes[0].grid(True)
    axes[0].legend()

    # 图2：e_y
    axes[1].plot(
        df_no["t"], df_no["e_y"],
        label=f'No DOB'
    )
    axes[1].plot(
        df_do["t"], df_do["e_y"],
        label=f'DOB'
    )

    axes[1].axhline(0.0, linestyle="--")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("e_y = y_ref - y_meas (m)")
    axes[1].set_title("Y Tracking Error Comparison")
    axes[1].grid(True)
    axes[1].legend()

    plt.tight_layout()
    plt.show()

    print("===== Metrics =====")
    print(f'[No DOB] steady-state bias = {m_no["steady_bias"]:.4f} m')
    print(f'[No DOB] RMSE              = {m_no["rmse"]:.4f} m')
    print(f'[DOB]    steady-state bias = {m_do["steady_bias"]:.4f} m')
    print(f'[DOB]    RMSE              = {m_do["rmse"]:.4f} m')


if __name__ == "__main__":
    main()