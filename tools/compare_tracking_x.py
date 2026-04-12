#!/usr/bin/env python3

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def compute_metrics(df, settle_window=5.0, start_ignore=0.0):
    df = df.copy()
    df = df[df["t"] >= start_ignore]

    if len(df) == 0:
        return {
            "steady_bias": np.nan,
            "rmse": np.nan
        }

    e = df["e_x"].to_numpy()
    rmse = np.sqrt(np.mean(e ** 2))

    t_end = df["t"].iloc[-1]
    df_tail = df[df["t"] >= (t_end - settle_window)]

    if len(df_tail) == 0:
        steady_bias = np.nan
    else:
        steady_bias = df_tail["e_x"].mean()

    return {
        "steady_bias": steady_bias,
        "rmse": rmse
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no_wind_csv", required=True)
    parser.add_argument("--wind_csv", required=True)
    parser.add_argument("--settle_window", type=float, default=5.0,
                        help="稳态偏差统计窗口，默认最后5秒")
    parser.add_argument("--start_ignore", type=float, default=2.0,
                        help="忽略开头若干秒，例如起飞阶段")
    args = parser.parse_args()

    df_no = pd.read_csv(args.no_wind_csv)
    df_wi = pd.read_csv(args.wind_csv)

    m_no = compute_metrics(df_no, args.settle_window, args.start_ignore)
    m_wi = compute_metrics(df_wi, args.settle_window, args.start_ignore)

    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)

    # 图1：x_ref / x_meas
    axes[0].plot(df_no["t"], df_no["x_ref"], label="x_ref (no wind)", linestyle="--")
    axes[0].plot(df_no["t"], df_no["x_meas"], label="x_meas (no wind)")

    axes[0].plot(df_wi["t"], df_wi["x_ref"], label="x_ref (wind)", linestyle="--")
    axes[0].plot(df_wi["t"], df_wi["x_meas"], label="x_meas (wind)")

    axes[0].set_ylabel("X Position (m)")
    axes[0].set_title("X Tracking Comparison")
    axes[0].grid(True)
    axes[0].legend()

    # 图2：e_x
    axes[1].plot(df_no["t"], df_no["e_x"], label=f'No wind, RMSE={m_no["rmse"]:.3f}, bias={m_no["steady_bias"]:.3f}')
    axes[1].plot(df_wi["t"], df_wi["e_x"], label=f'Wind, RMSE={m_wi["rmse"]:.3f}, bias={m_wi["steady_bias"]:.3f}')

    axes[1].axhline(0.0, linestyle="--")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("e_x = x_ref - x_meas (m)")
    axes[1].set_title("X Tracking Error Comparison")
    axes[1].grid(True)
    axes[1].legend()

    plt.tight_layout()
    plt.show()

    print("===== Metrics =====")
    print(f'[No wind] steady-state bias = {m_no["steady_bias"]:.4f} m')
    print(f'[No wind] RMSE              = {m_no["rmse"]:.4f} m')
    print(f'[Wind]    steady-state bias = {m_wi["steady_bias"]:.4f} m')
    print(f'[Wind]    RMSE              = {m_wi["rmse"]:.4f} m')


if __name__ == "__main__":
    main()