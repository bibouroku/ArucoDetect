#!/usr/bin/env python3
"""
CSV 数据分析和绘图工具

用法：
    python3 plot_tracking_data.py                    # 使用默认路径
    python3 plot_tracking_data.py /path/to/file.csv # 自定义路径
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
from pathlib import Path

# 默认CSV路径
DEFAULT_CSV_PATH = "/home/wh1te/ros2_ws/src/tracktor-beam/tracking_data.csv"

def load_data(csv_path):
    """加载CSV数据"""
    if not Path(csv_path).exists():
        print(f"❌ 文件不存在: {csv_path}")
        sys.exit(1)
    
    try:
        df = pd.read_csv(csv_path)
        print(f"✅ 成功加载数据: {csv_path}")
        print(f"   数据行数: {len(df)}")
        print(f"   时间跨度: {df['timestamp(s)'].min():.2f} ~ {df['timestamp(s)'].max():.2f} s")
        return df
    except Exception as e:
        print(f"❌ 读取失败: {e}")
        sys.exit(1)

def plot_positions(df, output_path=None):
    """绘制位置对比图"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Position Comparison: Raw vs Filtered', fontsize=16, fontweight='bold')
    
    t = df['timestamp(s)']
    
    # X 位置
    axes[0, 0].plot(t, df['raw_pos_x(m)'], label='Raw', linewidth=1, alpha=0.7)
    axes[0, 0].plot(t, df['filtered_pos_x(m)'], label='Filtered', linewidth=2)
    axes[0, 0].plot(t, df['target_pos_x(m)'], label='Target', linewidth=1.5, linestyle='--')
    axes[0, 0].set_ylabel('Position X (m)', fontsize=11)
    axes[0, 0].legend(loc='best')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_title('X Axis Position')
    
    # Y 位置
    axes[0, 1].plot(t, df['raw_pos_y(m)'], label='Raw', linewidth=1, alpha=0.7)
    axes[0, 1].plot(t, df['filtered_pos_y(m)'], label='Filtered', linewidth=2)
    axes[0, 1].plot(t, df['target_pos_y(m)'], label='Target', linewidth=1.5, linestyle='--')
    axes[0, 1].set_ylabel('Position Y (m)', fontsize=11)
    axes[0, 1].legend(loc='best')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].set_title('Y Axis Position')
    
    # Z 位置
    axes[1, 0].plot(t, df['raw_pos_z(m)'], label='Raw', linewidth=1, alpha=0.7)
    axes[1, 0].plot(t, df['filtered_pos_z(m)'], label='Filtered', linewidth=2)
    axes[1, 0].set_ylabel('Position Z (m)', fontsize=11)
    axes[1, 0].set_xlabel('Time (s)', fontsize=11)
    axes[1, 0].legend(loc='best')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].set_title('Z Axis Position (Height)')
    
    # 2D 位置轨迹
    axes[1, 1].plot(df['raw_pos_x(m)'], df['raw_pos_y(m)'], 'o-', label='Raw', linewidth=1, markersize=3, alpha=0.5)
    axes[1, 1].plot(df['filtered_pos_x(m)'], df['filtered_pos_y(m)'], 'o-', label='Filtered', linewidth=2, markersize=3)
    axes[1, 1].plot(df['target_pos_x(m)'], df['target_pos_y(m)'], 's--', label='Target', linewidth=1.5, markersize=4)
    axes[1, 1].set_xlabel('X (m)', fontsize=11)
    axes[1, 1].set_ylabel('Y (m)', fontsize=11)
    axes[1, 1].legend(loc='best')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].set_title('2D Trajectory (XY Plane)')
    axes[1, 1].axis('equal')
    
    plt.tight_layout()
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"📊 位置对比图已保存: {output_path}")
    plt.show()

def plot_velocities(df, output_path=None):
    """绘制速度对比图"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Velocity Comparison: Raw vs Filtered', fontsize=16, fontweight='bold')
    
    t = df['timestamp(s)']
    
    # X 速度
    axes[0, 0].plot(t, df['raw_vel_x(m/s)'], label='Raw', linewidth=1, alpha=0.7)
    axes[0, 0].plot(t, df['filtered_vel_x(m/s)'], label='Filtered', linewidth=2)
    axes[0, 0].set_ylabel('Velocity X (m/s)', fontsize=11)
    axes[0, 0].legend(loc='best')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_title('X Axis Velocity')
    
    # Y 速度
    axes[0, 1].plot(t, df['raw_vel_y(m/s)'], label='Raw', linewidth=1, alpha=0.7)
    axes[0, 1].plot(t, df['filtered_vel_y(m/s)'], label='Filtered', linewidth=2)
    axes[0, 1].set_ylabel('Velocity Y (m/s)', fontsize=11)
    axes[0, 1].legend(loc='best')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].set_title('Y Axis Velocity')
    
    # Z 速度
    axes[1, 0].plot(t, df['raw_vel_z(m/s)'], label='Raw', linewidth=1, alpha=0.7)
    axes[1, 0].plot(t, df['filtered_vel_z(m/s)'], label='Filtered', linewidth=2)
    axes[1, 0].set_ylabel('Velocity Z (m/s)', fontsize=11)
    axes[1, 0].set_xlabel('Time (s)', fontsize=11)
    axes[1, 0].legend(loc='best')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].set_title('Z Axis Velocity (Vertical)')
    
    # 速度幅值
    raw_speed_xy = np.sqrt(df['raw_vel_x(m/s)']**2 + df['raw_vel_y(m/s)']**2)
    filt_speed_xy = np.sqrt(df['filtered_vel_x(m/s)']**2 + df['filtered_vel_y(m/s)']**2)
    axes[1, 1].plot(t, raw_speed_xy, label='Raw XY Speed', linewidth=1, alpha=0.7)
    axes[1, 1].plot(t, filt_speed_xy, label='Filtered XY Speed', linewidth=2)
    axes[1, 1].set_ylabel('Speed (m/s)', fontsize=11)
    axes[1, 1].set_xlabel('Time (s)', fontsize=11)
    axes[1, 1].legend(loc='best')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].set_title('XY Plane Speed')
    
    plt.tight_layout()
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"📊 速度对比图已保存: {output_path}")
    plt.show()

def plot_accelerations(df, output_path=None):
    """绘制加速度图"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Acceleration Analysis', fontsize=16, fontweight='bold')
    
    t = df['timestamp(s)']
    
    # X 加速度
    axes[0, 0].plot(t, df['accel_x(m/s²)'], linewidth=1.5, color='tab:blue')
    axes[0, 0].axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    axes[0, 0].set_ylabel('Acceleration (m/s²)', fontsize=11)
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_title('X Axis Acceleration')
    
    # Y 加速度
    axes[0, 1].plot(t, df['accel_y(m/s²)'], linewidth=1.5, color='tab:orange')
    axes[0, 1].axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    axes[0, 1].set_ylabel('Acceleration (m/s²)', fontsize=11)
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].set_title('Y Axis Acceleration')
    
    # Z 加速度
    axes[1, 0].plot(t, df['accel_z(m/s²)'], linewidth=1.5, color='tab:green')
    axes[1, 0].axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    axes[1, 0].set_ylabel('Acceleration (m/s²)', fontsize=11)
    axes[1, 0].set_xlabel('Time (s)', fontsize=11)
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].set_title('Z Axis Acceleration')
    
    # 加速度幅值
    accel_mag = np.sqrt(df['accel_x(m/s²)']**2 + df['accel_y(m/s²)']**2 + df['accel_z(m/s²)']**2)
    axes[1, 1].plot(t, accel_mag, linewidth=1.5, color='tab:red', label='Magnitude')
    axes[1, 1].fill_between(t, 0, accel_mag, alpha=0.3, color='tab:red')
    axes[1, 1].set_ylabel('Acceleration Magnitude (m/s²)', fontsize=11)
    axes[1, 1].set_xlabel('Time (s)', fontsize=11)
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].set_title('Total Acceleration Magnitude')
    
    plt.tight_layout()
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"📊 加速度图已保存: {output_path}")
    plt.show()

def plot_tracking_error(df, output_path=None):
    """绘制跟踪误差"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Tracking Error Analysis', fontsize=16, fontweight='bold')
    
    t = df['timestamp(s)']
    
    # 位置误差
    pos_error_x = df['filtered_pos_x(m)'] - df['target_pos_x(m)']
    pos_error_y = df['filtered_pos_y(m)'] - df['target_pos_y(m)']
    pos_error_mag = np.sqrt(pos_error_x**2 + pos_error_y**2)
    
    axes[0, 0].plot(t, pos_error_x, label='Error X', linewidth=1.5)
    axes[0, 0].plot(t, pos_error_y, label='Error Y', linewidth=1.5)
    axes[0, 0].axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    axes[0, 0].set_ylabel('Position Error (m)', fontsize=11)
    axes[0, 0].legend(loc='best')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_title('Position Error (X, Y)')
    
    # 位置误差幅值
    axes[0, 1].plot(t, pos_error_mag, linewidth=2, color='tab:red')
    axes[0, 1].fill_between(t, 0, pos_error_mag, alpha=0.2, color='tab:red')
    axes[0, 1].set_ylabel('Position Error Magnitude (m)', fontsize=11)
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].set_title('2D Tracking Error')
    mean_error = pos_error_mag.mean()
    max_error = pos_error_mag.max()
    axes[0, 1].text(0.5, 0.95, f'Mean: {mean_error:.4f} m\nMax: {max_error:.4f} m',
                    transform=axes[0, 1].transAxes, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
                    fontsize=10)
    
    # 按状态统计误差
    state_errors = {}
    for state in df['state'].unique():
        mask = df['state'] == state
        if mask.sum() > 0:
            state_error = np.sqrt(
                (df[mask]['filtered_pos_x(m)'] - df[mask]['target_pos_x(m)'])**2 +
                (df[mask]['filtered_pos_y(m)'] - df[mask]['target_pos_y(m)'])**2
            )
            state_errors[state] = state_error.mean()
    
    if state_errors:
        states = list(state_errors.keys())
        errors = list(state_errors.values())
        colors = ['red' if s == 'TRACKING' else 'blue' for s in states]
        axes[1, 0].bar(states, errors, color=colors, alpha=0.7)
        axes[1, 0].set_ylabel('Mean Error (m)', fontsize=11)
        axes[1, 0].set_title('Mean Error by State')
        axes[1, 0].grid(True, alpha=0.3, axis='y')
        for i, (s, e) in enumerate(zip(states, errors)):
            axes[1, 0].text(i, e, f'{e:.4f}', ha='center', va='bottom')
    
    # 误差分布
    axes[1, 1].hist(pos_error_mag, bins=50, color='tab:blue', alpha=0.7, edgecolor='black')
    axes[1, 1].axvline(mean_error, color='red', linestyle='--', linewidth=2, label=f'Mean: {mean_error:.4f} m')
    axes[1, 1].set_xlabel('Position Error (m)', fontsize=11)
    axes[1, 1].set_ylabel('Frequency', fontsize=11)
    axes[1, 1].legend(loc='best')
    axes[1, 1].grid(True, alpha=0.3, axis='y')
    axes[1, 1].set_title('Error Distribution')
    
    plt.tight_layout()
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"📊 误差分析图已保存: {output_path}")
    plt.show()

def print_statistics(df):
    """打印数据统计信息"""
    print("\n" + "="*60)
    print("📊 数据统计信息")
    print("="*60)
    
    # 时间统计
    duration = df['timestamp(s)'].max() - df['timestamp(s)'].min()
    print(f"\n⏱️  时间统计:")
    print(f"   总时长: {duration:.2f} s")
    print(f"   采样率: {len(df)/duration:.1f} Hz (理论30Hz)")
    
    # 状态统计
    print(f"\n🎛️  状态统计:")
    for state in df['state'].unique():
        count = (df['state'] == state).sum()
        duration_state = duration * count / len(df)
        print(f"   {state}: {count} 次 ({duration_state:.2f} s)")
    
    # 位置统计
    print(f"\n📍 位置统计 (滤波):")
    print(f"   X: {df['filtered_pos_x(m)'].min():.4f} ~ {df['filtered_pos_x(m)'].max():.4f} m")
    print(f"   Y: {df['filtered_pos_y(m)'].min():.4f} ~ {df['filtered_pos_y(m)'].max():.4f} m")
    print(f"   Z: {df['filtered_pos_z(m)'].min():.4f} ~ {df['filtered_pos_z(m)'].max():.4f} m")
    
    # 速度统计
    print(f"\n🚀 速度统计 (滤波):")
    print(f"   X: {df['filtered_vel_x(m/s)'].min():.4f} ~ {df['filtered_vel_x(m/s)'].max():.4f} m/s")
    print(f"   Y: {df['filtered_vel_y(m/s)'].min():.4f} ~ {df['filtered_vel_y(m/s)'].max():.4f} m/s")
    print(f"   Z: {df['filtered_vel_z(m/s)'].min():.4f} ~ {df['filtered_vel_z(m/s)'].max():.4f} m/s")
    
    # 加速度统计
    print(f"\n⚡ 加速度统计:")
    accel_mag = np.sqrt(df['accel_x(m/s²)']**2 + df['accel_y(m/s²)']**2 + df['accel_z(m/s²)']**2)
    print(f"   幅值: {accel_mag.min():.4f} ~ {accel_mag.max():.4f} m/s²")
    print(f"   均值: {accel_mag.mean():.4f} m/s²")
    
    # 跟踪误差统计
    tracking_data = df[df['state'] == 'TRACKING']
    if len(tracking_data) > 0:
        pos_error = np.sqrt(
            (tracking_data['filtered_pos_x(m)'] - tracking_data['target_pos_x(m)'])**2 +
            (tracking_data['filtered_pos_y(m)'] - tracking_data['target_pos_y(m)'])**2
        )
        print(f"\n🎯 跟踪误差 (仅 TRACKING 状态):")
        print(f"   均值: {pos_error.mean():.4f} m")
        print(f"   最大: {pos_error.max():.4f} m")
        print(f"   最小: {pos_error.min():.4f} m")
    
    print("\n" + "="*60)

def main():
    # 获取CSV路径
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        csv_path = DEFAULT_CSV_PATH
    
    # 加载数据
    df = load_data(csv_path)
    
    # 打印统计信息
    print_statistics(df)
    
    # 确定输出目录
    output_dir = Path(csv_path).parent
    
    # 绘制图表
    print("\n🎨 正在生成图表...")
    plot_positions(df, output_dir / "01_positions_comparison.png")
    plot_velocities(df, output_dir / "02_velocities_comparison.png")
    plot_accelerations(df, output_dir / "03_accelerations.png")
    plot_tracking_error(df, output_dir / "04_tracking_error.png")
    
    print("\n✅ 所有图表已生成完毕!")
    print(f"📁 图表保存在: {output_dir}")

if __name__ == '__main__':
    main()
