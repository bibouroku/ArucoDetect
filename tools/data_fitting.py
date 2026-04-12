import numpy as np
import pandas as pd
from pathlib import Path
from sklearn.linear_model import LinearRegression
from scipy.spatial.transform import Rotation as R

# =========================
# 1. 配置区：改成你的CSV路径和列名
# =========================

BASE_DIR = Path(__file__).resolve().parent
CSV_PATH = BASE_DIR / "data_collect.csv"

COL_RPM = ["u0", "u1", "u2", "u3"]
COL_BAT = "voltage_v"

COL_ACC = ["acc_x", "acc_y", "acc_z"]

# 四元数顺序：这里假设你的CSV里是 w,x,y,z
COL_QUAT = ["quat_w", "quat_x", "quat_y", "quat_z"]

# 无人机质量，单位 kg
MASS = 2.0

# 如果你的acc已经“去重力”，设为 True
# 如果你的acc仍然包含重力，设为 False
ACC_IS_LINEAR = True

# 是否只取“近悬停/平稳”数据参与拟合
USE_FILTER = True

# 平稳筛选阈值
MAX_ACC_NORM = 3.0      # m/s^2
MAX_TILT_DEG = 30.0     # deg

# =========================
# 2. 工具函数
# =========================
def quat_wxyz_to_rotmat(qw, qx, qy, qz):
    # scipy用的是 [x, y, z, w]
    rot = R.from_quat([qx, qy, qz, qw])
    return rot.as_matrix()

def compute_thrust_target(row):
    """
    根据加速度 + 姿态，反推当前总推力大小 f_target
    返回单位：N
    """
    acc = np.array([row[COL_ACC[0]], row[COL_ACC[1]], row[COL_ACC[2]]], dtype=float)

    if ACC_IS_LINEAR:
        # acc 是去重力后的线加速度
        total_force_world = MASS * acc + np.array([0.0, 0.0, MASS * 9.81])
    else:
        # acc 仍包含重力
        total_force_world = MASS * acc

    qw, qx, qy, qz = row[COL_QUAT[0]], row[COL_QUAT[1]], row[COL_QUAT[2]], row[COL_QUAT[3]]
    R_bw = quat_wxyz_to_rotmat(qw, qx, qy, qz)

    # 四旋翼推力通常沿机体系 -zb
    thrust_dir_world = -R_bw[:, 2]

    # 把世界系总力投影到推力方向上，得到总推力大小
    f_target = -np.dot(total_force_world, thrust_dir_world)

    return f_target

def build_features_from_row(row):
    u = np.array([row[c] for c in COL_RPM], dtype=float)
    battery = float(row[COL_BAT])

    sum_u = np.sum(u)
    sum_u2 = np.sum(u ** 2)

    return np.array([
        sum_u,
        sum_u2,
        battery,
        battery ** 2,
        sum_u * battery,
        sum_u2 * battery
    ], dtype=float)

def compute_tilt_deg(row):
    qw, qx, qy, qz = row[COL_QUAT[0]], row[COL_QUAT[1]], row[COL_QUAT[2]], row[COL_QUAT[3]]
    rot = R.from_quat([qx, qy, qz, qw])
    roll, pitch, yaw = rot.as_euler("xyz", degrees=True)
    return max(abs(roll), abs(pitch))

# =========================
# 3. 读CSV
# =========================
df = pd.read_csv(CSV_PATH)

# 丢掉缺失值
needed_cols = COL_RPM + [COL_BAT] + COL_ACC + COL_QUAT
df = df.dropna(subset=needed_cols).copy()

# =========================
# 4. 计算标签 f_target
# =========================
df["f_target"] = df.apply(compute_thrust_target, axis=1)
df["f_target"] = df["f_target"].rolling(window=5, center=True, min_periods=1).mean()

DELAY = 3   # 先试 1,2,3,4

df["f_target"] = df["f_target"].shift(-DELAY)
df = df.dropna().copy()
# =========================
# 5. 可选：筛掉不平稳数据
# =========================
if USE_FILTER:
    acc_norm = np.linalg.norm(df[COL_ACC].values, axis=1)
    tilt_deg = df.apply(compute_tilt_deg, axis=1).values

    mask = (acc_norm < MAX_ACC_NORM) & (tilt_deg < MAX_TILT_DEG) & (df["f_target"] > 0.0)
    df_fit = df[mask].copy()
else:
    df_fit = df.copy()

print(f"总样本数: {len(df)}")
print(f"用于拟合样本数: {len(df_fit)}")

# =========================
# 6. 构造特征矩阵 X
# =========================
X = np.vstack(df_fit.apply(build_features_from_row, axis=1).values)
y = df_fit["f_target"].values

# =========================
# 7. 最小二乘拟合
# =========================
model = LinearRegression(fit_intercept=True)
model.fit(X, y)

y_pred = model.predict(X)

rmse = np.sqrt(np.mean((y - y_pred) ** 2))
mae = np.mean(np.abs(y - y_pred))
r2 = model.score(X, y)

print("\n===== 拟合结果 =====")
print("coefficients =")
print(model.coef_)
print("intercept =")
print(model.intercept_)

print("\n===== 拟合误差 =====")
print(f"RMSE = {rmse:.4f} N")
print(f"MAE  = {mae:.4f} N")
print(f"R^2  = {r2:.6f}")

# =========================
# 8. 生成可直接复制到C++的格式
# =========================
coef = model.coef_
intercept = model.intercept_

print("\n===== C++ 可直接复制 =====")
print("coefficients_ <<")
for i, c in enumerate(coef):
    if i < len(coef) - 1:
        print(f"    {c:.12e},")
    else:
        print(f"    {c:.12e};")

print(f"intercept_ = {intercept:.12e};")