# -*- coding: utf-8 -*-
#声明依赖
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

#输入参与计算的角速度数据来源
imu_csv = "/home/slam/euroc_bag_split/MH_01_easy_split/imu0_data.csv"
vis_csv = "/home/slam/time_offset_calculate_data/visual_angular_velocity_mid.csv"

#读取数据
imu_df = pd.read_csv(imu_csv)
vis_df = pd.read_csv(vis_csv)

#提取IMU时间戳和角速度分量
imu_t = imu_df["timestamp_sec"].values
imu_wx = imu_df["gyro_x"].values
imu_wy = imu_df["gyro_y"].values
imu_wz = imu_df["gyro_z"].values

#提取相机时间戳和角速度分量
vis_t = vis_df["timestamp"].values
vis_wx = vis_df["wx"].values
vis_wy = vis_df["wy"].values
vis_wz = vis_df["wz"].values

#bias搜索范围[-25, +25] ms，步长 1 ms
search_range = np.arange(-0.025, 0.025+0.001, 0.001) 


#定义函数计算归一化互相关
def compute_best_bias(imu_t, imu_s, vis_t, vis_s, search_range):
    #IMU信号去均值，避免DC偏置影响
    imu_s_centered = imu_s - np.mean(imu_s)
    #当前最佳互相关
    best_rho = -1
    #当前最佳互相关所对应的bias
    best_bias = 0
    rhos = []
    #开始遍历搜索范围的所有bias
    for bias in search_range:
        #平移相机时间戳
        shifted_t = vis_t + bias
        #插值相机的角速度到imu角速度序列
        f_interp = interp1d(shifted_t, vis_s, kind="linear",
                            bounds_error=False, fill_value="extrapolate")
        vis_interp = f_interp(imu_t)
        #相机信号去均值
        vis_centered = vis_interp - np.mean(vis_interp)
        #相关系数计算公式
        numerator = np.dot(imu_s_centered, vis_centered)
        denominator = np.linalg.norm(imu_s_centered) * np.linalg.norm(vis_centered)
        rho = numerator / denominator if denominator > 0 else 0
        rhos.append(rho)
        #互相关和bias更新
        if rho > best_rho:
            best_rho = rho
            best_bias = bias
    return best_bias, best_rho, rhos

#分别计算 x, y, z 三个轴的偏移
axes = ["x", "y", "z"]
#提取imu和camera的各分量角速度
imu_signals = [imu_wx, imu_wy, imu_wz]
vis_signals = [vis_wx, vis_wy, vis_wz]

results = {}
#把三个列表按位置配对第一轮取"x", imu_wx, vis_wx，第二轮 "y", imu_wy, vis_wy，第三轮 "z", imu_wz, vis_wz
for axis, imu_s, vis_s in zip(axes, imu_signals, vis_signals):
    #调用compute_best_bias函数
    bias, rho, rhos = compute_best_bias(imu_t, imu_s, vis_t, vis_s, search_range)
    #返回对应的值
    results[axis] = (bias, rho)
    print(f"轴 {axis}: 最佳时间偏移 b̂ = {bias*1000:.2f} ms")


#选取相关性最高的轴
best_axis = max(results, key=lambda k: results[k][1])
best_bias, best_rho = results[best_axis]
print("\n>>> 最佳结果来自轴 {}: 偏移 = {:.2f} ms".format(
    best_axis, best_bias*1000, best_rho))

