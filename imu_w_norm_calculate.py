#声明各个依赖
import pandas as pd
import numpy as np
import os

#输入imu数据集文件
imu_csv = "/home/slam/euroc_bag_split/MH_01_easy_split/imu0_data.csv"

#读取IMU数据
df = pd.read_csv(imu_csv)

#计算模长
df["imu_w_norm"] = np.sqrt(df["gyro_x"]**2 + df["gyro_y"]**2 + df["gyro_z"]**2)

#保存到新文件
save_path = "/home/slam/time_offset_calculate_data/imu0_data_w_norm.csv"
df[["timestamp_sec", "imu_w_norm"]].to_csv(save_path, index=False)

print(f"IMU 角速度模长已保存到 {save_path}")
