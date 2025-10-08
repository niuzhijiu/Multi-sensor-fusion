import pandas as pd
#偏移量bias
b = -0.002
#输入原本的时间戳csv文件
cam_csv = "/home/slam/euroc_bag_split/MH_01_easy_split/cam0_timestamp.csv"
#输出更新后的时间戳csv文件
cam_aligned_csv = "/home/slam/time_offset_calculate_data/cam0_aligned.csv"
# 读取原始相机 csv
df = pd.read_csv(cam_csv)
# 覆盖 timestamp_sec
df["timestamp_sec"] = df["timestamp_sec"] + b
# 保存
df.to_csv(cam_aligned_csv, index=False)
print(f"相机时间戳已整体平移 {b*1000:.2f} ms")
