#导入各个依赖
import cv2
import numpy as np
import os
import csv
import pandas as pd
from scipy.spatial.transform import Rotation as R

#定义一个名字叫estimate_visual_angular_velocity的函数（前一帧，后一帧，内参矩阵，帧间隔）
def estimate_visual_angular_velocity(img1, img2, K, dt):
    #orb算法提取特征点（最多多少个）
    orb = cv2.ORB_create(2000)
    #得到第一帧的关键点kp1和描述子des1
    kp1, des1 = orb.detectAndCompute(img1, None)
    #得到第二帧的关键点kp2和描述子des2
    kp2, des2 = orb.detectAndCompute(img2, None)
    #如果得不到返回None
    if des1 is None or des2 is None:
        return None

    #特征匹配使用Brute-Force匹配器，距离度量用Hamming，crossCheck=True保证匹配双向一致，结果按匹配距离升序排序
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    matches = sorted(matches, key=lambda x: x.distance)

    #从匹配结果中提取两帧图像中的点坐标，匹配点<5就放弃返回None。
    pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
    pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])
    if len(pts1) < 5:
        return None

    #用RANSAC算法估计本质矩阵E，记内点作mask即符合相机运动的几何约束的匹配点
    E, mask = cv2.findEssentialMat(pts1, pts2, K, cv2.RANSAC, 0.999, 1.0)
    if E is None:
        return None

    #筛选并且只保留内点
    pts1_inliers = pts1[mask.ravel() == 1]
    pts2_inliers = pts2[mask.ravel() == 1]
    if len(pts1_inliers) < 5:
        return None

    #由E恢复相对旋转矩阵R_rel
    _, R_rel, _, _ = cv2.recoverPose(E, pts1_inliers, pts2_inliers, K)

    #由旋转矩阵求旋转向量rotvec即Δθ，有SciPy兼容性设置，曾在这里遇到了兼容性问题，新版本from_matrix，老版本from_dcm
    try:
        rotvec = R.from_matrix(R_rel).as_rotvec()
    except AttributeError:
        rotvec = R.from_dcm(R_rel).as_rotvec()  

    #角速度=Δθ/Δt
    omega = rotvec / dt
    return omega


if __name__ == "__main__":
    #相机内参矩阵K，cx和cy通过查阅产品说明得到，fx和fy是6mm时候的
    K = np.array([[1000.0,   0.0, 376.0],
                  [  0.0, 1000.0, 240.0],
                  [  0.0,   0.0,   1.0]])

    #图像文件夹
    folder = "/home/slam/euroc_bag_split/MH_01_easy_split/cam0/"

    #从时间戳读取时间戳和文件名
    csv_path = "/home/slam/euroc_bag_split/MH_01_easy_split/cam0_timestamp.csv"
    #需要对应表头：timestamp_sec,image_filename
    df = pd.read_csv(csv_path)
    
    #初始化一个空列表
    results = []
    #从第一帧开始遍历
    for i in range(len(df) - 1):
        #读取各组前后帧时间戳t1和t2计算dt
        t1, file1 = df.iloc[i]["timestamp_sec"], df.iloc[i]["image_filename"]
        t2, file2 = df.iloc[i + 1]["timestamp_sec"], df.iloc[i + 1]["image_filename"]
        dt = t2 - t1
        if dt <= 0:
            continue

        img1_path = os.path.join(folder, file1)
        img2_path = os.path.join(folder, file2)

        #读取图像
        img1 = cv2.imread(img1_path, cv2.IMREAD_GRAYSCALE)
        img2 = cv2.imread(img2_path, cv2.IMREAD_GRAYSCALE)
        if img1 is None or img2 is None:
            continue

        #用定义的函数去计算得到角速度
        omega = estimate_visual_angular_velocity(img1, img2, K, dt)
        if omega is not None:
            omega_norm = np.linalg.norm(omega)
            #时间戳取前后帧的中点
            t_mid = 0.5 * (t1 + t2)  
            results.append([t_mid, omega[0], omega[1], omega[2], omega_norm])
            print(f"t={t_mid:.6f}, dt={dt:.6f}s, ω={omega}, |ω|={omega_norm:.6f}")

    #输出结果为CSV
    save_path = "/home/slam/time_offset_calculate_data/visual_angular_velocity_mid.csv"
    os.makedirs(os.path.dirname(save_path), exist_ok=True)

    #里面有时间，该时间的三轴角速度，角速度模值
    with open(save_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "wx", "wy", "wz", "w_norm"])
        writer.writerows(results)

    print(f"结果已保存到 {save_path}")
    #比图像总数少1，因为计算的是相对的角速度所以最后一帧没有角速度
    print(f"共写入 {len(results)} 条角速度数据")