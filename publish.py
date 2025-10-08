#!/usr/bin/env python3
#声明依赖
import rospy
import csv
import cv2
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu

def main():
    #ros节点名字叫pub_node
    rospy.init_node("pub_node")

    #读取相关参数，相机文件夹、相机修正时间戳、imu数据
    cam_csv = rospy.get_param("~cam_csv", "/home/slam/time_offset_calculate_data/cam0_aligned.csv")
    image_dir = rospy.get_param("~image_dir", "/home/slam/euroc_bag_split/MH_01_easy_split/cam0/")
    imu_csv = rospy.get_param("~imu_csv", "/home/slam/euroc_bag_split/MH_01_easy_split/imu0_data.csv")

    #进入publish发布过程，topic名字分别是cam_image和imu_data
    pub_img = rospy.Publisher("/cam_image", Image, queue_size=10)
    pub_imu = rospy.Publisher("/imu_data", Imu, queue_size=2000)

    #opencv和ros转换
    bridge = CvBridge()

    #读取相机CSV文件内容，格式是多行两列
    with open(cam_csv, "r") as f:
        cam_reader = csv.reader(f)
        next(cam_reader)
        cam_data = [(float(row[0]), row[1]) for row in cam_reader]

    #读取IMU CSV文件内容
    with open(imu_csv, "r") as f:
        imu_reader = csv.reader(f)
        header = next(imu_reader)
        #搜寻表头关键字，以免格式不统一
        ts_idx = [i for i, h in enumerate(header) if "time" in h.lower()][0]
        acc_idx = [i for i, h in enumerate(header) if "acc" in h.lower()]
        gyro_idx = [i for i, h in enumerate(header) if any(k in h.lower() for k in ["gyro","ang","omega"])]
        #格式是多行七列
        imu_data = [
            (float(row[ts_idx]),
             [float(row[i]) for i in acc_idx],
             [float(row[i]) for i in gyro_idx])
            for row in imu_reader
        ]

    #初始化两个读取指针
    i_cam, i_imu = 0, 0

    #高速循环，根据时间戳控制节奏，rospy.Rate(1000)防止CPU占满
    rate = rospy.Rate(1000)  

    #主循环：当ROS未关闭且两个数据表至少有一个还有剩余数据时继续运行
    while not rospy.is_shutdown() and (i_cam < len(cam_data) or i_imu < len(imu_data)):
        #如果相机还没发完，取当前的相机时间戳，否则置为无穷大
        t_cam = cam_data[i_cam][0] if i_cam < len(cam_data) else float("inf")
        #如果IMU还没发完，取当前的imu时间戳，否则置为无穷大
        t_imu = imu_data[i_imu][0] if i_imu < len(imu_data) else float("inf")

        #如果当前相机时间早于或等于当前的IMU时间 
        if t_cam <= t_imu and i_cam < len(cam_data):
            #发布相机
            cam_timestamp, image_filename = cam_data[i_cam]
            #拼接得到图片完整路径
            img_path = os.path.join(image_dir, image_filename)
            #使用OpenCV读取图片
            img = cv2.imread(img_path, cv2.IMREAD_COLOR)
            #图像读取到的话进行发布
            if img is not None:
                #将OpenCV图像转换为sensor_msgs/Image
                img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
                #使用相机CSV中的时间戳作为消息的header.stamp
                img_msg.header.stamp = rospy.Time.from_sec(cam_timestamp)
                #把文件名放到header.frame_id中
                img_msg.header.frame_id = image_filename
                #发布图像相关信息
                pub_img.publish(img_msg)
                #输出日志
                rospy.loginfo(f"Published image {image_filename}")
            #处理下一帧
            i_cam += 1
        #说明和相机相比当前IMU时间更早 
        elif i_imu < len(imu_data):
            imu_timestamp, acc, gyro = imu_data[i_imu]
            imu_msg = Imu()
            #使用imu的CSV中的时间戳作为消息的header.stamp
            imu_msg.header.stamp = rospy.Time.from_sec(imu_timestamp)
            #填充三轴加速度和三轴角速度数值
            imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = acc
            imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = gyro
            #发布
            pub_imu.publish(imu_msg)
            #处理下一条
            i_imu += 1

        #按设定主循环速率sleep防止cpu卡死
        rate.sleep()

    # 循环退出时打印统计结果，确认发了多少数据
    rospy.loginfo(f"All data published. Total images: {i_cam}, Total IMUs: {i_imu}")

if __name__ == "__main__":
    main()

