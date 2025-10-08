#!/usr/bin/env python3
#声明依赖
import rospy
import csv
import os
import cv2
import threading
import queue
from cv_bridge import CvBridge
from sensor_time_align.msg import FusedState

#定义类FusedSubscriber
class FusedSubscriber:
    def __init__(self):
        #opencv和ros转化
        self.bridge = CvBridge()

        #保存路径
        self.base_dir = rospy.get_param("~save_dir", "/home/slam/subscribe_test_09281724")
        self.image_dir = os.path.join(self.base_dir, "camera")
        imu_csv_path = os.path.join(self.base_dir, "imu_subscribe.csv")
        cam_csv_path = os.path.join(self.base_dir, "camera_subscribe.csv")

        os.makedirs(self.base_dir, exist_ok=True)
        if os.path.exists(self.image_dir):
            for f in os.listdir(self.image_dir):
                os.remove(os.path.join(self.image_dir, f))
        else:
            os.makedirs(self.image_dir, exist_ok=True)

        # 打开 CSV 文件
        self.imu_csv = open(imu_csv_path, "w", newline="")
        self.imu_writer = csv.writer(self.imu_csv)
        self.imu_writer.writerow(["timestamp_sec", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"])

        self.cam_csv = open(cam_csv_path, "w", newline="")
        self.cam_writer = csv.writer(self.cam_csv)
        self.cam_writer.writerow(["timestamp_sec", "image_filename"]) 

        self.num_images = 0
        self.num_imus = 0

        # IMU 缓冲区（批量写）
        self.imu_buffer = []
        self.imu_buffer_size = 1000  # 每1000条写一次磁盘

        # 异步保存队列（只用于图像）
        self.save_queue = queue.Queue()   # 不设 maxsize，防止丢数据
        self.saving_thread = threading.Thread(target=self.saver)
        self.saving_thread.daemon = True
        self.saving_thread.start()

        # 订阅 fused_topic，队列开大
        rospy.Subscriber("/fused_topic", FusedState, self.callback, queue_size=5000)
        rospy.loginfo(f"Saving subscribed data into {self.base_dir} (async mode)")

    def callback(self, msg: FusedState):
        """ROS回调：直接写IMU缓冲和相机CSV，图像放到队列异步保存"""
        timestamp = msg.stamp.to_sec()

        # 先写相机CSV
        self.cam_writer.writerow([timestamp, msg.image_filename])
        self.cam_csv.flush()

        # 缓存 IMU 数据
        for i in range(len(msg.imu_timestamp)):
            imu_t = msg.imu_timestamp[i]
            gx, gy, gz = msg.angular_velocity[i].x, msg.angular_velocity[i].y, msg.angular_velocity[i].z
            ax, ay, az = msg.linear_acceleration[i].x, msg.linear_acceleration[i].y, msg.linear_acceleration[i].z
            self.imu_buffer.append([imu_t, ax, ay, az, gx, gy, gz])
            self.num_imus += 1

        # 批量写入磁盘
        if len(self.imu_buffer) >= self.imu_buffer_size:
            self.imu_writer.writerows(self.imu_buffer)
            self.imu_csv.flush()
            self.imu_buffer.clear()

        # 图像交给异步线程保存
        if msg.image.data:
            self.save_queue.put((msg.image, msg.image_filename))

    def saver(self):
        """后台线程：异步保存图像"""
        while not rospy.is_shutdown():
            try:
                image_msg, filename = self.save_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                cv_img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
                save_path = os.path.join(self.image_dir, filename)
                cv2.imwrite(save_path, cv_img)
                self.num_images += 1
            except Exception as e:
                rospy.logwarn(f"[CAM] Failed to save image {filename}: {e}")

            self.save_queue.task_done()

    def shutdown(self):
        rospy.loginfo("Waiting for save_queue to be flushed...")

        # 等待图像保存完
        self.save_queue.join()

        # 写入剩余的 IMU 缓冲
        if self.imu_buffer:
            self.imu_writer.writerows(self.imu_buffer)
            self.imu_csv.flush()
            self.imu_buffer.clear()

        # 关闭文件
        self.imu_csv.close()
        self.cam_csv.close()
        rospy.loginfo("CSV files closed.")
        rospy.loginfo(f"Total images saved: {self.num_images}")
        rospy.loginfo(f"Total IMU samples saved: {self.num_imus}")

def main():
    rospy.init_node("fusion_sub")
    node = FusedSubscriber()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()

if __name__ == "__main__":
    main()
