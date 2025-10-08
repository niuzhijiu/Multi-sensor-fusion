#!/usr/bin/env python3
#声明依赖
import rospy
from sensor_msgs.msg import Image, Imu
from sensor_time_align.msg import FusedState 

#定义类叫做FusionNode
class FusionNode:
    def __init__(self):
        #发布话题名字叫fused_topic
        self.pub = rospy.Publisher("/fused_topic", FusedState, queue_size=10)

        #订阅相机和IMU
        rospy.Subscriber("/cam_image", Image, self.cam_callback, queue_size=100)
        rospy.Subscriber("/imu_data", Imu, self.imu_callback, queue_size=1000)

        #imu频率高进入缓存
        self.imu_buffer = []
        #记录上一次相机帧的时间戳
        self.last_cam_stamp = None
        #特殊处理第一帧的标志
        self.first_frame = True
        #记录发布了多少帧融合消息
        self.frame_count = 0

    #定义imu回调，每次收到Imu消息就把它追加到imu_buffer
    def imu_callback(self, msg):

        self.imu_buffer.append(msg)

        #缓存修剪。清理过期缓存防止缓存无限增长，清理是以最新IMU时间戳为基准的
        t_imu = msg.header.stamp.to_sec()
        self.imu_buffer = [
            imu for imu in self.imu_buffer
            if (t_imu - imu.header.stamp.to_sec()) < 2.0
        ]

    #定义相机回调
    def cam_callback(self, cam_msg):
        #提取当前相机帧的时间戳
        t_curr = cam_msg.header.stamp.to_sec()

        #第一帧仍然保存，但不带IMU
        if self.first_frame:
            
            #填充数据
            fused_msg = FusedState()
            fused_msg.stamp = cam_msg.header.stamp
            fused_msg.image_filename = cam_msg.header.frame_id
            fused_msg.image = cam_msg

            fused_msg.angular_velocity = []
            fused_msg.linear_acceleration = []
            fused_msg.imu_timestamp = []

            #发布这条融合消息
            self.pub.publish(fused_msg)
            #打印第一帧发布成功消息
            rospy.loginfo("Published first fused data: %s (no IMU)" %
                          fused_msg.image_filename)

            #状态更新
            self.first_frame = False
            self.last_cam_stamp = cam_msg.header.stamp
            self.frame_count += 1
            return

        #上一帧相机时间戳
        t_prev = self.last_cam_stamp.to_sec()

        #使用前闭后开区间 [t_prev, t_curr)保证imu不重合，从缓存中筛选属于区间的IMU消息，前闭后开的选择避免同一条IMU被同时归到两帧
        imu_selected = [
            imu for imu in self.imu_buffer
            if t_prev <= imu.header.stamp.to_sec() < t_curr
        ]

        #填充相机部分
        fused_msg = FusedState()
        fused_msg.stamp = cam_msg.header.stamp
        fused_msg.image_filename = cam_msg.header.frame_id
        fused_msg.image = cam_msg

        #填充imu部分
        for imu in imu_selected:
            fused_msg.angular_velocity.append(imu.angular_velocity)
            fused_msg.linear_acceleration.append(imu.linear_acceleration)
            fused_msg.imu_timestamp.append(imu.header.stamp.to_sec())

        #发布
        self.pub.publish(fused_msg)
        rospy.loginfo("Published fused data: %s with %d IMU samples" %
                      (fused_msg.image_filename, len(imu_selected)))

        # 更新上一帧时间戳
        self.last_cam_stamp = cam_msg.header.stamp
        self.frame_count += 1

    def shutdown(self):
        rospy.loginfo(f"Fusion node shutting down. Total frames published: {self.frame_count}")

if __name__ == "__main__":
    rospy.init_node("fusion_node", anonymous=True)
    node = FusionNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
