// 声明头文件和依赖
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

// 声明ROS发布者对象
ros::Publisher imu_raw_pub;

// 回调函数：订阅到/imu/data数据后执行，纯转发逻辑
void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    // 核心逻辑：收到什么数据，直接原封不动发布，零修改
    imu_raw_pub.publish(imu_msg);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "imu_topic_convert_node");
    ros::NodeHandle nh;

    // 创建发布者，发布/imu_raw 话题，消息类型sensor_msgs/Imu，队列长度10
    imu_raw_pub = nh.advertise<sensor_msgs::Imu>("/imu_raw", 10);

    // 创建订阅者：订阅/imu/data 话题，回调函数处理数据，队列长度10
    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 10, imuCallback);

    ROS_INFO("node open successful");
    ROS_INFO("subscribe topic: /imu/data");
    ROS_INFO("publish topic: /imu_raw");

    // 循环等待回调函数触发
    ros::spin();

    return 0;
}
