//声明头文件和依赖
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_time_align/FusedState.h>

#include <vector>
#include <string>
#include <csignal>   // for signal handling

//定义一个FusionNode类，把节点的状态和回调封装起来
class FusionNode {
public:
    //构造函数FusionNode
    FusionNode(ros::NodeHandle& nh) {
        //创建一个发布器，发布FusedState消息到/fused_topic，队列是10
        pub_ = nh.advertise<sensor_time_align::FusedState>("/fused_topic", 10);

        //订阅相机话题/cam_image，队列100
        sub_cam_ = nh.subscribe("/cam_image", 100,
                                &FusionNode::camCallback, this);
        //订阅imu话题/imu_data，根据真实频率设置队列1000
        sub_imu_ = nh.subscribe("/imu_data", 1000,
                                &FusionNode::imuCallback, this);

        //标记第一帧
        first_frame_ = true;
        //计数器的初始化
        frame_count_ = 0;
        imu_total_count_ = 0;
    }

    //imu回调，把接收到的IMU数据存到缓存里，等到相机图像到来时再配对
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        imu_buffer_.push_back(*msg);
    }

    //cam回调，记录当前相机帧的时间，用它来找到与之区间对应的IMU数据。
    void camCallback(const sensor_msgs::Image::ConstPtr& cam_msg) {
        double t_curr = cam_msg->header.stamp.toSec();

        //第一帧特殊处理，融合逻辑是把第一帧之前和最后一帧之后的imu舍弃，以cam作标杆
        if (first_frame_) {
            //创建要发布的融合消息
            sensor_time_align::FusedState fused_msg;
            //以相机时间戳作为融合消息的时间戳
            fused_msg.timestamp = cam_msg->header.stamp;
            //读取文件名
            fused_msg.image_filename = cam_msg->header.frame_id; 
            //输入图像
            fused_msg.image = *cam_msg;

            //发布融合消息
            pub_.publish(fused_msg);
            //提示日志
            ROS_INFO("Published first fused data: %s (no IMU)",
                     fused_msg.image_filename.c_str());

            //标记已处理第一帧
            first_frame_ = false;
            //记录上一个相机时间戳
            last_cam_stamp_ = cam_msg->header.stamp;
            //累加
            frame_count_++;
            //返回
            return;
        }

        //取上一次相机时间
        double t_prev = last_cam_stamp_.toSec();

        //左闭右开区间[t_prev, t_curr)把满足条件的IMU加入要发布的队列
        std::vector<sensor_msgs::Imu> imu_selected;
        for (auto& imu : imu_buffer_) {
            double t_imu = imu.header.stamp.toSec();
            if (t_prev <= t_imu && t_imu < t_curr) {
                imu_selected.push_back(imu);
            }
        }

        //填充融合消息
        sensor_time_align::FusedState fused_msg;
        //时间戳
        fused_msg.timestamp = cam_msg->header.stamp;
        //图片名字
        fused_msg.image_filename = cam_msg->header.frame_id;
        //图片数据
        fused_msg.image = *cam_msg;

        //输入选中的各个imu的角速度、加速度、时间戳
        for (auto& imu : imu_selected) {
            fused_msg.angular_velocity.push_back(imu.angular_velocity);
            fused_msg.linear_acceleration.push_back(imu.linear_acceleration);
            fused_msg.imu_timestamp.push_back(imu.header.stamp.toSec());
        }

        //发布融合消息到 /fused_topic
        pub_.publish(fused_msg);
        //记录每个消息里的imu数，用于分析数据传输情况
        ROS_INFO("Published fused data: %s with %lu IMU samples",
                 fused_msg.image_filename.c_str(), imu_selected.size());

        //更新统计last_cam_stamp_ 为当前帧，用于下一次回调
        last_cam_stamp_ = cam_msg->header.stamp;
        //往前推进一帧
        frame_count_++;
        //累加全部imu数，用于分析数据传输情况
        imu_total_count_ += imu_selected.size();
    }

    //运行结束之后计数，用于分析数据传输情况
    void shutdown() {
        ROS_INFO("Fusion node finished. Total camera frames: %d, total IMU samples fused: %d",
                 frame_count_, imu_total_count_);
    }

private:
    //发布器
    ros::Publisher pub_;
    //订阅器
    ros::Subscriber sub_cam_, sub_imu_;

    //imu数据缓存，只有几万条因此此处未设置清除操作
    std::vector<sensor_msgs::Imu> imu_buffer_;
    //上一帧时间，融合逻辑是把从上一帧时间开始（包括上一帧时间）到这一帧时间的imu发布到这一帧的消息里
    ros::Time last_cam_stamp_;

    //第一帧标志
    bool first_frame_;
    //计数器
    int frame_count_;
    int imu_total_count_;
};

//全局指针，下面主函数中用于Ctrl+C捕获
FusionNode* g_node = nullptr;

//Ctrl+C之后信号处理函数
void sigintHandler(int sig) {
    if (g_node) {
        g_node->shutdown();
    }
    ros::shutdown();
}

int main(int argc, char** argv) {
    //初始化ROS节点，节点名fusion_node
    ros::init(argc, argv, "fusion_node", ros::init_options::NoSigintHandler);
    //创建NodeHandle用于订阅和发布
    ros::NodeHandle nh;

    //实例化FusionNode
    FusionNode node(nh);
    //把全局指针指向该实例，以便信号处理函数使用
    g_node = &node;

    //捕获 Ctrl+C
    signal(SIGINT, sigintHandler);

    //进入回调循环
    ros::spin();
    return 0;
}
