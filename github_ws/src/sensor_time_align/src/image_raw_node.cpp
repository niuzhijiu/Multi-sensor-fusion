#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <ros/master.h>

// 仅保留必要依赖，删除文件操作相关头文件（boost/filesystem、fstream等）
class ImageTimestampPublisher {
private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;          // 订阅相机原始话题 /usb_cam/image_raw
    ros::Publisher image_ts_pub_;        // 发布目标话题 image_timestamp_raw（sensor_msgs/Image类型）
    cv_bridge::CvImagePtr cv_ptr_;
    bool first_frame_published_;         // 首帧发布标记（仅日志提示用）

public:
    // 构造函数：初始化发布者+等待相机话题，删除文件相关初始化
    ImageTimestampPublisher() : first_frame_published_(false) {
        // 1. 初始化发布者：话题名 image_timestamp_raw，消息类型 sensor_msgs/Image，队列大小10（适配25Hz帧率）
        image_ts_pub_ = nh_.advertise<sensor_msgs::Image>(
            "image_timestamp_raw",
            10
        );

        // 2. 保留原逻辑：等待相机话题 /usb_cam/image_raw 就绪
        waitForCameraTopic(10.0);

        // 3. 启动日志：删除文件路径相关内容，新增发布话题提示
        ROS_INFO("==================================================");
        ROS_INFO("Image Timestamp Publisher Started!");
        ROS_INFO_STREAM("Subscribed Topic: /usb_cam/image_raw");
        ROS_INFO_STREAM("Published Topic: image_timestamp_raw (sensor_msgs/Image)");
        ROS_INFO("TimeStamp Precision: 6 Decimal Places (Microseconds)");
        ROS_INFO("Press Ctrl+C to Stop");
        ROS_INFO("==================================================");
    }

    // 析构函数：删除CSV文件关闭逻辑（无文件操作）
    ~ImageTimestampPublisher() {
        ROS_INFO("Image Timestamp Publisher Stopped");
    }

    // 保留原逻辑：等待相机话题，确保相机启动后再订阅
    void waitForCameraTopic(double timeout) {
        ROS_INFO_STREAM("Waiting for Camera Topic '/usb_cam/image_raw' (Timeout: " << timeout << "s)...");
        ros::Time start_time = ros::Time::now();

        while (ros::ok()) {
            // 超时判断
            if ((ros::Time::now() - start_time).toSec() > timeout) {
                ROS_FATAL_STREAM("Timeout " << timeout << "s! Camera Topic Not Detected, Please Start Camera Node Manually");
                ros::shutdown();
                return;
            }

            // 检查话题是否存在
            ros::master::V_TopicInfo topic_info_list;
            if (ros::master::getTopics(topic_info_list)) {
                for (const auto& topic_info : topic_info_list) {
                    if (topic_info.name == "/usb_cam/image_raw") {
                        ROS_INFO("Camera Topic Detected, Starting Image Subscription...");
                        // 订阅相机话题，绑定回调函数（处理后发布到目标话题）
                        image_sub_ = nh_.subscribe(
                            "/usb_cam/image_raw",
                            10,
                            &ImageTimestampPublisher::imageCallback,
                            this
                        );
                        return;
                    }
                }
            } else {
                ROS_WARN("Failed to Get Topic List (Will Retry)");
            }

            ros::Duration(0.5).sleep();  // 每0.5秒重试一次，降低CPU占用
        }
    }

    // 核心修改：图像回调函数（修复时间戳赋值方式，确保编译通过）
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // 1. 保留原逻辑：图像格式转换（兼容yuyv/uyvy/mono8等编码，转为BGR8）
            if (msg->encoding == "yuyv" || msg->encoding == "uyvy") {
                cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } else if (msg->encoding == "mono8") {
                cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
                cv::cvtColor(cv_ptr_->image, cv_ptr_->image, cv::COLOR_GRAY2BGR);
            } else {
                cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR_STREAM("Image Conversion Failed (Skipping Frame): " << e.what());
            return;
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Image Processing Error (Skipping Frame): " << e.what());
            return;
        }

        // 2. 保留原逻辑：生成6位小数的微秒级时间戳
        double timestamp = msg->header.stamp.toSec();
        timestamp = round(timestamp * 1e6) / 1e6;  // 精确到微秒（如1760150000.123456）

        // 3. 修复：通过ros::Time对象调用fromSec()，避免静态函数解析报错
        ros::Time ts;  // 创建ros::Time对象
        ts.fromSec(timestamp);  // 非静态方式赋值时间戳（兼容所有编译环境）
        cv_ptr_->header.stamp = ts;  // 将时间戳写入sensor_msgs/Image的header.stamp
        cv_ptr_->header.frame_id = "usb_cam";  // 可选：设置坐标系ID，适配后续SLAM/标定

        // 4. 发布sensor_msgs/Image类型消息到image_timestamp_raw话题
        image_ts_pub_.publish(cv_ptr_->toImageMsg());

        // 5. 首帧发布日志（仅提示一次，避免刷屏）
        if (!first_frame_published_) {
            first_frame_published_ = true;
            ROS_INFO_STREAM("First Frame Published! Timestamp: " << timestamp);
        }
    }

    // 删除原文件相关函数（clearOldData/createDirectories/initCsv/writeToCsv）
    // （无文件操作，这些函数无需保留）
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_raw_node");  // 节点名不变，兼容原启动习惯

    try {
        ImageTimestampPublisher publisher;
        ros::spin();  // 阻塞等待回调函数执行（持续订阅+发布）
    } catch (const ros::Exception& e) {
        ROS_INFO("Image Timestamp Publisher Stopped Manually");
    } catch (const std::exception& e) {
        ROS_FATAL_STREAM("Program Terminated Unexpectedly: " << e.what());
    }

    return 0;
}

