#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <ros/master.h>

//定义类去封装图像订阅、处理、发布的所有逻辑
class ImageTimestampPublisher {
private:
    //ROS节点句柄
    ros::NodeHandle nh_;
    //订阅相机原始话题 /usb_cam/image_raw
    ros::Subscriber image_sub_;          
    //发布sensor_msgs/Image类型的目标话题image_timestamp_raw
    ros::Publisher image_ts_pub_;        
    cv_bridge::CvImagePtr cv_ptr_;
    //首帧发布标记
    bool first_frame_published_;       

public:
    //构造函数：初始化成员变量，创建发布者，等待相机话题，打印启动日志
    ImageTimestampPublisher() : first_frame_published_(false) {
        //初始化发布者：话题名image_timestamp_raw，消息类型sensor_msgs/Image，队列大小10
        image_ts_pub_ = nh_.advertise<sensor_msgs::Image>(
            "image_timestamp_raw",
            10
        );

        //等待所要订阅的相机话题/usb_cam/image_raw就绪
        waitForCameraTopic(10.0);

        //启动日志：删除文件路径相关内容，发布话题提示
        ROS_INFO("==================================================");
        ROS_INFO("Image Timestamp Publisher Started!");
        ROS_INFO_STREAM("Subscribed Topic: /usb_cam/image_raw");
        ROS_INFO_STREAM("Published Topic: image_timestamp_raw (sensor_msgs/Image)");
        ROS_INFO("TimeStamp Precision: 6 Decimal Places (Microseconds)");
        ROS_INFO("Press Ctrl+C to Stop");
        ROS_INFO("==================================================");
    }

    //析构函数：停止时进行提醒
    ~ImageTimestampPublisher() {
        ROS_INFO("Image Timestamp Publisher Stopped");
    }

    //等待相机话题函数：确保相机启动后再订阅
    void waitForCameraTopic(double timeout) {
        ROS_INFO_STREAM("Waiting for Camera Topic '/usb_cam/image_raw' (Timeout: " << timeout << "s)...");
        ros::Time start_time = ros::Time::now();

        while (ros::ok()) {
            //超时判断
            if ((ros::Time::now() - start_time).toSec() > timeout) {
                ROS_FATAL_STREAM("Timeout " << timeout << "s! Camera Topic Not Detected, Please Start Camera Node Manually");
                ros::shutdown();
                return;
            }

            //检查话题是否存在
            ros::master::V_TopicInfo topic_info_list;
            if (ros::master::getTopics(topic_info_list)) {
                for (const auto& topic_info : topic_info_list) {
                    if (topic_info.name == "/usb_cam/image_raw") {
                        ROS_INFO("Camera Topic Detected, Starting Image Subscription...");
                        //订阅相机话题，绑定回调函数
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

            //每0.5秒重试一次，降低CPU占用
            ros::Duration(0.5).sleep();  
        }
    }

    //图像回调函数
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            //图像格式转换，兼容yuyv/uyvy/mono8等编码转为BGR8
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

        //生成6位小数的微秒级时间戳
        double timestamp = msg->header.stamp.toSec();
        timestamp = round(timestamp * 1e6) / 1e6;  // 精确到微秒（如1760150000.123456）

        //通过ros::Time对象调用fromSec()，避免静态函数解析报错
        ros::Time ts;  //创建ros::Time对象
        ts.fromSec(timestamp);  //非静态方式赋值时间戳
        cv_ptr_->header.stamp = ts;  //将时间戳写入sensor_msgs/Image的header.stamp
        cv_ptr_->header.frame_id = "usb_cam";  //设置坐标系ID

        //发布sensor_msgs/Image类型消息到image_timestamp_raw话题
        image_ts_pub_.publish(cv_ptr_->toImageMsg());

        //首帧发布日志
        if (!first_frame_published_) {
            first_frame_published_ = true;
            ROS_INFO_STREAM("First Frame Published! Timestamp: " << timestamp);
        }
    }

};

int main(int argc, char** argv) {
    //初始化ros节点
    ros::init(argc, argv, "image_raw_node");

    try {
        //创建类对象，触发构造函数，启动初始化流程
        ImageTimestampPublisher publisher;
        ros::spin();
    } catch (const ros::Exception& e) {
        ROS_INFO("Image Timestamp Publisher Stopped Manually");
    } catch (const std::exception& e) {
        ROS_FATAL_STREAM("Program Terminated Unexpectedly: " << e.what());
    }

    return 0;
}

