// 声明头文件和依赖
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <ros/master.h>

namespace fs = boost::filesystem;

// 定义类去封装图像订阅、处理、发布的所有逻辑
class ImageDataRecorderAndPublisher {
private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;          // 订阅相机原始话题 /usb_cam/image_raw
    ros::Publisher image_ts_pub_;        // 发布目标话题 /image_timestamp_raw
    cv_bridge::CvImagePtr cv_ptr_;       // 图像格式转换中间变量
    
    // 保存相关成员变量
    std::string base_dir_;
    std::string images_dir_;
    std::string timestamp_path_;
    bool first_frame_saved_;
    std::ofstream csv_file_;              
    
    // 发布相关成员变量
    bool first_frame_published_;          // 首帧发布标记

public:
    // 构造函数：初始化保存路径和CSV和话题发布者，等待相机话题
    ImageDataRecorderAndPublisher() : first_frame_saved_(false), first_frame_published_(false) {
        // 初始化保存路径
        base_dir_ = "/home/cat/1230";
        images_dir_ = base_dir_ + "/camera";
        timestamp_path_ = base_dir_ + "/camera_timestamp.csv";

        // 清理旧数据，创建目录，初始化CSV
        clearOldData();
        createDirectories();
        initCsv();

        // 初始化话题发布者，发布 sensor_msgs/Image 到 /image_timestamp_raw
        image_ts_pub_ = nh_.advertise<sensor_msgs::Image>(
            "/image_timestamp_raw",
            10 
        );

        // 等待相机话题，确保相机就绪后再订阅
        waitForCameraTopic(10.0);

        // 打印启动日志：同时提示保存路径和发布话题
        ROS_INFO("==================================================");
        ROS_INFO("Image Recorder & Publisher Started (Overwrite Mode)!");
        ROS_INFO_STREAM("Image Save Path: " << images_dir_);
        ROS_INFO_STREAM("Timestamp CSV Path: " << timestamp_path_);
        ROS_INFO_STREAM("Published Topic: /image_timestamp_raw (sensor_msgs/Image)");
        ROS_INFO("TimeStamp Precision: 6 Decimal Places (Microseconds)");
        ROS_INFO("Press Ctrl+C to Stop");
        ROS_INFO("==================================================");
    }

    // 析构函数：关闭CSV文件
    ~ImageDataRecorderAndPublisher() {
        if (csv_file_.is_open()) {
            csv_file_.close();
            ROS_INFO("CSV File Closed");
        }
        ROS_INFO("Image Recorder & Publisher Stopped");
    }

    // 旧数据清理函数clearOldData：清理掉所保存的旧数据
    void clearOldData() {
        // 检查图像保存目录是否存在，存在就删掉旧的
        if (fs::exists(images_dir_)) {
            fs::remove_all(images_dir_);
            ROS_INFO_STREAM("Old Image Folder Deleted (Overwrite Mode): " << images_dir_);
        }
        // 检查csv保存目录是否存在，存在就删掉旧的
        if (fs::exists(timestamp_path_)) {
            fs::remove(timestamp_path_);
            ROS_INFO_STREAM("Old CSV File Deleted (Overwrite Mode): " << timestamp_path_);
        }
    }

    // 目录创建函数createDirectories：创建图像保存目录
    void createDirectories() {
        try {
            fs::create_directories(images_dir_);
            ROS_INFO_STREAM("New Image Directory Created: " << images_dir_);
        } catch (const fs::filesystem_error& e) {
            ROS_FATAL_STREAM("Failed to Create Directory: " << e.what() << " (Check Path Permissions)");
            ros::shutdown();
        }
    }

    // csv初始化函数initCsv：初始化时间戳CSV文件，写入表头
    void initCsv() {
        try {
            csv_file_.open(timestamp_path_, std::ios::out | std::ios::trunc);
            if (csv_file_.is_open()) {
                csv_file_ << "timestamp_sec,image_filename" << std::endl;
                ROS_INFO("CSV File Initialized (Overwrite Mode, Old Data Cleared)");
            } else {
                ROS_FATAL_STREAM("CSV Initialization Failed: Cannot Open File (Check Path Permissions)");
                ros::shutdown();
            }
        } catch (const std::exception& e) {
            ROS_FATAL_STREAM("CSV Initialization Failed: " << e.what() << " (Check Path Permissions)");
            ros::shutdown();
        }
    }

    // 等待相机话题函数waitForCameraTopic：确保相机启动之后再进行订阅，防止相机节点未启动时就订阅话题导致消息丢失
    void waitForCameraTopic(double timeout) {
        ROS_INFO_STREAM("Waiting for Camera Topic '/usb_cam/image_raw' (Timeout: " << timeout << "s)...");
        ros::Time start_time = ros::Time::now();

        // 超时判断环节，超时未检测到话题就关闭节点
        while (ros::ok()) {
            if ((ros::Time::now() - start_time).toSec() > timeout) {
                ROS_FATAL_STREAM("Timeout " << timeout << "s! Camera Topic Not Detected, Please Start Camera Node Manually");
                ros::shutdown();
                return;
            }

            // 获取ROS系统中所有话题列表
            ros::master::V_TopicInfo topic_info_list;
            if (ros::master::getTopics(topic_info_list)) {
                // 遍历话题，检查是否存在目标相机话题
                for (const auto& topic_info : topic_info_list) {
                    // 找到目标话题之后订阅目标话题
                    if (topic_info.name == "/usb_cam/image_raw") {
                        ROS_INFO("Camera Topic Detected, Starting Subscription...");
                        // 订阅相机话题，绑定回调函数
                        image_sub_ = nh_.subscribe(
                            "/usb_cam/image_raw",
                            10,
                            &ImageDataRecorderAndPublisher::imageCallback,
                            this
                        );
                        return;
                    }
                }
            } else {
                ROS_WARN("Failed to Get Topic List (Will Retry)");
            }

            // 每0.5秒重试一次，降低CPU占用
            ros::Duration(0.5).sleep();
        }
    }

    // CSV写入函数，将图像的时间戳和对应文件名写入CSV
    void writeToCsv(double timestamp, const std::string& filename) {
        try {
            if (csv_file_.is_open()) {
                // 保留6位小数，微秒级精度
                csv_file_.precision(6);
                csv_file_ << std::fixed << timestamp << "," << filename << std::endl;
            } else {
                ROS_ERROR_STREAM("CSV Write Failed: File Not Open (Check Initialization)");
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("CSV Write Failed: " << e.what() << " (Check File Permissions)");
        }
    }

    // 回调函数imageCallback：同时处理保存和发布
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // 图像格式转换
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

        // 生成微秒级时间戳
        double timestamp = msg->header.stamp.toSec();
        timestamp = round(timestamp * 1e6) / 1e6;  // 保留6位小数即微秒级
        std::stringstream ss;
        ss.precision(6);
        ss << std::fixed << timestamp;
        std::string filename = ss.str() + ".png";
        std::string save_path = images_dir_ + "/" + filename;

        // 分支逻辑1：保存图像并且写入CSV，实现保存数据逻辑
        try {
            std::vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(0);  
            
            bool save_ok = cv::imwrite(save_path, cv_ptr_->image, compression_params);
            // 保存成功情况
            if (save_ok) {
                // 首帧提示
                if (!first_frame_saved_) {
                    first_frame_saved_ = true;
                    ROS_INFO_STREAM("First Frame Saved: " << filename);
                }
                // 保存成功才写入CSV
                writeToCsv(timestamp, filename);  
            } else {
                ROS_ERROR_STREAM("Image Save Failed (No CSV Write): " << filename);
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Image Save Error (No CSV Write): " << e.what());
        }

        // 分支逻辑2：发布话题逻辑
        try {
            // 为发布的消息设置时间戳
            ros::Time ts;
            ts.fromSec(timestamp);
            cv_ptr_->header.stamp = ts;
            cv_ptr_->header.frame_id = "usb_cam";

            // 发布话题
            image_ts_pub_.publish(cv_ptr_->toImageMsg());

            // 首帧发布日志
            if (!first_frame_published_) {
                first_frame_published_ = true;
                ROS_INFO_STREAM("First Frame Published to 'image_timestamp_raw' (Timestamp: " << timestamp << ")");
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Image Publish Error (Frame Saved Normally): " << e.what());
        }
    }
};

// 创建类对象
int main(int argc, char** argv) {
    ros::init(argc, argv, "image_raw_node"); 

    try {
        ImageDataRecorderAndPublisher recorder_publisher;
        ros::spin();  
    } catch (const ros::Exception& e) {
        ROS_INFO("Image Recorder & Publisher Stopped Manually");
    } catch (const std::exception& e) {
        ROS_FATAL_STREAM("Program Terminated Unexpectedly: " << e.what());
    }

    return 0;
}
