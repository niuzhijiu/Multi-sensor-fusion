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

class ImageDataRecorderAndPublisher {
private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;          // 订阅相机原始话题 /usb_cam/image_raw
    ros::Publisher image_ts_pub_;        // 发布目标话题 image_timestamp_raw（sensor_msgs/Image类型）
    cv_bridge::CvImagePtr cv_ptr_;       // 图像格式转换中间变量（复用，避免重复转换）
    
    // 保存相关成员变量
    std::string base_dir_;
    std::string images_dir_;
    std::string timestamp_path_;
    bool first_frame_saved_;
    std::ofstream csv_file_;              // 持续打开的CSV文件（覆盖模式）
    
    // 发布相关成员变量
    bool first_frame_published_;          // 首帧发布标记（日志用）

public:
    // 构造函数：初始化保存路径+CSV+话题发布者+等待相机话题
    ImageDataRecorderAndPublisher() : first_frame_saved_(false), first_frame_published_(false) {
        // 1. 初始化保存路径（与原“只保存”版本一致）
        base_dir_ = "/home/cat/1031_kalibr_test";
        images_dir_ = base_dir_ + "/camera";
        timestamp_path_ = base_dir_ + "/camera_timestamp.csv";

        // 2. 清理旧数据+创建目录+初始化CSV（原保存逻辑不变）
        clearOldData();
        createDirectories();
        initCsv();

        // 3. 初始化话题发布者（新增：发布sensor_msgs/Image到image_timestamp_raw）
        image_ts_pub_ = nh_.advertise<sensor_msgs::Image>(
            "image_timestamp_raw",
            10  // 队列大小10，适配25Hz帧率
        );

        // 4. 等待相机话题（原逻辑不变，确保相机就绪后再订阅）
        waitForCameraTopic(10.0);

        // 5. 启动日志：同时提示保存路径和发布话题
        ROS_INFO("==================================================");
        ROS_INFO("Image Recorder & Publisher Started (Overwrite Mode)!");
        ROS_INFO_STREAM("Image Save Path: " << images_dir_);
        ROS_INFO_STREAM("Timestamp CSV Path: " << timestamp_path_);
        ROS_INFO_STREAM("Published Topic: image_timestamp_raw (sensor_msgs/Image)");
        ROS_INFO("TimeStamp Precision: 6 Decimal Places (Microseconds)");
        ROS_INFO("Press Ctrl+C to Stop");
        ROS_INFO("==================================================");
    }

    // 析构函数：关闭CSV文件（原保存逻辑不变）
    ~ImageDataRecorderAndPublisher() {
        if (csv_file_.is_open()) {
            csv_file_.close();
            ROS_INFO("CSV File Closed");
        }
        ROS_INFO("Image Recorder & Publisher Stopped");
    }

    // ---------------------- 原“只保存”版本核心函数（不变） ----------------------
    void clearOldData() {
        if (fs::exists(images_dir_)) {
            fs::remove_all(images_dir_);
            ROS_INFO_STREAM("Old Image Folder Deleted (Overwrite Mode): " << images_dir_);
        }
        if (fs::exists(timestamp_path_)) {
            fs::remove(timestamp_path_);
            ROS_INFO_STREAM("Old CSV File Deleted (Overwrite Mode): " << timestamp_path_);
        }
    }

    void createDirectories() {
        try {
            fs::create_directories(images_dir_);
            ROS_INFO_STREAM("New Image Directory Created: " << images_dir_);
        } catch (const fs::filesystem_error& e) {
            ROS_FATAL_STREAM("Failed to Create Directory: " << e.what() << " (Check Path Permissions)");
            ros::shutdown();
        }
    }

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

    void waitForCameraTopic(double timeout) {
        ROS_INFO_STREAM("Waiting for Camera Topic '/usb_cam/image_raw' (Timeout: " << timeout << "s)...");
        ros::Time start_time = ros::Time::now();

        while (ros::ok()) {
            if ((ros::Time::now() - start_time).toSec() > timeout) {
                ROS_FATAL_STREAM("Timeout " << timeout << "s! Camera Topic Not Detected, Please Start Camera Node Manually");
                ros::shutdown();
                return;
            }

            ros::master::V_TopicInfo topic_info_list;
            if (ros::master::getTopics(topic_info_list)) {
                for (const auto& topic_info : topic_info_list) {
                    if (topic_info.name == "/usb_cam/image_raw") {
                        ROS_INFO("Camera Topic Detected, Starting Subscription...");
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

            ros::Duration(0.5).sleep();
        }
    }

    void writeToCsv(double timestamp, const std::string& filename) {
        try {
            if (csv_file_.is_open()) {
                csv_file_.precision(6);
                csv_file_ << std::fixed << timestamp << "," << filename << std::endl;
            } else {
                ROS_ERROR_STREAM("CSV Write Failed: File Not Open (Check Initialization)");
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("CSV Write Failed: " << e.what() << " (Check File Permissions)");
        }
    }

    // ---------------------- 核心回调：同时处理“保存”和“发布” ----------------------
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // 1. 图像格式转换（复用cv_ptr_，同时为“保存”和“发布”提供数据）
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

        // 2. 生成微秒级时间戳（复用时间戳，确保“保存”和“发布”完全同步）
        double timestamp = msg->header.stamp.toSec();
        timestamp = round(timestamp * 1e6) / 1e6;  // 保留6位小数（微秒）
        std::stringstream ss;
        ss.precision(6);
        ss << std::fixed << timestamp;
        std::string filename = ss.str() + ".png";
        std::string save_path = images_dir_ + "/" + filename;

        // ---------------------- 分支1：保存图像+写入CSV（原逻辑不变） ----------------------
        try {
            std::vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(0);  // 无压缩保存
            
            bool save_ok = cv::imwrite(save_path, cv_ptr_->image, compression_params);
            if (save_ok) {
                if (!first_frame_saved_) {
                    first_frame_saved_ = true;
                    ROS_INFO_STREAM("First Frame Saved: " << filename);
                }
                writeToCsv(timestamp, filename);  // 保存成功才写CSV
            } else {
                ROS_ERROR_STREAM("Image Save Failed (No CSV Write): " << filename);
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Image Save Error (No CSV Write): " << e.what());
        }

        // ---------------------- 分支2：发布带时间戳的sensor_msgs/Image（新增逻辑） ----------------------
        try {
            // 为发布的消息设置时间戳（与保存的时间戳完全一致）
            ros::Time ts;
            ts.fromSec(timestamp);  // 兼容所有编译环境的时间戳赋值方式
            cv_ptr_->header.stamp = ts;
            cv_ptr_->header.frame_id = "usb_cam";  // 可选：坐标系ID，适配SLAM/标定

            // 发布话题（复用cv_ptr_转换结果，无需重复创建图像消息）
            image_ts_pub_.publish(cv_ptr_->toImageMsg());

            // 首帧发布日志（仅提示一次）
            if (!first_frame_published_) {
                first_frame_published_ = true;
                ROS_INFO_STREAM("First Frame Published to 'image_timestamp_raw' (Timestamp: " << timestamp << ")");
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Image Publish Error (Frame Saved Normally): " << e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_raw_node_test");  // 节点名不变，兼容原启动习惯

    try {
        ImageDataRecorderAndPublisher recorder_publisher;
        ros::spin();  // 阻塞等待回调（同时处理保存和发布）
    } catch (const ros::Exception& e) {
        ROS_INFO("Image Recorder & Publisher Stopped Manually");
    } catch (const std::exception& e) {
        ROS_FATAL_STREAM("Program Terminated Unexpectedly: " << e.what());
    }

    return 0;
}
