#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h> 
#include <geometry_msgs/PointStamped.h>
#include <cv_bridge/cv_bridge.h>          
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <algorithm>
#include <set>

//命名空间简化
namespace fs = boost::filesystem;
using namespace std;

class EurocBagSplitter {
private:
    string bag_path_;       //输入bag文件路径
    string output_root_;    //输出根目录
    ros::NodeHandle nh_;    

public:
    EurocBagSplitter(const string& bag_path, const string& output_root)
        : bag_path_(bag_path), output_root_(output_root), nh_("~") {
        //初始化输出目录
        if (!fs::exists(output_root_)) {
            if (!fs::create_directories(output_root_)) {
                throw runtime_error("Failed to create the output directory:" + output_root_);
            }
        }
        ROS_INFO_STREAM("[Noetic Bag Splitter] output directory is ready:" << output_root_);
    }

    
    //分解函数 
    void split() {
        //检查bag文件有效性
        if (!fs::exists(bag_path_) || !fs::is_regular_file(bag_path_)) {
            throw runtime_error("Bag cannot be used:" + bag_path_);
        }

        try {
            //打开bag文件
            rosbag::Bag bag;
            bag.open(bag_path_, rosbag::bagmode::Read);
            ROS_INFO_STREAM("[Noetic Bag Splitter] open bag successful:" << bag_path_);

            //获取bag中所有话题
            rosbag::View full_view(bag);  //遍历整个bag的所有消息
            set<string> topic_set;        //自动去重复
            for (const rosbag::MessageInstance& msg_inst : full_view) {
                topic_set.insert(msg_inst.getTopic());  //稳定返回话题名
            }
            vector<string> topics(topic_set.begin(), topic_set.end());

            //打印检测到的话题
            ROS_INFO("[Noetic Bag Splitter]  %zu topics detected:", topics.size());
            for (size_t i = 0; i < topics.size(); i++) {
                ROS_INFO("  %zu. %s", i + 1, topics[i].c_str());
            }

            //提取相机图像
            extractCamera(bag, "/cam0/image_raw", "cam0");
            extractCamera(bag, "/cam1/image_raw", "cam1");

            //提取IMU数据
            string imu_topic = (find(topics.begin(), topics.end(), "/imu0") != topics.end()) 
                               ? "/imu0" : "/imu0/data";
            extractImu(bag, imu_topic, "imu0");

            //提取Leica位置数据
            if (find(topics.begin(), topics.end(), "/leica/position") != topics.end()) {
                extractLeica(bag, "/leica/position", "leica");
            }

            //关闭bag文件
            bag.close();
            ROS_INFO_STREAM("\n[Noetic Bag Splitter] split finish! data has been saved:" << output_root_);

        } catch (const rosbag::BagException& e) {
            //rosbag的异常捕获环节
            throw runtime_error("Bag operate abnormal：" + string(e.what()));
        } catch (const exception& e) {
            throw runtime_error("split abnormal：" + string(e.what()));
        }
    }

private:
    //提取相机图像
    void extractCamera(rosbag::Bag& bag, const string& topic, const string& save_dir) {
        //检查话题是否存在
        rosbag::View topic_view(bag, rosbag::TopicQuery(topic));
        if (topic_view.size() == 0) {
            ROS_WARN_STREAM("[Camera Extractor] topic " << topic << " No data,skip extraction");
            return;
        }

        //创建保存目录
        fs::path save_path = fs::path(output_root_) / save_dir;
        if (!fs::exists(save_path)) {
            fs::create_directory(save_path);
        }

        //时间戳CSV文件
        string csv_path = (fs::path(output_root_) / (save_dir + "_timestamp.csv")).string();
        ofstream csv_file(csv_path);
        if (!csv_file.is_open()) {
            ROS_ERROR_STREAM("[Camera Extractor] cannot create CSV:" << csv_path);
            return;
        }
        csv_file << "timestamp_sec,image_filename" << endl;  // CSV表头

        //提取并保存图像
        int img_count = 0;
        for (const rosbag::MessageInstance& msg_inst : topic_view) {
            sensor_msgs::ImageConstPtr img_msg = msg_inst.instantiate<sensor_msgs::Image>();
            if (!img_msg) continue;

            try {
                cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(
                    img_msg, sensor_msgs::image_encodings::MONO8
                );

                //生成时间戳文件名
                double ts = img_msg->header.stamp.toSec();
                stringstream ss;
                ss << fixed << setprecision(6) << ts << ".png";
                string img_name = ss.str();
                string img_path = (save_path / img_name).string();

                //保存图像
                if (!cv::imwrite(img_path, cv_ptr->image)) {
                    ROS_WARN_STREAM("[Camera Extractor] save image fail:" << img_path);
                    continue;
                }

                //记录时间戳
                csv_file << fixed << setprecision(6) << ts << "," << img_name << endl;
                img_count++;

            } catch (const cv_bridge::Exception& e) {
                //特定cv_bridge异常
                ROS_WARN_STREAM("[Camera Extractor] Image conversion failed:" << e.what());
                continue;
            }
        }

        //关闭文件并打印统计
        csv_file.close();
        ROS_INFO_STREAM("[Camera Extractor] " << topic << " extract completed:" 
                        << img_count << " Frame to Directory:" << save_path 
                        << ",timestamp file:" << csv_path);
    }

    //提取IMU数据
    void extractImu(rosbag::Bag& bag, const string& topic, const string& save_prefix) {
        rosbag::View topic_view(bag, rosbag::TopicQuery(topic));
        if (topic_view.size() == 0) {
            ROS_WARN_STREAM("[IMU Extractor] topic " << topic << " no data,skip extract");
            return;
        }

        //创建IMU数据CSV文件
        string csv_path = (fs::path(output_root_) / (save_prefix + "_data.csv")).string();
        ofstream csv_file(csv_path);
        if (!csv_file.is_open()) {
            ROS_ERROR_STREAM("[IMU Extractor] cannot create CSV:" << csv_path);
            return;
        }
        //IMU数据字段顺序设置
        csv_file << "timestamp_sec,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z" << endl;

        //提取IMU数据
        int imu_count = 0;
        for (const rosbag::MessageInstance& msg_inst : topic_view) {
            sensor_msgs::ImuConstPtr imu_msg = msg_inst.instantiate<sensor_msgs::Imu>();
            if (!imu_msg) continue;

            //提取Noetic标准的IMU字段
            double ts = imu_msg->header.stamp.toSec();
            csv_file << fixed << setprecision(6)
                     << ts << ","
                     << imu_msg->linear_acceleration.x << ","
                     << imu_msg->linear_acceleration.y << ","
                     << imu_msg->linear_acceleration.z << ","
                     << imu_msg->angular_velocity.x << ","
                     << imu_msg->angular_velocity.y << ","
                     << imu_msg->angular_velocity.z << endl;
            imu_count++;
        }

        csv_file.close();
        ROS_INFO_STREAM("[IMU Extractor] " << topic << " extract finish:" 
                        << imu_count << " Article to File:" << csv_path);
    }

    //提取Leica位置数据
    void extractLeica(rosbag::Bag& bag, const string& topic, const string& save_prefix) {
        rosbag::View topic_view(bag, rosbag::TopicQuery(topic));
        if (topic_view.size() == 0) {
            ROS_WARN_STREAM("[Leica Extractor] topic " << topic << " no data,skip extract");
            return;
        }

        string csv_path = (fs::path(output_root_) / (save_prefix + "_position.csv")).string();
        ofstream csv_file(csv_path);
        if (!csv_file.is_open()) {
            ROS_ERROR_STREAM("[Leica Extractor] cannot create CSV:" << csv_path);
            return;
        }
        csv_file << "timestamp_sec,x,y,z" << endl;

        int leica_count = 0;
        for (const rosbag::MessageInstance& msg_inst : topic_view) {
            geometry_msgs::PointStampedConstPtr point_msg = msg_inst.instantiate<geometry_msgs::PointStamped>();
            if (!point_msg) continue;

            double ts = point_msg->header.stamp.toSec();
            csv_file << fixed << setprecision(6)
                     << ts << ","
                     << point_msg->point.x << ","
                     << point_msg->point.y << ","
                     << point_msg->point.z << endl;
            leica_count++;
        }

        csv_file.close();
        ROS_INFO_STREAM("[Leica Extractor] " << topic << " extract finish:" 
                        << leica_count << " Article to File:" << csv_path);
    }
};

int main(int argc, char** argv) {
    //Noetic初始化节点
    ros::init(argc, argv, "euroc_bag_splitter", ros::init_options::AnonymousName);
    ROS_INFO("[Noetic Bag Splitter] node initialization completed");

    try {
        //配置路径
        string BAG_PATH = "/home/cat/bag/MH_01_easy.bag";          //要分解的bag文件路径
        string OUTPUT_ROOT = "/home/cat/1030_test_euroc_split";    //分解后的输出目录

        //初始化分解器并执行
        EurocBagSplitter splitter(BAG_PATH, OUTPUT_ROOT);
        splitter.split();

    } catch (const exception& e) {
        ROS_FATAL_STREAM("[Noetic Bag Splitter] program abnormal exit:" << e.what());
        return 1;
    }

    //Noetic节点正常退出
    ros::shutdown();
    return 0;
}

