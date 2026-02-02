// 声明头文件和依赖
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>
#include <thread>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>

// 定义数据结构
struct ImuSample {
    double t;
    double gx, gy, gz;
    double g_norm;     // IMU角速度模值
};

struct ImageSample {
    double t;
    cv::Mat img; 
};

struct VisSample {
    double t;
    double wx, wy, wz;
    double w_norm;     // 视觉角速度模值
};

// 定义全局变量
std::vector<ImuSample> imu_buffer;
std::vector<ImageSample> image_buffer;
std::mutex buf_mutex;

bool collected = false;
bool computing = false;
bool ready_to_publish = false;
double estimated_bias = 0.0;

ros::Publisher pub_correct;

const int TARGET_IMAGES = 2000;           // 校准阶段帧数，可根据实际需求修改

static inline std::string to_s(double v) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << v;
    return oss.str();
}

// 计算视觉角速度以及模值
std::vector<VisSample> computeVisualAngularVelocity(const std::vector<ImageSample>& imgs) {
    std::vector<VisSample> vis;
    if (imgs.size() < 2) return vis;

    // 相机内参，需要根据实际使用的相机硬件参数来设置
    cv::Mat K = (cv::Mat_<double>(3,3) <<
        685.0, 0.0, 320.0,
        0.0, 685.0, 240.0,
        0.0, 0.0, 1.0);

    cv::Ptr<cv::ORB> orb = cv::ORB::create(2000);

    for (size_t i = 0; i + 1 < imgs.size(); ++i) {
        double t1 = imgs[i].t;
        double t2 = imgs[i+1].t;
        double dt = t2 - t1;
        if (dt <= 0.0) continue;

        std::vector<cv::KeyPoint> kp1, kp2;
        cv::Mat des1, des2;
        orb->detectAndCompute(imgs[i].img, cv::Mat(), kp1, des1);
        orb->detectAndCompute(imgs[i+1].img, cv::Mat(), kp2, des2);
        if (des1.empty() || des2.empty()) continue;

        cv::BFMatcher bf(cv::NORM_HAMMING, true);
        std::vector<cv::DMatch> matches;
        bf.match(des1, des2, matches);
        if (matches.size() < 5) continue;

        std::vector<cv::Point2f> pts1, pts2;
        pts1.reserve(matches.size());
        pts2.reserve(matches.size());
        for (auto &m : matches) {
            pts1.push_back(kp1[m.queryIdx].pt);
            pts2.push_back(kp2[m.trainIdx].pt);
        }

        cv::Mat mask;
        cv::Mat E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999, 1.0, mask);
        if (E.empty()) continue;
        if (cv::countNonZero(mask) < 5) continue;

        cv::Mat R, tvec;
        cv::recoverPose(E, pts1, pts2, K, R, tvec, mask);
        
        cv::Mat rvec;
        cv::Rodrigues(R, rvec);
        if (rvec.rows != 3 || rvec.cols != 1) continue;

        cv::Vec3d rotvec(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0));
        cv::Vec3d omega = rotvec / dt;

        VisSample v;
        v.t = 0.5 * (t1 + t2);
        v.wx = omega[0];
        v.wy = omega[1];
        v.wz = omega[2];
        // 计算视觉角速度模值
        v.w_norm = std::sqrt(v.wx*v.wx + v.wy*v.wy + v.wz*v.wz);
        vis.push_back(v);
    }

    return vis;
}

std::vector<ImuSample> filterImuInRange(const std::vector<ImuSample>& imu_all, double t0, double t1) {
    std::vector<ImuSample> out;
    out.reserve(imu_all.size());
    for (const auto& s : imu_all) {
        if (s.t >= t0 - 1e-9 && s.t <= t1 + 1e-9) out.push_back(s);
    }
    return out;
}

// 插值视觉模值到IMU时间序列
std::vector<double> interpolateVisNormToImu(const std::vector<VisSample>& vis,
                                          const std::vector<double>& imu_t,
                                          double bias) {
    std::vector<double> vis_interp;
    vis_interp.reserve(imu_t.size());
    if (vis.empty()) {
        vis_interp.assign(imu_t.size(), 0.0);
        return vis_interp;
    }

    for (double t : imu_t) {
        double shifted_t = t - bias;
        size_t j = 0;
        while (j + 1 < vis.size() && vis[j+1].t < shifted_t) ++j;

        if (j + 1 >= vis.size()) {
            vis_interp.push_back(vis.back().w_norm);
            continue;
        }

        double t1 = vis[j].t, t2 = vis[j+1].t;
        double v1 = vis[j].w_norm, v2 = vis[j+1].w_norm;
        double alpha = 0.0;
        if (t2 - t1 > 1e-12) alpha = (shifted_t - t1) / (t2 - t1);
        vis_interp.push_back(v1 + alpha * (v2 - v1));
    }
    return vis_interp;
}

// 计算模值序列的互相关系数
double computeNormCorrelation(const std::vector<double>& imu_norm, const std::vector<double>& vis_norm) {
    if (imu_norm.size() != vis_norm.size() || imu_norm.empty()) return 0.0;

    double mean_imu = 0.0, mean_vis = 0.0;
    for (size_t i = 0; i < imu_norm.size(); ++i) {
        mean_imu += imu_norm[i];
        mean_vis += vis_norm[i];
    }
    mean_imu /= imu_norm.size();
    mean_vis /= vis_norm.size();

    double num = 0.0, denom1 = 0.0, denom2 = 0.0;
    for (size_t i = 0; i < imu_norm.size(); ++i) {
        double a = imu_norm[i] - mean_imu;
        double b = vis_norm[i] - mean_vis;
        num += a*b;
        denom1 += a*a;
        denom2 += b*b;
    }

    if (denom1 <= 0.0 || denom2 <= 0.0) return 0.0;
    return num / std::sqrt(denom1 * denom2);
}

// 基于模值估计时间偏移
double estimateBiasFromSegment(const std::vector<ImuSample>& imu_seg, const std::vector<VisSample>& vis) {
    if (imu_seg.empty() || vis.empty()) return 0.0;

    // 提取IMU时间戳和模值序列
    std::vector<double> imu_t, imu_norm;
    imu_t.reserve(imu_seg.size());
    imu_norm.reserve(imu_seg.size());
    for (const auto& s : imu_seg) {
        imu_t.push_back(s.t);
        imu_norm.push_back(s.g_norm);
    }

    // 初始化最佳偏移和相关系数
    double best_bias = 0.0;
    double best_rho = -1.0;

    // 搜索最佳偏移
    for (double bias = -0.025; bias <= 0.025 + 1e-12; bias += 0.0005) {
        // 插值视觉模值到IMU时间序列
        auto vis_interp = interpolateVisNormToImu(vis, imu_t, bias);
        // 计算模值序列的互相关系数
        double rho = computeNormCorrelation(imu_norm, vis_interp);
        
        // 更新最佳结果
        if (rho > best_rho) {
            best_rho = rho;
            best_bias = bias;
        }
    }

    ROS_INFO("best norm-based bias = %.3f ms (rho=%.6f)", best_bias*1000.0, best_rho);
    return best_bias;
}

// IMU回调函数
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ImuSample s;
    s.t = msg->header.stamp.toSec();
    s.gx = msg->angular_velocity.x;
    s.gy = msg->angular_velocity.y;
    s.gz = msg->angular_velocity.z;
    // 计算IMU角速度模值
    s.g_norm = std::sqrt(s.gx*s.gx + s.gy*s.gy + s.gz*s.gz);
    
    std::lock_guard<std::mutex> lock(buf_mutex);
    imu_buffer.push_back(s);
}

// 图像回调函数
void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    double t = msg->header.stamp.toSec();

    // 将ROS图像消息sensor_msgs/Image转为OpenCV的cv::Mat格式
    cv::Mat img;  // 保存原始彩色图，用于最终发布
    try {
        // 先判断原始编码是否为rgb8格式
        if (msg->encoding == "rgb8") {
            // 若是rgb8格式，将rgb8格式转为bgr8格式，bgr8格式是OpenCV原生格式
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "rgb8");
            cv::cvtColor(cv_ptr->image, img, cv::COLOR_RGB2BGR);
        } else {
            // 若不是rgb8格式，优先尝试去读取转化bgr8格式
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            img = cv_ptr->image.clone();
        }
    } catch (cv_bridge::Exception& e) {
        try {
            // 不是bgr8格式的话尝试mono8格式
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "mono8");
            cv::cvtColor(cv_ptr->image, img, cv::COLOR_GRAY2BGR);
        } catch (...) {
            // 所有编码都失败则跳过
            ROS_WARN("imageCallback: unsupported image encoding (%s), skipping this image", msg->encoding.c_str());
            return;
        }
    }

    std::lock_guard<std::mutex> lock(buf_mutex);

    if (!collected) {
        collected = true;
        ROS_INFO("First image received - start collecting %d images for bias estimation", TARGET_IMAGES);
    }

    if (!ready_to_publish) {
        // 校准阶段将彩色图转为灰度图，仅用灰度图参与计算
        cv::Mat gray_img;  // 灰度图，仅用于校准计算
        if (img.channels() == 3) {  
            // 若是彩色图则转灰度图
            cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
        } else {  
            // 已是灰度图则直接复用
            gray_img = img.clone();
        }

        // 缓存灰度图用于校准计算
        ImageSample s; s.t = t; s.img = gray_img;
        image_buffer.push_back(s);

        if ((int)image_buffer.size() == TARGET_IMAGES && !computing) {
            computing = true;
            ROS_INFO("Collected %d images - launching bias computation thread", TARGET_IMAGES);
            std::thread compute_thread([](){
                std::vector<ImageSample> imgs_copy;
                std::vector<ImuSample> imu_copy;
                {
                    std::lock_guard<std::mutex> lock(buf_mutex);
                    imgs_copy = image_buffer;  
                    imu_copy = imu_buffer;  
                }

                if ((int)imgs_copy.size() < TARGET_IMAGES) {
                    ROS_ERROR("compute thread: unexpected small image buffer");
                    computing = false;
                    return;
                }

                double t0 = imgs_copy.front().t;
                double t1 = imgs_copy[TARGET_IMAGES - 1].t;

                std::vector<ImuSample> imu_seg;
                imu_seg.reserve(imu_copy.size());
                for (const auto& s : imu_copy) {
                    if (s.t >= t0 - 1e-9 && s.t <= t1 + 1e-9) imu_seg.push_back(s);
                }

                if (imu_seg.size() < 2) {
                    ROS_ERROR("Not enough IMU samples in the image interval to estimate bias (needed >=2, got %zu)", imu_seg.size());
                    computing = false;
                    return;
                }

                std::vector<ImageSample> imgs_for_vis(imgs_copy.begin(), imgs_copy.begin() + TARGET_IMAGES);
                auto vis = computeVisualAngularVelocity(imgs_for_vis);
                
                if (vis.size() < 2) {
                    ROS_ERROR("Not enough visual angular velocity samples to estimate bias (got %zu)", vis.size());
                    computing = false;
                    return;
                }

                double bias = estimateBiasFromSegment(imu_seg, vis);

                {
                    std::lock_guard<std::mutex> lock(buf_mutex);
                    estimated_bias = bias;
                    ready_to_publish = true;
                    computing = false;
                    image_buffer.clear();
                    
                    ROS_INFO("\n==================================================");
                    ROS_INFO("===  bias calculation completed ===");
                    ROS_INFO("b = %.6f s ( %.3f ms)", estimated_bias, estimated_bias * 1000.0);
                    ROS_INFO("b is positive so camera timestamp lags behind the IMU       b is negative so camera timestamp is ahead of IMU");
                    ROS_INFO("==================================================");
                    ROS_INFO("Now entering publish mode; subsequent images will be corrected and published.");
                }
            });
            compute_thread.detach();
        }
        return;
    }

    // 修正阶段：使用原始彩色图发布，无灰度处理
    sensor_msgs::ImagePtr out_msg;
    {
        std_msgs::Header hdr;
        hdr.stamp = ros::Time(t + estimated_bias);
        // 将OpenCV彩色图像转为ROS消息，编码为bgr8
        cv_bridge::CvImage cv_out(hdr, "bgr8", img);
        out_msg = cv_out.toImageMsg();
    }
    pub_correct.publish(out_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_imu_norm_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_imu = nh.subscribe("/imu_raw", 20000, imuCallback);
    ros::Subscriber sub_img = nh.subscribe("/image_timestamp_raw", 2000, imageCallback);
    pub_correct = nh.advertise<sensor_msgs::Image>("/image_correct_raw", 2000);
    ROS_INFO("timestamp_correct_node_norm started. Waiting for images and imu...");
    ros::spin();
    return 0;
}
