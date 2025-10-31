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

// ---------------- 数据结构 ----------------
struct ImuSample {
    double t;
    double gx, gy, gz;
};

struct ImageSample {
    double t;
    cv::Mat img; // 保留图像用于视觉角速度计算
};

// 视觉角速度数据结构（对相邻帧的中点时间）
struct VisSample {
    double t;
    double wx, wy, wz;
};

// ---------------- 全局状态 ----------------
std::vector<ImuSample> imu_buffer;       // 缓存收到的IMU样本（全局不断追加）
std::vector<ImageSample> image_buffer;   // 缓存收到的图像样本（用于前3000帧和计算）
std::mutex buf_mutex;

bool collected = false;          // 是否已经开始收集（收到第一帧图像时置 true）
bool computing = false;          // 是否正在计算偏移
bool ready_to_publish = false;   // 偏移计算完成后置 true，之后收到的图像将被修正并发布
double estimated_bias = 0.0;     // 估计出的偏移（秒）

ros::Publisher pub_correct;      // 发布修正后图像

const int TARGET_IMAGES = 3000;  // 需要用于估计的图像帧数（3000）

// ---------------- 工具函数 ----------------
static inline std::string to_s(double v) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << v;
    return oss.str();
}

// 计算视觉角速度：对 image_buffer 中相邻帧计算旋转向量 r / dt -> omega
std::vector<VisSample> computeVisualAngularVelocity(const std::vector<ImageSample>& imgs) {
    std::vector<VisSample> vis;
    if (imgs.size() < 2) return vis;

    // 内参
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

        // 特征提取与匹配
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
        vis.push_back(v);
    }

    return vis;
}

// 过滤出在 time range [t0, t1] 内的 imu 样本（包含边界）
std::vector<ImuSample> filterImuInRange(const std::vector<ImuSample>& imu_all, double t0, double t1) {
    std::vector<ImuSample> out;
    out.reserve(imu_all.size());
    for (const auto& s : imu_all) {
        if (s.t >= t0 - 1e-9 && s.t <= t1 + 1e-9) out.push_back(s);
    }
    return out;
}

// 插值视觉信号到 imu 时间序列（按单轴值 vis_s）
std::vector<double> interpolateVisToImu(const std::vector<VisSample>& vis,
                                        const std::vector<double>& imu_t,
                                        const std::vector<double>& vis_s,
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
            vis_interp.push_back(vis_s.back());
            continue;
        }
        double t1 = vis[j].t, t2 = vis[j+1].t;
        double v1 = vis_s[j], v2 = vis_s[j+1];
        double alpha = 0.0;
        if (t2 - t1 > 1e-12) alpha = (shifted_t - t1) / (t2 - t1);
        vis_interp.push_back(v1 + alpha * (v2 - v1));
    }
    return vis_interp;
}

// 归一化互相关
double computeCorrelation(const std::vector<double>& imu_s, const std::vector<double>& vis_s) {
    if (imu_s.size() != vis_s.size() || imu_s.empty()) return 0.0;
    double mean_imu = 0.0, mean_vis = 0.0;
    for (size_t i = 0; i < imu_s.size(); ++i) { mean_imu += imu_s[i]; mean_vis += vis_s[i]; }
    mean_imu /= imu_s.size(); mean_vis /= vis_s.size();
    double num = 0.0, denom1 = 0.0, denom2 = 0.0;
    for (size_t i = 0; i < imu_s.size(); ++i) {
        double a = imu_s[i] - mean_imu;
        double b = vis_s[i] - mean_vis;
        num += a*b; denom1 += a*a; denom2 += b*b;
    }
    if (denom1 <= 0.0 || denom2 <= 0.0) return 0.0;
    return num / std::sqrt(denom1 * denom2);
}

// 估计偏移：给定段内 IMU 与 Vis（已经截取到同一时间段），搜索 bias
double estimateBiasFromSegment(const std::vector<ImuSample>& imu_seg, const std::vector<VisSample>& vis) {
    if (imu_seg.empty() || vis.empty()) return 0.0;

    // 构造 imu 时间序列和轴数据
    std::vector<double> imu_t, imu_x, imu_y, imu_z;
    imu_t.reserve(imu_seg.size());
    imu_x.reserve(imu_seg.size());
    imu_y.reserve(imu_seg.size());
    imu_z.reserve(imu_seg.size());
    for (const auto& s : imu_seg) {
        imu_t.push_back(s.t);
        imu_x.push_back(s.gx);
        imu_y.push_back(s.gy);
        imu_z.push_back(s.gz);
    }

    // 构造视觉轴序列
    std::vector<double> vis_x, vis_y, vis_z;
    vis_x.reserve(vis.size()); vis_y.reserve(vis.size()); vis_z.reserve(vis.size());
    for (const auto& v : vis) { vis_x.push_back(v.wx); vis_y.push_back(v.wy); vis_z.push_back(v.wz); }

    double best_bias = 0.0;
    double best_rho = -1.0;

    // 对三轴分别搜索
    for (const std::string& axis : {"x","y","z"}) {
        const std::vector<double>* imu_s_ptr = nullptr;
        const std::vector<double>* vis_s_ptr = nullptr;
        if (axis == "x") { imu_s_ptr = &imu_x; vis_s_ptr = &vis_x; }
        else if (axis == "y") { imu_s_ptr = &imu_y; vis_s_ptr = &vis_y; }
        else { imu_s_ptr = &imu_z; vis_s_ptr = &vis_z; }

        double best_axis_bias = 0.0;
        double best_axis_rho = -1.0;

        // bias 搜索范围 -25ms .. +25ms, 步长 1ms
        for (double bias = -0.025; bias <= 0.025 + 1e-12; bias += 0.001) {
            // 将视觉插值到 imu 时间上
            auto vis_interp = interpolateVisToImu(vis, imu_t, *vis_s_ptr, bias);
            double rho = computeCorrelation(*imu_s_ptr, vis_interp);
            if (rho > best_axis_rho) {
                best_axis_rho = rho;
                best_axis_bias = bias;
            }
        }
        ROS_INFO("axis %s best bias = %.3f ms (rho=%.6f)", axis.c_str(), best_axis_bias*1000.0, best_axis_rho);
        if (best_axis_rho > best_rho) {
            best_rho = best_axis_rho;
            best_bias = best_axis_bias;
        }
    }

    ROS_INFO("best overall bias = %.3f ms (rho=%.6f)", best_bias*1000.0, best_rho);
    return best_bias;
}

// ---------------- 回调函数 ----------------
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // 始终缓存 IMU 样本（包括偏移估计阶段与之后）
    ImuSample s;
    s.t = msg->header.stamp.toSec();
    s.gx = msg->angular_velocity.x;
    s.gy = msg->angular_velocity.y;
    s.gz = msg->angular_velocity.z;

    std::lock_guard<std::mutex> lock(buf_mutex);
    imu_buffer.push_back(s);
}

// image 回调：收集前 3000 帧用于估计；估计完成后将每帧修正并发布
void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    double t = msg->header.stamp.toSec();

    // 转成 cv::Mat（bgr8 或 monochrome 都尝试）
    cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        img = cv_ptr->image.clone();
    } catch (cv_bridge::Exception& e) {
        try {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "mono8");
            cv::cvtColor(cv_ptr->image, img, cv::COLOR_GRAY2BGR);
        } catch (...) {
            ROS_WARN("imageCallback: unsupported image encoding, skipping this image");
            return;
        }
    }

    std::lock_guard<std::mutex> lock(buf_mutex);

    if (!collected) {
        // 第一帧图像到来，开始收集
        collected = true;
        ROS_INFO("First image received - start collecting %d images for bias estimation", TARGET_IMAGES);
    }

    if (!ready_to_publish) {
        // 仍在收集或正在计算期间，把图像缓存（这些图像不会被发布）
        ImageSample s; s.t = t; s.img = img;
        image_buffer.push_back(s);

        // 检查是否达到目标帧数：当刚好等于 TARGET_IMAGES 时触发计算（在另一个线程中）
        if ((int)image_buffer.size() == TARGET_IMAGES && !computing) {
            computing = true;
            ROS_INFO("Collected %d images - launching bias computation thread", TARGET_IMAGES);
            // 开线程计算，不在回调阻塞
            std::thread compute_thread([](){
                // 复制用于计算的数据（在互斥下）
                std::vector<ImageSample> imgs_copy;
                std::vector<ImuSample> imu_copy;
                {
                    std::lock_guard<std::mutex> lock(buf_mutex);
                    imgs_copy = image_buffer; // 包含前 3000 帧（以及可能在计算期间到达的更多帧，但我们将只使用前3000）
                    imu_copy = imu_buffer;    // 包含自节点启动至今所有 IMU 样本
                }

                // 为安全起见，确保 imgs_copy 至少有 TARGET_IMAGES 帧
                if ((int)imgs_copy.size() < TARGET_IMAGES) {
                    ROS_ERROR("compute thread: unexpected small image buffer");
                    computing = false;
                    return;
                }

                // 定位用于计算的时间段：使用前 TARGET_IMAGES 帧的时间范围
                double t0 = imgs_copy.front().t;
                double t1 = imgs_copy[TARGET_IMAGES - 1].t;

                // 截取 IMU 段（只取在 [t0, t1] 的 IMU）
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

                // 计算视觉角速度（仅使用前 TARGET_IMAGES 帧）
                std::vector<ImageSample> imgs_for_vis(imgs_copy.begin(), imgs_copy.begin() + TARGET_IMAGES);
                auto vis = computeVisualAngularVelocity(imgs_for_vis);
                if (vis.size() < 2) {
                    ROS_ERROR("Not enough visual angular velocity samples to estimate bias (got %zu)", vis.size());
                    computing = false;
                    return;
                }

                // 将 imu_seg 直接作为 imu 段（包含 gx,gy,gz）
                double bias = estimateBiasFromSegment(imu_seg, vis);

                // 设置结果并进入发布阶段
                {
                    std::lock_guard<std::mutex> lock(buf_mutex);
                    estimated_bias = bias;
                    ready_to_publish = true;
                    computing = false;
                    // 按你的要求：这前 TARGET_IMAGES 帧“仅用于计算”，不发布 -> 清空 image_buffer
                    image_buffer.clear();
                    
                    // 清晰打印最终时间偏移量b
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
        // 不发布在收集/计算阶段到达的图像
        return;
    }

    // ready_to_publish == true：对到达的每帧图像进行时间戳修正并发布
    sensor_msgs::ImagePtr out_msg;
    {
        // 创建 sensor_msgs::Image 消息并填充（保持原 header 的 frame_id）
        std_msgs::Header hdr;
        hdr.stamp = ros::Time(t + estimated_bias);
        // copy encoding and image data via cv_bridge
        cv_bridge::CvImage cv_out(hdr, "bgr8", img);
        out_msg = cv_out.toImageMsg();
    }
    pub_correct.publish(out_msg);
}

// ---------------- main ----------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "timestamp_correct_node");
    ros::NodeHandle nh;

    // 订阅 imu 与 image
    ros::Subscriber sub_imu = nh.subscribe("/imu_raw", 20000, imuCallback);
    ros::Subscriber sub_img = nh.subscribe("/image_timestamp_raw", 2000, imageCallback);

    pub_correct = nh.advertise<sensor_msgs::Image>("/image_correct_raw", 2000);

    ROS_INFO("timestamp_correct_node started. Waiting for images and imu...");
    ros::spin();

    return 0;
}

