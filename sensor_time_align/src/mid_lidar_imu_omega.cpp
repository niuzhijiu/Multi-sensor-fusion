// 声明头文件和依赖
#include <ros/ros.h>                        // ROS节点、话题、时间、参数服务器等基础功能
#include <sensor_msgs/Imu.h>                // IMU消息类型，接收IMU数据
#include <livox_ros_driver2/CustomMsg.h>    // MID360自定义点云消息类型
#include <cv_bridge/cv_bridge.h>            // cv_bridge，OpenCV与ROS交互
#include <vector>                           // 容器，用于存储IMU和雷达缓存
#include <mutex>                            // 线程同步，保证缓存操作的线程安全
#include <thread>                           // 多线程，启动独立线程计算时间偏移，避免阻塞回调
#include <cmath>                            // 数学函数，依靠函数进行计算
#include <sstream>                          // 字符串流，to_s函数中格式化浮点数
#include <iomanip>                          // 格式化输出，to_s函数中设置小数点后6位
#include <algorithm>                        // 算法函数，插值时对齐数据长度
#include <numeric>                          // 数值计算，计算均值、互相关系数
#include <opencv2/core/core.hpp>            // OpenCV核心数据结构

// 数据结构定义
// IMU样本
struct ImuSample {
    double t;                               // IMU时间戳
    double gx, gy, gz;                      // 三轴角速度
};

// 雷达样本
struct LidarSample {
    double t;                               // 雷达帧级时间戳
    std::vector<cv::Point3f> pts;           // 有效扫描点的3D直角坐标xyz
    double frame_duration;                  // 帧持续时间=最大offset_time-最小offset_time
};

// 全局变量
std::vector<ImuSample> imu_buffer;          // IMU缓存
std::vector<LidarSample> lidar_buffer;      // 雷达缓存
std::mutex buf_mutex;                       // 全局互斥锁，保护缓存操作
bool collected = false;                     // 开始收集标记，收到第一个雷达帧置true
bool computing = false;                     // 正在计算偏移标记
bool ready_to_publish = false;              // 偏移计算完成标记
double estimated_lidar_bias = 0.0;          // 雷达与IMU的时间偏移bias
ros::Publisher pub_lidar_correct;           // 修正后雷达数据的发布者

// 校准参数
int TARGET_LIDAR_FRAMES;                    // 校准需要的雷达帧数
double LIDAR_MIN_RANGE;                     // 雷达有效距离最小值
double LIDAR_MAX_RANGE;                     // 雷达有效距离最大值
double SEARCH_MIN_BIAS;                     // 偏移搜索最小值
double SEARCH_MAX_BIAS;                     // 偏移搜索最大值
double SEARCH_STEP;                         // 搜索步长

// 工具函数
// to_s函数，浮点数转6位小数字符串
static inline std::string to_s(double v) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << v;
    return oss.str();
}

// computeCorrelation函数，计算归一化互相关系数
double computeCorrelation(const std::vector<double>& imu_s, const std::vector<double>& lidar_s) {
    // 校验，如果两个序列长度不一致或为空，直接返回0
    if (imu_s.size() != lidar_s.size() || imu_s.empty()) return 0.0;

    // 计算均值，目的是去直流分量
    double mean_imu = std::accumulate(imu_s.begin(), imu_s.end(), 0.0) / imu_s.size();
    double mean_lidar = std::accumulate(lidar_s.begin(), lidar_s.end(), 0.0) / lidar_s.size();

    // 计算互相关分子、分母，分子是协方差，分母是特征方差
    double num = 0.0, denom1 = 0.0, denom2 = 0.0;
    for (size_t i = 0; i < imu_s.size(); ++i) {
        double a = imu_s[i] - mean_imu;
        double b = lidar_s[i] - mean_lidar;
        num += a * b;                       // 分子，协方差
        denom1 += a * a;                    // 分母部分1，IMU特征方差
        denom2 += b * b;                    // 分母部分2，雷达特征方差
    }

    // 避免分母为0
    if (denom1 <= 1e-12 || denom2 <= 1e-12) return 0.0;
    return num / std::sqrt(denom1 * denom2);     // 归一化互相关系数计算
}

// interpolateImuToLidar函数
std::vector<double> interpolateImuToLidar(const std::vector<ImuSample>& imu_all,
                                          const std::vector<double>& lidar_t,
                                          const std::string& axis) {
    // 初始化插值结果数组
    std::vector<double> imu_interp;
    imu_interp.reserve(lidar_t.size());

    // 边界判断，IMU或雷达数据为空时，返回全0数组
    if (imu_all.empty() || lidar_t.empty()) {
        imu_interp.assign(lidar_t.size(), 0.0);
        return imu_interp;
    }

    // 提取IMU时间和指定轴的角速度
    std::vector<double> imu_t, imu_s;
    for (const auto& s : imu_all) {
        imu_t.push_back(s.t);
        if (axis == "x") imu_s.push_back(s.gx);           // 指定提取x轴的话，提取x轴角速度
        else if (axis == "y") imu_s.push_back(s.gy);      // 指定提取y轴的话，提取y轴角速度
        else imu_s.push_back(s.gz);                       // 如果没有指定提取哪个轴，默认提取z轴角速度
    }

    // 对每个雷达时间戳插值
    for (double t_lidar : lidar_t) {
        // 找到t_lidar在IMU时间序列中的位置，满足条件imu_t[j] ≤ t_lidar < imu_t[j+1]
        size_t j = 0;
        while (j + 1 < imu_t.size() && imu_t[j+1] < t_lidar) ++j;

        // 边界处理，t_lidar超出范围时使用边界值
        // 若其晚于所有的IMU数据，那么使用最后一个IMU值
        if (j + 1 >= imu_t.size()) {
            imu_interp.push_back(imu_s.back());
            continue;
        }
        // 若其早于所有的IMU数据，那么使用第一个IMU值
        if (j == 0 && t_lidar < imu_t[0]) {
            imu_interp.push_back(imu_s[0]);
            continue;
        }

        // 线性插值，计算t_lidar时刻的IMU角速度
        double t1 = imu_t[j], t2 = imu_t[j+1];                                   // 相邻两个IMU的时间戳
        double v1 = imu_s[j], v2 = imu_s[j+1];                                   // 相邻两个IMU的角速度值
        double alpha = (t2 - t1) > 1e-12 ? (t_lidar - t1) / (t2 - t1) : 0.0;     // 插值系数alpha
        imu_interp.push_back(v1 + alpha * (v2 - v1));                            // 线性插值公式：v=v1+α*(v2 - v1)
    }

    return imu_interp;
}

// 核心功能函数
// lidar3DPointFilter函数，过滤无效点并提取3D坐标
void lidar3DPointFilter(const livox_ros_driver2::CustomMsg::ConstPtr& msg,
                        std::vector<cv::Point3f>& pts,
                        double& frame_duration) {
    // 每次调用清空输出容器
    pts.clear();
    // 未收到雷达数据警告
    if (msg->point_num == 0) {
        ROS_WARN_THROTTLE(1.0, "Received empty lidar data");
        frame_duration = 0.0;
        return;
    }

    // 记录帧内点的最小/最大偏移时间，计算帧持续时间
    int64_t min_offset = msg->points[0].offset_time;
    int64_t max_offset = msg->points[0].offset_time;

    // 遍历每个3D点
    for (int i = 0; i < msg->point_num; ++i) {
        const auto& p = msg->points[i];
        
        // 过滤无效点：tag=0为有效点，非0为噪声/重复点
        if (p.tag != 0) continue;

        // 计算点的三维距离，过滤距离超出范围的点
        double range = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        if (range < LIDAR_MIN_RANGE || range > LIDAR_MAX_RANGE) {
            continue;
        }

        // 提取有效3D点坐标
        pts.emplace_back(p.x, p.y, p.z);

        // 更新帧内偏移时间范围
        if (p.offset_time < min_offset) min_offset = p.offset_time;
        if (p.offset_time > max_offset) max_offset = p.offset_time;
    }

    // 计算帧持续时间
    frame_duration = (max_offset - min_offset) / 1e9;

    // 过滤后有效点为0报警
    if (pts.empty()) {
        ROS_WARN_THROTTLE(1.0, "No valid points left after lidar data filtering");
    }
}

// computeLidarMotionFeature函数，计算雷达帧间旋转角速度
std::vector<double> computeLidarMotionFeature(const std::vector<LidarSample>& lidar_seg) {
    std::vector<double> motion;
    // 检验是不是有2个帧了，有2个帧就可以进行帧间旋转角速度计算了
    if (lidar_seg.size() < 2) {
        ROS_WARN("Insufficient lidar segment frames (<2), cannot compute motion features");
        return motion;
    }

    // 预分配内存，n帧雷达有n-1个偏移
    motion.reserve(lidar_seg.size() - 1);
    // 遍历相邻雷达帧
    for (size_t i = 0; i + 1 < lidar_seg.size(); ++i) {
        const auto& s1 = lidar_seg[i];
        const auto& s2 = lidar_seg[i+1];

        // 跳过无效帧
        if (s1.pts.empty() || s2.pts.empty()) {
            motion.push_back(0.0);
            continue;
        }

        // 计算前一帧3D重心
        cv::Point3f c1(0, 0, 0);
        for (const auto& p : s1.pts) {
            c1.x += p.x;
            c1.y += p.y;
            c1.z += p.z;
        }
        c1.x /= s1.pts.size();
        c1.y /= s1.pts.size();
        c1.z /= s1.pts.size();

        // 计算后一帧3D重心
        cv::Point3f c2(0, 0, 0);
        for (const auto& p : s2.pts) {
            c2.x += p.x;
            c2.y += p.y;
            c2.z += p.z;
        }
        c2.x /= s2.pts.size();
        c2.y /= s2.pts.size();
        c2.z /= s2.pts.size();

        // 计算两帧重心的Z轴旋转角
        double angle1 = atan2(c1.y, c1.x);
        double angle2 = atan2(c2.y, c2.x);
        double rot_angle = fabs(angle2 - angle1);
        // 旋转角归一化到[0, π]
        if (rot_angle > M_PI) rot_angle = 2*M_PI - rot_angle;

        // 帧间时间差
        double dt = s2.t - s1.t;
        if (dt < 1e-6) {
            motion.push_back(0.0);
            continue;
        }

        // 旋转角速度，作为匹配IMU的运动特征
        double rot_velocity = rot_angle / dt;
        motion.push_back(rot_velocity);
    }

    return motion;
}

// estimateLidarImuBias函数，计算时间偏移量
double estimateLidarImuBias(const std::vector<ImuSample>& imu_seg,
                            const std::vector<LidarSample>& lidar_seg) {
    // 校验是否有IMU数据以及雷达帧数是否足够
    if (imu_seg.empty() || lidar_seg.size() < 2) {
        ROS_ERROR("Insufficient IMU or lidar data, cannot estimate time offset");
        return 0.0;
    }

    // 提取雷达时间序列
    std::vector<double> lidar_t;
    for (const auto& s : lidar_seg) lidar_t.push_back(s.t);
    // 计算帧间旋转角速度
    std::vector<double> lidar_motion = computeLidarMotionFeature(lidar_seg);
    // 失败报警
    if (lidar_motion.empty()) {
        ROS_ERROR("Failed to compute lidar motion features, offset estimation failed");
        return 0.0;
    }

    // 提取雷达运动特征对应的时间序列，因为运动特征是帧间参数，因此去掉第一帧的时间戳
    std::vector<double> lidar_t_motion(lidar_t.begin() + 1, lidar_t.end());
    // 校验时间戳数量是否和运动特征数量一样
    if (lidar_t_motion.size() != lidar_motion.size()) {
        ROS_WARN("Mismatched length between lidar motion features and time series");
        return 0.0;
    }

    // 遍历IMU三轴搜索最优偏移
    // 初始化最佳相关系数和最佳偏移
    double best_bias = 0.0;
    double best_rho = -1.0;
    // 开始遍历IMU的三轴
    for (const std::string& axis : {"x", "y", "z"}) {
        double best_axis_bias = 0.0;
        double best_axis_rho = -1.0;

        // 遍历所有可能的偏移量，+1e-12是为了避免浮点数精度问题
        for (double bias = SEARCH_MIN_BIAS; bias <= SEARCH_MAX_BIAS + 1e-12; bias += SEARCH_STEP) {
            // 模拟某时间偏移时的雷达时间戳修正
            std::vector<double> lidar_t_shifted;
            lidar_t_shifted.reserve(lidar_t_motion.size());
            for (double t : lidar_t_motion) {
                // 模拟修正t_lidar_imu=t_lidar+bias
                lidar_t_shifted.push_back(t + bias);
            }

            // 将IMU角速度插值到修正后的雷达时间轴
            auto imu_interp = interpolateImuToLidar(imu_seg, lidar_t_shifted, axis);

            // 长度对齐，防止插值之后由于浮点数精度、插值边界处理、数据过滤等原因造成插值后IMU和雷达序列长度不一样
            // 取两序列最短长度
            size_t min_size = std::min(imu_interp.size(), lidar_motion.size());
            // 跳过空数据
            if (min_size == 0) continue;
            
            // 长的序列长度截断，短的序列长度不变
            imu_interp.resize(min_size);
            std::vector<double> lidar_motion_trimmed(lidar_motion.begin(), lidar_motion.begin() + min_size);

            // 计算互相关系数
            double rho = computeCorrelation(imu_interp, lidar_motion_trimmed);
            // 当前系数更新
            if (rho > best_axis_rho) {
                best_axis_rho = rho;
                best_axis_bias = bias;
            }
        }

        // 打印当前轴的最佳结果
        ROS_INFO("Lidar→IMU %s-axis best bias: %.3f ms (correlation coefficient: %.6f)",
                 axis.c_str(), best_axis_bias*1000.0, best_axis_rho);

        // 更新全局最优偏移
        if (best_axis_rho > best_rho) {
            best_rho = best_axis_rho;
            best_bias = best_axis_bias;
        }
    }

    // 打印全局最佳结果
    ROS_INFO("Lidar→IMU global best bias: %.3f ms (correlation coefficient: %.6f)",
             best_bias*1000.0, best_rho);
    return best_bias;
}

// 回调函数
// IMU回调
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // 看是不是没有收到消息
    if (!msg) {
        ROS_ERROR("Received empty IMU message");
        return;
    }

    // 解析IMU消息到自定义结构体s
    ImuSample s;
    s.t = msg->header.stamp.toSec();
    s.gx = msg->angular_velocity.x;
    s.gy = msg->angular_velocity.y;
    s.gz = msg->angular_velocity.z;

    // 加锁缓存IMU数据
    std::lock_guard<std::mutex> lock(buf_mutex);
    imu_buffer.push_back(s);

    // 限制IMU缓存大小，最多保留20000帧数据
    if (imu_buffer.size() > 20000) {
        imu_buffer.erase(imu_buffer.begin(), imu_buffer.end() - 20000);
    }
}

// 雷达回调
void lidarCallback(const livox_ros_driver2::CustomMsg::ConstPtr& msg) {
    // 看是不是没有收到消息
    if (!msg) {
        ROS_ERROR("Received empty lidar message");
        return;
    }

    // 过滤无效点并提取3D坐标
    std::vector<cv::Point3f> pts;
    double frame_duration = 0.0;
    lidar3DPointFilter(msg, pts, frame_duration);
    if (pts.empty()) return;

    // 加锁缓存雷达数据
    std::lock_guard<std::mutex> lock(buf_mutex);
    LidarSample s;
    s.t = msg->header.stamp.toSec();
    s.pts = pts;
    s.frame_duration = frame_duration;
    lidar_buffer.push_back(s);

    // 首次收到雷达帧之后进行标记
    if (!collected) {
        collected = true;
        ROS_INFO("First lidar frame received - start collecting %d frames for offset estimation", TARGET_LIDAR_FRAMES);
    }

    // 校准阶段：只缓存不发布
    if (!ready_to_publish) {
        // 收集满目标帧数，触发计算线程
        if ((int)lidar_buffer.size() == TARGET_LIDAR_FRAMES && !computing) {
            computing = true;
            ROS_INFO("Collected %d lidar frames - starting offset calculation thread", TARGET_LIDAR_FRAMES);

            // 启动计算线程，避免阻塞回调
            std::thread compute_thread([]() {
                // 复制数据，避免原缓存被修改
                std::vector<LidarSample> lidar_copy;
                std::vector<ImuSample> imu_copy;
                {
                    // 仅在复制数据时加锁，减少阻塞
                    std::lock_guard<std::mutex> lock(buf_mutex);
                    lidar_copy = lidar_buffer;
                    imu_copy = imu_buffer;
                }

                // 安全检查，看雷达的帧数是否足够
                if ((int)lidar_copy.size() < TARGET_LIDAR_FRAMES) {
                    ROS_ERROR("Insufficient lidar buffer data, calculation failed");
                    computing = false;
                    return;
                }

                // 确定雷达数据的时间范围
                double t0 = lidar_copy.front().t;
                double t1 = lidar_copy.back().t;
                ROS_INFO("Lidar data time range: %.6f - %.6f (duration: %.2f seconds)", t0, t1, t1 - t0);

                // 筛选该时间段内的IMU数据
                std::vector<ImuSample> imu_seg;
                for (const auto& s : imu_copy) {
                    if (s.t >= t0 - 1e-9 && s.t <= t1 + 1e-9) {
                        imu_seg.push_back(s);
                    }
                }
                // IMU帧数不足报警
                if (imu_seg.size() < 2) {
                    ROS_ERROR("Insufficient IMU data, cannot estimate offset");
                    computing = false;
                    return;
                }
                ROS_INFO("Filtered %zu IMU samples, time range: %.6f - %.6f", 
                         imu_seg.size(), imu_seg.front().t, imu_seg.back().t);

                // 估算雷达IMU之间偏移
                double bias = estimateLidarImuBias(imu_seg, lidar_copy);

                // 更新全局状态
                {
                    std::lock_guard<std::mutex> lock(buf_mutex);
                    estimated_lidar_bias = bias;
                    ready_to_publish = true;
                    computing = false;
                    lidar_buffer.clear();           // 清空校准阶段数据

                    // 打印最终结果
                    ROS_INFO("\n==================================================");
                    ROS_INFO("=== Lidar-IMU time offset calculation completed ===");
                    ROS_INFO("Offset value: %.6f s ( %.3f ms)", estimated_lidar_bias, estimated_lidar_bias * 1000.0);
                    ROS_INFO("Positive offset → lidar timestamp lags IMU | Negative offset → lidar timestamp leads IMU");
                    ROS_INFO("==================================================");
                    ROS_INFO("Enter publishing mode, subsequent lidar frames will be published with corrected timestamps");
                }
            });
            compute_thread.detach();
        }
        return;
    }

    // 修正阶段：发布时间戳修正后的雷达
    livox_ros_driver2::CustomMsgPtr out_msg = boost::make_shared<livox_ros_driver2::CustomMsg>(*msg);
    // 雷达帧级时间戳修正
    out_msg->header.stamp = ros::Time(msg->header.stamp.toSec() + estimated_lidar_bias);
    // 发布修正后的雷达数据
    pub_lidar_correct.publish(out_msg);
}

// 加载校准参数
void loadParameters(ros::NodeHandle& nh) {
    // 加载参数，具体数值的设定需要根据硬件参数来决定
    nh.param("target_lidar_frames", TARGET_LIDAR_FRAMES, 600);
    nh.param("lidar_min_range", LIDAR_MIN_RANGE, 0.15);
    nh.param("lidar_max_range", LIDAR_MAX_RANGE, 16.0);
    nh.param("search_min_bias", SEARCH_MIN_BIAS, -0.025);
    nh.param("search_max_bias", SEARCH_MAX_BIAS, 0.025);
    nh.param("search_step", SEARCH_STEP, 0.0005);

    // 打印参数信息
    ROS_INFO("Loading calibration parameters:");
    ROS_INFO("  - Target lidar frames: %d", TARGET_LIDAR_FRAMES);
    ROS_INFO("  - Valid lidar range: [%.2f, %.2f]m", LIDAR_MIN_RANGE, LIDAR_MAX_RANGE);
    ROS_INFO("  - Offset search range: [%.3f, %.3f]s (step: %.3fms)",
             SEARCH_MIN_BIAS, SEARCH_MAX_BIAS, SEARCH_STEP*1000);
}

// 主函数
int main(int argc, char** argv) {
    // 节点的初始化
    ros::init(argc, argv, "lidar_imu_omega_node");
    ros::NodeHandle nh("~");         // 使用私有命名空间，方便参数配置

    // 加载校准参数
    loadParameters(nh);

    // 订阅话题
    std::string imu_topic, lidar_topic, output_topic;
    nh.param("imu_topic", imu_topic, std::string("/imu_raw"));
    nh.param("lidar_topic", lidar_topic, std::string("/livox/lidar"));
    nh.param("output_topic", output_topic, std::string("/scan_correct"));
    
    // 创建订阅者，订阅IMU和雷达话题
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 20000, imuCallback);
    ros::Subscriber sub_lidar = nh.subscribe(lidar_topic, 1000, lidarCallback);

    // 创建发布者，发布修正后雷达话题
    pub_lidar_correct = nh.advertise<livox_ros_driver2::CustomMsg>(output_topic, 1000);

    // 打印启动信息
    ROS_INFO("Lidar-IMU timestamp alignment node started:");
    ROS_INFO("  - Subscribed IMU topic: %s", imu_topic.c_str());
    ROS_INFO("  - Subscribed lidar topic: %s", lidar_topic.c_str());
    ROS_INFO("  - Published corrected lidar topic: %s", output_topic.c_str());
    ROS_INFO("  - Required lidar frames for calibration: %d", TARGET_LIDAR_FRAMES);
    ROS_INFO("Waiting for lidar and IMU data...");

    // 阻塞式处理回调，直到节点关闭
    ros::spin();
    return 0;
}
