// 声明头文件和依赖
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>
#include <thread>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <boost/make_shared.hpp>

// 定义数据结构
// ImuSample存储单条IMU样本的时间戳和角速度
struct ImuSample {
    double t;
    double gx, gy, gz;
};

// ImageSample存储单帧图像样本的时间戳和图像信息
struct ImageSample {
    double t;
    cv::Mat img; 
};

// VisSample存储计算出的视觉角速度样本的时间戳和角速度
struct VisSample {
    double t;
    double wx, wy, wz;
};

// LidarSample存储单帧雷达样本的时间戳和有效扫描点坐标
struct LidarSample {
    double t;
    std::vector<cv::Point2f> pts;
};

// 命名空间隔离：图像校准相关变量
namespace ImageCalib {
    // 定义图像校准专用全局变量
    std::vector<ImageSample> image_buffer;     // 缓存收到的图像样本
    bool collected = false;                    // 开始收集状态标记，收到第一帧图像时置true
    bool computing = false;                    // 计算偏移状态标记，开始计算时间偏移时置true
    bool ready_to_publish = false;             // 完成偏移计算状态标记，偏移计算完成后置true
    double estimated_bias = 0.0;               // 估计出的偏移
    ros::Publisher pub_correct;                // 发布修正后图像的发布者
    const int TARGET_IMAGES = 2000;            // 校准阶段需要用于估计的图像帧数，可以设置成其它数值
}

// 命名空间隔离：雷达校准相关变量
namespace LidarCalib {
    // 定义雷达校准专用全局变量
    std::vector<LidarSample> lidar_buffer;     // 缓存收到的雷达样本
    bool collected = false;                    // 开始收集状态标记，收到第一帧雷达时置true
    bool computing = false;                    // 计算偏移状态标记，开始计算时间偏移时置true
    bool ready_to_publish = false;             // 完成偏移计算状态标记，偏移计算完成后置true
    double estimated_bias = 0.0;               // 估计出的偏移
    ros::Publisher pub_correct;                // 发布修正后雷达的发布者

    // 雷达校准参数
    int TARGET_LIDAR_FRAMES = 600;             // 校准阶段需要用于估计的雷达帧数
    double LIDAR_MIN_RANGE = 0.15;             // 雷达有效距离最小值，此参数根据雷达硬件参数特性设置
    double LIDAR_MAX_RANGE = 16.0;             // 雷达有效距离最大值，此参数根据雷达硬件参数特性设置
    double SEARCH_MIN_BIAS = -0.025;           // 偏移搜索最小值
    double SEARCH_MAX_BIAS = 0.025;            // 偏移搜索最大值
    double SEARCH_STEP = 0.0005;               // 偏移搜索步长
}

// 图像和雷达共用的共享全局变量
std::vector<ImuSample> imu_buffer;             // 缓存收到的IMU样本，全局不断追加
std::mutex buf_mutex;                          // 全局互斥锁，保护所有缓存操作

// to_s函数：将浮点数v转为保留6位小数的字符串
static inline std::string to_s(double v) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << v;
    return oss.str();
}

// 视觉惯性时间校准部分
// computeVisualAngularVelocity函数：计算视觉角速度
std::vector<VisSample> computeVisualAngularVelocity(const std::vector<ImageSample>& imgs) {
    std::vector<VisSample> vis;
    if (imgs.size() < 2) return vis;

    // 相机内参矩阵，此参数根据相机硬件参数特性设置，此处为d435的内参
    cv::Mat K = (cv::Mat_<double>(3,3) <<
        384.0, 0.0, 320.0,
        0.0, 384.0, 240.0,
        0.0, 0.0, 1.0);

    // 创建ORB特征提取器，提取图像特征点和描述子，可以改成别的数值
    cv::Ptr<cv::ORB> orb = cv::ORB::create(2000);

    // 遍历相邻图像帧，计算角速度
    for (size_t i = 0; i + 1 < imgs.size(); ++i) {
        // 计算帧间时间差
        double t1 = imgs[i].t;
        double t2 = imgs[i+1].t;
        double dt = t2 - t1;
        if (dt <= 0.0) continue;

        // 对相邻两帧图像提取ORB特征
        std::vector<cv::KeyPoint> kp1, kp2;                             // 存储特征点
        cv::Mat des1, des2;                                             // 存储特征描述子
        orb->detectAndCompute(imgs[i].img, cv::Mat(), kp1, des1);       // 对前一帧进行特征提取
        orb->detectAndCompute(imgs[i+1].img, cv::Mat(), kp2, des2);     // 对后一帧进行特征提取
        if (des1.empty() || des2.empty()) continue;

        // 使用bf即Brute Force Matcher暴力匹配器进行特征匹配
        cv::BFMatcher bf(cv::NORM_HAMMING, true);                       // 特征匹配是基于汉明距离的
        std::vector<cv::DMatch> matches;
        bf.match(des1, des2, matches);                                  // 进行匹配
        if (matches.size() < 5) continue;                               // 匹配点个数少于5的话不予匹配直接跳过

        // 提取匹配的特征点的坐标
        std::vector<cv::Point2f> pts1, pts2;
        pts1.reserve(matches.size());
        pts2.reserve(matches.size());
        for (auto &m : matches) {
            pts1.push_back(kp1[m.queryIdx].pt);
            pts2.push_back(kp2[m.trainIdx].pt);
        }

        // 估计本质矩阵E
        cv::Mat mask;                                                                     // 掩码标记有效的内点和无效的外点
        cv::Mat E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999, 1.0, mask);    // 估计本质矩阵E
        if (E.empty()) continue;                                                          // 跳过估计失败的情况
        // 校验E矩阵是否为3×3方阵
        if (E.rows != 3 || E.cols != 3) continue;                                         // 跳过非3×3的无效本质矩阵
        if (cv::countNonZero(mask) < 5) continue;                                         // 跳过匹配的有效点太少的情况

        // 由本质矩阵E得到旋转矩阵R
        cv::Mat R, tvec;
        cv::recoverPose(E, pts1, pts2, K, R, tvec, mask);
        
        // 校验旋转矩阵R是否为3×3方阵
        if (R.empty() || R.rows != 3 || R.cols != 3) continue;                            // 跳过无效的旋转矩阵

        // 将旋转矩阵R转为旋转向量rvec，使用Rodrigues变化
        cv::Mat rvec;
        cv::Rodrigues(R, rvec);
        if (rvec.rows != 3 || rvec.cols != 1) continue;

        // 计算视觉角速度=旋转向量/帧间时间差dt
        cv::Vec3d rotvec(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0));
        cv::Vec3d omega = rotvec / dt;

        // 存储视觉角速度，时间戳取两帧中点，代表该角速度的有效时间
        VisSample v;
        v.t = 0.5 * (t1 + t2);
        v.wx = omega[0];
        v.wy = omega[1];
        v.wz = omega[2];
        vis.push_back(v);
    }

    return vis;
}

// filterImuInRange函数：过滤出指定时间范围内的IMU样本。从所有IMU缓存中，筛选出时间戳在[t0, t1]范围内的样本，用于与该时间段的图像数据对应
std::vector<ImuSample> filterImuInRange(const std::vector<ImuSample>& imu_all, double t0, double t1) {
    std::vector<ImuSample> out;
    out.reserve(imu_all.size());
    for (const auto& s : imu_all) {
        if (s.t >= t0 - 1e-9 && s.t <= t1 + 1e-9) out.push_back(s);
    }
    return out;
}

// interpolateVisToImu函数：插值视觉信号到imu信号的时间序列
std::vector<double> interpolateVisToImu(const std::vector<VisSample>& vis,
                                        const std::vector<double>& imu_t,
                                        const std::vector<double>& vis_s,
                                        double bias) {
    // 存储插值结果的容器
    std::vector<double> vis_interp;
    // 预分配内存容量=IMU时间戳数量,提高效率
    vis_interp.reserve(imu_t.size());
    // 边界处理：视觉为空时候用0填充结果
    if (vis.empty()) {
        vis_interp.assign(imu_t.size(), 0.0);
        return vis_interp;
    }
    // 遍历每个IMU时间戳t
    for (double t : imu_t) {
        // 根据t_cam+t_bias=t_imu逻辑把imu先和视觉统一到视觉时间尺度
        double shifted_t = t - bias;
        // 用于定位shifted_t在视觉样本中的位置
        size_t j = 0;
        // 循环找到shifted_t所在的视觉时间区间
        while (j + 1 < vis.size() && vis[j+1].t < shifted_t) ++j;
        // 边界处理：如果shifted_t超过最后一个视觉样本的时间，则用最后一个视觉值填充
        if (j + 1 >= vis.size()) {
            vis_interp.push_back(vis_s.back());
            continue;
        }
        // 提取当前视觉区间的时间[t1, t2]
        double t1 = vis[j].t, t2 = vis[j+1].t;
        // 提取当前视觉区间的值[v1, v2]
        double v1 = vis_s[j], v2 = vis_s[j+1];
        // 定义插值系数alpha
        double alpha = 0.0;
        // 计算插值系数：若时间差不为0，alpha=(当前时间-起始时间)/区间总时间
        if (t2 - t1 > 1e-12) alpha = (shifted_t - t1) / (t2 - t1);
        // 线性插值v1+alpha*(v2-v1)，得到shifted_t对应的视觉角速度
        vis_interp.push_back(v1 + alpha * (v2 - v1));
    }
    return vis_interp;
}

// computeCorrelation函数：计算IMU角速度与插值后的视觉角速度的归一化互相关系数
double computeCorrelation(const std::vector<double>& imu_s, const std::vector<double>& vis_s) {
    if (imu_s.size() != vis_s.size() || imu_s.empty()) return 0.0;
    // 计算两个信号的均值，之后可以去均值防止直流分量的影响
    double mean_imu = 0.0, mean_vis = 0.0;
    for (size_t i = 0; i < imu_s.size(); ++i) { mean_imu += imu_s[i]; mean_vis += vis_s[i]; }
    mean_imu /= imu_s.size(); mean_vis /= vis_s.size();
    // 计算互相关公式的分子和分母
    double num = 0.0, denom1 = 0.0, denom2 = 0.0;
    for (size_t i = 0; i < imu_s.size(); ++i) {
        double a = imu_s[i] - mean_imu;                 // IMU信号去均值
        double b = vis_s[i] - mean_vis;                 // 视觉信号去均值
        num += a*b; denom1 += a*a; denom2 += b*b;       // 计算互相关公式的分子和分母
    }
    // 避免信号无波动时分母为0，返回0表示无相关
    if (denom1 <= 0.0 || denom2 <= 0.0) return 0.0;
    // 根据公式计算归一化互相关系数
    return num / std::sqrt(denom1 * denom2);
}

// estimateBiasFromSegment函数：给定同一时间段内IMU与Vis数据，遍历搜索bias，取系数最大的偏移作为最终结果，进行时间偏移量的估算
double estimateBiasFromSegment(const std::vector<ImuSample>& imu_seg, const std::vector<VisSample>& vis) {
    if (imu_seg.empty() || vis.empty()) return 0.0;

    // 提取IMU段的时间序列和三轴角速度
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

    // 提取视觉三轴角速度
    std::vector<double> vis_x, vis_y, vis_z;
    vis_x.reserve(vis.size()); vis_y.reserve(vis.size()); vis_z.reserve(vis.size());
    for (const auto& v : vis) { vis_x.push_back(v.wx); vis_y.push_back(v.wy); vis_z.push_back(v.wz); }

    // 初始化最优偏移和最优互相关系数
    double best_bias = 0.0;
    double best_rho = -1.0;

    // 对三轴分别搜索最佳偏移
    for (const std::string& axis : {"x","y","z"}) {
        // 锁定当前轴的IMU和视觉数据
        const std::vector<double>* imu_s_ptr = nullptr;
        const std::vector<double>* vis_s_ptr = nullptr;
        if (axis == "x") { imu_s_ptr = &imu_x; vis_s_ptr = &vis_x; }
        else if (axis == "y") { imu_s_ptr = &imu_y; vis_s_ptr = &vis_y; }
        else { imu_s_ptr = &imu_z; vis_s_ptr = &vis_z; }

        // 初始化当前轴的最优偏移和系数
        double best_axis_bias = 0.0;
        double best_axis_rho = -1.0;

        // 确定搜索范围和搜索步长，遍历可能的偏移值
        for (double bias = -0.025; bias <= 0.025 + 1e-12; bias += 0.0005) {
            // 将视觉插值到imu时间上，对齐时间
            auto vis_interp = interpolateVisToImu(vis, imu_t, *vis_s_ptr, bias);
            // 计算对齐后IMU与视觉数据的互相关系数
            double rho = computeCorrelation(*imu_s_ptr, vis_interp);
            // 更新当前轴的最优偏移
            if (rho > best_axis_rho) {
                best_axis_rho = rho;
                best_axis_bias = bias;
            }
        }
        ROS_INFO("axis %s best bias = %.3f ms (rho=%.6f)", axis.c_str(), best_axis_bias*1000.0, best_axis_rho);
        // 取三轴中系数最大的偏移更新全局最优偏移
        if (best_axis_rho > best_rho) {
            best_rho = best_axis_rho;
            best_bias = best_axis_bias;
        }
    }

    ROS_INFO("best overall bias = %.3f ms (rho=%.6f)", best_bias*1000.0, best_rho);
    return best_bias;
}

// 图像消息回调函数imageCallback：把整个在线时间戳粗对齐过程分为校准阶段和修正阶段两个阶段。校准阶段收集前2000帧用于估计,修正阶段在校准阶段估计完成后将新到来的每一帧修正并发布
void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // 提取图像时间戳
    double t = msg->header.stamp.toSec();

    // 将ROS图像消息sensor_msgs/Image转为OpenCV的cv::Mat格式
    cv::Mat img;
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

    cv::Mat gray_img;
    if (img.channels() == 3) {  
        // 若为彩色图，转为单通道灰度图
        cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
    } else {  
        // 若已是灰度图，直接复用
        gray_img = img.clone();
    }
    // 保留原始彩色img，新增灰度图副本用于缓存计算，不覆盖原始彩色img
    cv::Mat img_for_calib = gray_img.clone();

    // 加互斥锁保护全局数据
    std::lock_guard<std::mutex> lock(buf_mutex);

    // 第一帧图像到来时候标记为开始收集图像数据，意味着进入校准阶段
    if (!ImageCalib::collected) {
        ImageCalib::collected = true;
        ROS_INFO("First image received - start collecting %d images for bias estimation", ImageCalib::TARGET_IMAGES);
    }

    // 数据收集阶段：只缓存图像不发布
    if (!ImageCalib::ready_to_publish) {
        ImageSample s; s.t = t; s.img = img_for_calib;
        ImageCalib::image_buffer.push_back(s);

        // 检查是否达到目标帧数，当刚好等于TARGET_IMAGES所设置的目标帧数时在另一个线程中触发计算
        if ((int)ImageCalib::image_buffer.size() == ImageCalib::TARGET_IMAGES && !ImageCalib::computing) {
            ImageCalib::computing = true;
            ROS_INFO("Collected %d images - launching bias computation thread", ImageCalib::TARGET_IMAGES);
            std::thread compute_thread([](){
                // 复制用于计算的数据，起到加锁作用，避免原缓存被修改
                std::vector<ImageSample> imgs_copy;
                std::vector<ImuSample> imu_copy;
                {
                    std::lock_guard<std::mutex> lock(buf_mutex);
                    imgs_copy = ImageCalib::image_buffer;  
                    imu_copy = imu_buffer;  
                }

                // 安全检查确保图像数至少有TARGET_IMAGES帧
                if ((int)imgs_copy.size() < ImageCalib::TARGET_IMAGES) {
                    ROS_ERROR("compute thread: unexpected small image buffer");
                    ImageCalib::computing = false;
                    return;
                }

                // 确定计算用的时间范围，使用前TARGET_IMAGES帧的时间范围[t0,t1]
                double t0 = imgs_copy.front().t;
                double t1 = imgs_copy[ImageCalib::TARGET_IMAGES - 1].t;

                // 截取在[t0,t1]时间段的IMU
                std::vector<ImuSample> imu_seg;
                imu_seg.reserve(imu_copy.size());
                for (const auto& s : imu_copy) {
                    if (s.t >= t0 - 1e-9 && s.t <= t1 + 1e-9) imu_seg.push_back(s);
                }

                // 检查IMU样本数是否大于等于于2，因为至少2个才够计算
                if (imu_seg.size() < 2) {
                    ROS_ERROR("Not enough IMU samples in the image interval to estimate bias (needed >=2, got %zu)", imu_seg.size());
                    ImageCalib::computing = false;
                    return;
                }

                // 计算视觉角速度，仅使用前TARGET_IMAGES帧
                std::vector<ImageSample> imgs_for_vis(imgs_copy.begin(), imgs_copy.begin() + ImageCalib::TARGET_IMAGES);
                auto vis = computeVisualAngularVelocity(imgs_for_vis);
                
                // 检查相机样本数是否大于等于于2，因为至少2个才够计算
                if (vis.size() < 2) {
                    ROS_ERROR("Not enough visual angular velocity samples to estimate bias (got %zu)", vis.size());
                    ImageCalib::computing = false;
                    return;
                }

                // 计算最优时间偏移
                double bias = estimateBiasFromSegment(imu_seg, vis);

                // 更新全局状态
                {
                    std::lock_guard<std::mutex> lock(buf_mutex);     // 保存最优偏移
                    ImageCalib::estimated_bias = bias;               // 设置修正阶段的时间偏移
                    ImageCalib::ready_to_publish = true;             // 标记可以开始发布了
                    ImageCalib::computing = false;                   // 标记校准阶段的计算结束
                    ImageCalib::image_buffer.clear();                // 清空image_buffer，因为此处的逻辑是前TARGET_IMAGES帧是只参与计算的，所以发布阶段要把它们清除
                    
                    // 打印最终时间偏移量b
                    ROS_INFO("\n==================================================");
                    ROS_INFO("===  bias calculation completed ===");
                    ROS_INFO("b = %.6f s ( %.3f ms)", ImageCalib::estimated_bias, ImageCalib::estimated_bias * 1000.0);
                    ROS_INFO("b is positive so camera timestamp lags behind the IMU       b is negative so camera timestamp is ahead of IMU");
                    ROS_INFO("==================================================");
                    ROS_INFO("Now entering publish mode; subsequent images will be corrected and published.");
                }
            });
            compute_thread.detach();
        }
        // 不发布在校准阶段到达的图像
        return;
    }

    // 修正阶段：对到达的每帧图像进行时间戳修正并发布
    sensor_msgs::ImagePtr out_msg;
    {
        // 创建sensor_msgs::Image类型的消息并填充
        std_msgs::Header hdr;
        hdr.stamp = ros::Time(t + ImageCalib::estimated_bias);
        // 将OpenCV图像转为ROS消息，编码为bgr8去输出修正后的图像，使用原始彩色img
        cv_bridge::CvImage cv_out(hdr, "bgr8", img);
        out_msg = cv_out.toImageMsg();
    }
    // 发布修正后的图像
    ImageCalib::pub_correct.publish(out_msg);
}

// 雷达惯性时间校准过程
// lidarPolarToCartesian函数，提取雷达扫描点的直角坐标，实现从极坐标到直角坐标的转化
void lidarPolarToCartesian(const sensor_msgs::LaserScan::ConstPtr& msg,
                           std::vector<cv::Point2f>& pts) {
    // 每次调用清空输出容器    
    pts.clear();
    // 未收到雷达数据警告    
    if (msg->ranges.empty()) {
        ROS_WARN_THROTTLE(1.0, "Received empty lidar data");
        return;
    }

    // 角度数据初始化
    double angle = msg->angle_min;
    // 遍历每个角度雷达对应的距离值    
    for (float r : msg->ranges) {
        // 过滤无效点
        if (std::isinf(r) || std::isnan(r) || r < LidarCalib::LIDAR_MIN_RANGE || r > LidarCalib::LIDAR_MAX_RANGE) {
            // 角度自增
            angle += msg->angle_increment;
            continue;
        }
        // 极坐标转直角坐标
        double x = r * cos(angle);
        double y = r * sin(angle);
        // 返回输出容器
        pts.emplace_back(x, y);
        // 角度自增
        angle += msg->angle_increment;
    }

    // 过滤后有效点为0个的情况报警
    if (pts.empty()) {
        ROS_WARN_THROTTLE(1.0, "No valid points left after lidar filtering");
    }
}

// computeLidarMotionFeature函数，计算雷达帧间重心偏移，之后推导角速度，作为雷达的运动特征
std::vector<double> computeLidarMotionFeature(const std::vector<LidarSample>& lidar_seg) {
    std::vector<double> motion;
    // 检验是不是有2个帧了，有了2个帧之后就可以进行帧间重心偏移计算了
    if (lidar_seg.size() < 2) {
        ROS_WARN("Insufficient lidar frames (<2) for motion features");
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

        // 计算前一帧重心，重心的x、y坐标是所有点x、y坐标的均值
        cv::Point2f c1(0, 0);
        for (const auto& p : s1.pts) {
            c1.x += p.x;
            c1.y += p.y;
        }
        c1.x /= s1.pts.size();
        c1.y /= s1.pts.size();

        // 计算后一帧重心
        cv::Point2f c2(0, 0);
        for (const auto& p : s2.pts) {
            c2.x += p.x;
            c2.y += p.y;
        }
        c2.x /= s2.pts.size();
        c2.y /= s2.pts.size();

        // 计算帧间时间差，避免除以0
        double dt = s2.t - s1.t;
        if (dt <= 1e-6) {
            motion.push_back(0.0);
            continue;
        }

        // 计算重心的绝对角度
        double theta1 = atan2(c1.y, c1.x);
        double theta2 = atan2(c2.y, c2.x);
        double delta_theta = theta2 - theta1;
        // 计算角度变化量，归一化约束到[-π, π]，处理跨0度跳变
        delta_theta = atan2(sin(delta_theta), cos(delta_theta));

        // 计算雷达角速度
        double omega = delta_theta / dt;
        motion.push_back(omega);
    }

    return motion;
}

// interpolateImuToLidar函数，将IMU信号插值到雷达时间轴，计算出该雷达时间点上IMU的角速度值，让IMU数据和雷达数据在时间轴上一一对应
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
        // 指定提取x轴的话，提取x轴角速度
        if (axis == "x") imu_s.push_back(s.gx);
        // 指定提取y轴的话，提取y轴角速度
        else if (axis == "y") imu_s.push_back(s.gy);
        // 如果没有指定提取哪一个轴，就默认提取z轴角速度
        else imu_s.push_back(s.gz);
    }

    // 对每个雷达时间戳进行线性插值
    for (double t_lidar : lidar_t) {
        // 找到t_lidar在IMU时间序列中的位置，满足条件imu_t[j] ≤ t_lidar < imu_t[j+1]
        size_t j = 0;
        while (j + 1 < imu_t.size() && imu_t[j+1] < t_lidar) ++j;

        // 边界处理，t_lidar超出IMU时间范围时候用边界值
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
        double t1 = imu_t[j], t2 = imu_t[j+1];                                     // 相邻两个IMU的时间戳
        double v1 = imu_s[j], v2 = imu_s[j+1];                                     // 相邻两个IMU的角速度值
        double alpha = (t2 - t1) > 1e-12 ? (t_lidar - t1) / (t2 - t1) : 0.0;       // 插值系数alpha
        imu_interp.push_back(v1 + alpha * (v2 - v1));                              // 线性插值公式：v=v1+α*(v2 - v1)
    }

    return imu_interp;
}

// estimateLidarImuBias函数，估算雷达与IMU之间的时间偏移
double estimateLidarImuBias(const std::vector<ImuSample>& imu_seg,
                            const std::vector<LidarSample>& lidar_seg) {
    // 校验是否有IMU数据以及雷达帧数是否足够
    if (imu_seg.empty() || lidar_seg.size() < 2) {
        ROS_ERROR("Insufficient IMU or lidar data for offset estimation");
        return 0.0;
    }

    // 提取雷达时间序列
    std::vector<double> lidar_t;
    for (const auto& s : lidar_seg) lidar_t.push_back(s.t);
    
    // 计算帧间重心偏移推导得到的雷达角速度
    std::vector<double> lidar_motion = computeLidarMotionFeature(lidar_seg);
    // 失败报警
    if (lidar_motion.empty()) {
        ROS_ERROR("Failed to compute lidar motion features");
        return 0.0;
    }

    // 提取雷达运动特征对应的时间序列，因为运动特征是帧间参数，因此去掉第一帧的时间戳
    std::vector<double> lidar_t_motion(lidar_t.begin() + 1, lidar_t.end());
    // 校验时间戳数量是否和运动特征数量一样
    if (lidar_t_motion.size() != lidar_motion.size()) {
        ROS_WARN("Mismatched lidar motion and time series length");
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
        for (double bias = LidarCalib::SEARCH_MIN_BIAS; bias <= LidarCalib::SEARCH_MAX_BIAS + 1e-12; bias += LidarCalib::SEARCH_STEP) {
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
        ROS_INFO("Lidar-IMU %s-axis best bias: %.3f ms (rho: %.6f)",
                 axis.c_str(), best_axis_bias*1000.0, best_axis_rho);
        // 更新全局最优偏移
        if (best_axis_rho > best_rho) {
            best_rho = best_axis_rho;
            best_bias = best_axis_bias;
        }
    }

    // 打印全局最佳结果
    ROS_INFO("Lidar-IMU global best bias: %.3f ms (rho: %.6f)",
             best_bias*1000.0, best_rho);
    return best_bias;
}

// 雷达消息回调函数lidarCallback：校准阶段收集雷达帧，修正阶段发布修正后雷达数据
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // 看是不是没有收到消息
    if (!msg) {
        ROS_ERROR("Received empty lidar message");
        return;
    }

    // 转换雷达点坐标，从极坐标到直角坐标，过滤无效点
    std::vector<cv::Point2f> pts;
    lidarPolarToCartesian(msg, pts);
    if (pts.empty()) return;

    // 加锁缓存雷达数据
    std::lock_guard<std::mutex> lock(buf_mutex);
    
    // 存储雷达样本
    LidarSample s;
    s.t = msg->header.stamp.toSec();
    s.pts = pts;
    LidarCalib::lidar_buffer.push_back(s);

    // 第一帧雷达到来时标记开始收集
    if (!LidarCalib::collected) {
        LidarCalib::collected = true;
        ROS_INFO("First lidar frame received - start collecting %d frames for bias estimation", LidarCalib::TARGET_LIDAR_FRAMES);
    }

    // 校准阶段：只缓存数据，不发布
    if (!LidarCalib::ready_to_publish) {
        // 检查是否收集满目标帧数，收集满目标帧数，触发计算线程
        if ((int)LidarCalib::lidar_buffer.size() == LidarCalib::TARGET_LIDAR_FRAMES && !LidarCalib::computing) {
            LidarCalib::computing = true;
            ROS_INFO("Collected %d lidar frames - launching bias computation thread", LidarCalib::TARGET_LIDAR_FRAMES);
            
            // 启动计算线程
            std::thread compute_thread([](){
                // 复制计算用的数据，避免原缓存被修改
                std::vector<LidarSample> lidar_copy;
                std::vector<ImuSample> imu_copy;
                {
                    // 仅在复制数据时加锁，减少阻塞
                    std::lock_guard<std::mutex> lock(buf_mutex);
                    lidar_copy = LidarCalib::lidar_buffer;
                    imu_copy = imu_buffer;
                }

                // 安全检查，看雷达的帧数是否足够
                if ((int)lidar_copy.size() < LidarCalib::TARGET_LIDAR_FRAMES) {
                    ROS_ERROR("compute thread: unexpected small lidar buffer");
                    LidarCalib::computing = false;
                    return;
                }

                // 确定雷达数据的时间范围
                double t0 = lidar_copy.front().t;
                double t1 = lidar_copy.back().t;
                ROS_INFO("Lidar time range: %.6f - %.6f (duration: %.2f s)", t0, t1, t1 - t0);

                // 筛选该时间段内的IMU数据
                std::vector<ImuSample> imu_seg = filterImuInRange(imu_copy, t0, t1);
                // IMU帧数不足报警
                if (imu_seg.size() < 2) {
                    ROS_ERROR("Not enough IMU samples in the lidar interval to estimate bias (needed >=2, got %zu)", imu_seg.size());
                    LidarCalib::computing = false;
                    return;
                }
                ROS_INFO("Filtered %zu IMU samples in lidar time range", imu_seg.size());

                // 估算雷达IMU之间的时间偏移
                double bias = estimateLidarImuBias(imu_seg, lidar_copy);

                // 更新全局状态
                {
                    std::lock_guard<std::mutex> lock(buf_mutex);
                    LidarCalib::estimated_bias = bias;
                    LidarCalib::ready_to_publish = true;
                    LidarCalib::computing = false;
                    LidarCalib::lidar_buffer.clear();
                    
                    // 打印最终偏移结果
                    ROS_INFO("\n==================================================");
                    ROS_INFO("===  Lidar-IMU bias calculation completed ===");
                    ROS_INFO("b = %.6f s ( %.3f ms)", LidarCalib::estimated_bias, LidarCalib::estimated_bias * 1000.0);
                    ROS_INFO("b is positive so lidar timestamp lags behind the IMU       b is negative so lidar timestamp is ahead of IMU");
                    ROS_INFO("==================================================");
                    ROS_INFO("Now entering publish mode; subsequent lidar frames will be corrected and published.");
                }
            });
            compute_thread.detach();
        }
        return;
    }

    // 修正阶段：发布时间戳修正后的雷达
    sensor_msgs::LaserScanPtr out_msg = boost::make_shared<sensor_msgs::LaserScan>(*msg);
    // 雷达时间戳修正
    double lidar_t = msg->header.stamp.toSec();
    out_msg->header.stamp = ros::Time(lidar_t + LidarCalib::estimated_bias);
    // 发布修正后的雷达数据
    LidarCalib::pub_correct.publish(out_msg);
}

// 共享的IMU回调函数
// IMU消息回调函数imuCallback：收到/imu_raw话题消息时触发
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // 将ROS IMU消息转换为自定义的格式，只保留时间戳和角速度
    ImuSample s;
    s.t = msg->header.stamp.toSec();
    s.gx = msg->angular_velocity.x;
    s.gy = msg->angular_velocity.y;
    s.gz = msg->angular_velocity.z;
    // 加互斥锁保护imu_buffer，避免多线程同时读写
    std::lock_guard<std::mutex> lock(buf_mutex);
    imu_buffer.push_back(s);

    // 限制IMU缓存大小，防止内存溢出
    if (imu_buffer.size() > 20000) {
        imu_buffer.erase(imu_buffer.begin(), imu_buffer.end() - 20000);
    }
}

// 加载雷达校准参数，此处的参数值需要根据雷达硬件实际参数去设置
void loadLidarParameters(ros::NodeHandle& nh) {
    nh.param("target_lidar_frames", LidarCalib::TARGET_LIDAR_FRAMES, 600);
    nh.param("lidar_min_range", LidarCalib::LIDAR_MIN_RANGE, 0.15);
    nh.param("lidar_max_range", LidarCalib::LIDAR_MAX_RANGE, 16.0);
    nh.param("search_min_bias", LidarCalib::SEARCH_MIN_BIAS, -0.025);
    nh.param("search_max_bias", LidarCalib::SEARCH_MAX_BIAS, 0.025);
    nh.param("search_step", LidarCalib::SEARCH_STEP, 0.0005);

    // 打印加载的参数
    ROS_INFO("Loaded lidar calibration parameters:");
    ROS_INFO("  - Target lidar frames: %d", LidarCalib::TARGET_LIDAR_FRAMES);
    ROS_INFO("  - Valid range: [%.2f, %.2f]m", LidarCalib::LIDAR_MIN_RANGE, LidarCalib::LIDAR_MAX_RANGE);
    ROS_INFO("  - Search range: [%.3f, %.3f]s (step: %.3fms)",
             LidarCalib::SEARCH_MIN_BIAS, LidarCalib::SEARCH_MAX_BIAS, LidarCalib::SEARCH_STEP*1000);
}

// 主函数：整合图像+雷达校准逻辑
int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "lidar_camera_imu_node");
    // 创建节点句柄，使用私有命名空间
    ros::NodeHandle nh("~");

    // 加载雷达校准参数
    loadLidarParameters(nh);

    // 配置话题名称
    std::string imu_topic, image_topic, lidar_topic;
    std::string image_output_topic, lidar_output_topic;
    
    nh.param("imu_topic", imu_topic, std::string("/imu_raw"));
    nh.param("image_topic", image_topic, std::string("/image_timestamp_raw"));
    nh.param("lidar_topic", lidar_topic, std::string("/scan"));
    nh.param("image_output_topic", image_output_topic, std::string("/image_correct_raw"));
    nh.param("lidar_output_topic", lidar_output_topic, std::string("/scan_correct"));

    // 创建发布者，发布修正后的雷达和相机话题
    ImageCalib::pub_correct = nh.advertise<sensor_msgs::Image>(image_output_topic, 2000);
    LidarCalib::pub_correct = nh.advertise<sensor_msgs::LaserScan>(lidar_output_topic, 1000);

    // 创建订阅者，订阅IMU、相机和雷达话题
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 20000, imuCallback);
    ros::Subscriber sub_img = nh.subscribe(image_topic, 2000, imageCallback);
    ros::Subscriber sub_lidar = nh.subscribe(lidar_topic, 1000, lidarCallback);

    // 打印启动信息
    ROS_INFO("sensor_timestamp_align_node started. Waiting for sensors data...");
    ROS_INFO("  - Subscribed IMU: %s", imu_topic.c_str());
    ROS_INFO("  - Subscribed Image: %s", image_topic.c_str());
    ROS_INFO("  - Subscribed Lidar: %s", lidar_topic.c_str());
    ROS_INFO("  - Publishing corrected Image: %s", image_output_topic.c_str());
    ROS_INFO("  - Publishing corrected Lidar: %s", lidar_output_topic.c_str());

    // 进入ROS消息循环，阻塞式处理回调，直到节点关闭
    ros::spin();
    return 0;
}
