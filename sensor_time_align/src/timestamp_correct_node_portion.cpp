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

//定义数据结构
//ImuSample存储单条IMU样本的时间戳和角速度
struct ImuSample {
    double t;
    double gx, gy, gz;
};

//ImageSample存储单帧图像样本的时间戳和图像信息
struct ImageSample {
    double t;
    cv::Mat img; 
};

//VisSample存储计算出的视觉角速度样本的时间戳和角速度
struct VisSample {
    double t;
    double wx, wy, wz;
};

//定义全局变量
std::vector<ImuSample> imu_buffer;       //缓存收到的IMU样本，全局不断追加
std::vector<ImageSample> image_buffer;   //缓存收到的图像样本，用于前3000帧和计算
std::mutex buf_mutex;

bool collected = false;          //是否已经开始收集状态标记，收到第一帧图像时置true
bool computing = false;          //是否正在计算偏移状态标记，开始计算时间偏移时置true
bool ready_to_publish = false;   //是否完成偏移计算状态标记，偏移计算完成后置true
double estimated_bias = 0.0;     //估计出的偏移

ros::Publisher pub_correct;      //ROS发布者：发布修正后图像

const int TARGET_IMAGES = 3000;  //校准阶段需要用于估计的图像帧数：此处设置为3000，也可以进行更改设置成其它数值

//to_s：将浮点数v转为保留6位小数的字符串
static inline std::string to_s(double v) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << v;
    return oss.str();
}

//computeVisualAngularVelocity：计算视觉角速度
std::vector<VisSample> computeVisualAngularVelocity(const std::vector<ImageSample>& imgs) {
    std::vector<VisSample> vis;
    if (imgs.size() < 2) return vis;

    //相机内参矩阵
    cv::Mat K = (cv::Mat_<double>(3,3) <<
        685.0, 0.0, 320.0,
        0.0, 685.0, 240.0,
        0.0, 0.0, 1.0);

    //创建ORB特征提取器，提取图像特征点和描述子，此处设置它最多可以提取2000个特征点,也可以改成别的数值
    cv::Ptr<cv::ORB> orb = cv::ORB::create(2000);

    //遍历相邻图像帧，计算角速度
    for (size_t i = 0; i + 1 < imgs.size(); ++i) {
        //计算帧间时间差
        double t1 = imgs[i].t;
        double t2 = imgs[i+1].t;
        double dt = t2 - t1;
        if (dt <= 0.0) continue;

        //对相邻两帧图像提取ORB特征
        std::vector<cv::KeyPoint> kp1, kp2;  //存储特征点
        cv::Mat des1, des2;  //存储特征描述子
        orb->detectAndCompute(imgs[i].img, cv::Mat(), kp1, des1);  //前一帧特征提取
        orb->detectAndCompute(imgs[i+1].img, cv::Mat(), kp2, des2);  //后一帧特征提取
        if (des1.empty() || des2.empty()) continue;

        //使用bf即Brute Force Matcher暴力匹配器进行特征匹配
        cv::BFMatcher bf(cv::NORM_HAMMING, true);  //基于汉明距离
        std::vector<cv::DMatch> matches;
        bf.match(des1, des2, matches);  //进行匹配
        if (matches.size() < 5) continue;  //匹配点个数少于5的话不予匹配直接跳过

        //提取匹配的特征点坐标
        std::vector<cv::Point2f> pts1, pts2;
        pts1.reserve(matches.size());
        pts2.reserve(matches.size());
        for (auto &m : matches) {
            pts1.push_back(kp1[m.queryIdx].pt);
            pts2.push_back(kp2[m.trainIdx].pt);
        }

        //估计本质矩阵E
        cv::Mat mask;  //掩码标记有效的内点和无效的外点
        cv::Mat E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999, 1.0, mask);  //估计本质矩阵E
        if (E.empty()) continue;  //跳过估计失败的情况
        if (cv::countNonZero(mask) < 5) continue;  //跳过匹配的有效点太少的情况

        //由本质矩阵E得到旋转矩阵R
        cv::Mat R, tvec;
        cv::recoverPose(E, pts1, pts2, K, R, tvec, mask);
        
        //将旋转矩阵R转为旋转向量rvec，使用Rodrigues变化
        cv::Mat rvec;
        cv::Rodrigues(R, rvec);
        if (rvec.rows != 3 || rvec.cols != 1) continue;

        //计算视觉角速度=旋转向量/帧间时间差dt
        cv::Vec3d rotvec(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0));
        cv::Vec3d omega = rotvec / dt;

        //存储视觉角速度，时间戳取两帧中点，代表该角速度的有效时间
        VisSample v;
        v.t = 0.5 * (t1 + t2);
        v.wx = omega[0];
        v.wy = omega[1];
        v.wz = omega[2];
        vis.push_back(v);
    }

    return vis;
}

//filterImuInRange：过滤出指定时间范围内的IMU样本。从所有IMU缓存中，筛选出时间戳在[t0, t1]范围内的样本，用于与该时间段的图像数据对应
std::vector<ImuSample> filterImuInRange(const std::vector<ImuSample>& imu_all, double t0, double t1) {
    std::vector<ImuSample> out;
    out.reserve(imu_all.size());
    for (const auto& s : imu_all) {
        if (s.t >= t0 - 1e-9 && s.t <= t1 + 1e-9) out.push_back(s);
    }
    return out;
}

//interpolateVisToImu：插值视觉信号到imu信号的时间序列
std::vector<double> interpolateVisToImu(const std::vector<VisSample>& vis,
                                        const std::vector<double>& imu_t,
                                        const std::vector<double>& vis_s,
                                        double bias) {
    //存储插值结果的容器
    std::vector<double> vis_interp;
    //预分配内存容量=IMU时间戳数量,提高效率
    vis_interp.reserve(imu_t.size());
    //边界处理：视觉为空时候用0填充结果
    if (vis.empty()) {
        vis_interp.assign(imu_t.size(), 0.0);
        return vis_interp;
    }
    //遍历每个IMU时间戳t
    for (double t : imu_t) {
        //根据t_cam+t_bias=t_imu逻辑把imu先和视觉统一到视觉时间尺度
        double shifted_t = t - bias;
        //用于定位shifted_t在视觉样本中的位置
        size_t j = 0;
        //循环找到shifted_t所在的视觉时间区间
        while (j + 1 < vis.size() && vis[j+1].t < shifted_t) ++j;
        //边界处理：如果shifted_t超过最后一个视觉样本的时间，则用最后一个视觉值填充
        if (j + 1 >= vis.size()) {
            vis_interp.push_back(vis_s.back());
            continue;
        }
        //提取当前视觉区间的时间[t1, t2]
        double t1 = vis[j].t, t2 = vis[j+1].t;
        //提取当前视觉区间的值[v1, v2]
        double v1 = vis_s[j], v2 = vis_s[j+1];
        //定义插值系数alpha
        double alpha = 0.0;
        //计算插值系数：若时间差不为0，alpha=(当前时间-起始时间)/区间总时间
        if (t2 - t1 > 1e-12) alpha = (shifted_t - t1) / (t2 - t1);
        //线性插值v1+alpha*(v2-v1)，得到shifted_t对应的视觉角速度
        vis_interp.push_back(v1 + alpha * (v2 - v1));
    }
    return vis_interp;
}

//computeCorrelation：计算IMU角速度与插值后的视觉角速度的归一化互相关系数IMU角速度与视觉角速度的归一化互相关
double computeCorrelation(const std::vector<double>& imu_s, const std::vector<double>& vis_s) {
    if (imu_s.size() != vis_s.size() || imu_s.empty()) return 0.0;
    //计算两个信号的均值，之后可以去均值防止直流分量的影响
    double mean_imu = 0.0, mean_vis = 0.0;
    for (size_t i = 0; i < imu_s.size(); ++i) { mean_imu += imu_s[i]; mean_vis += vis_s[i]; }
    mean_imu /= imu_s.size(); mean_vis /= vis_s.size();
    //计算互相关公式的分子和分母
    double num = 0.0, denom1 = 0.0, denom2 = 0.0;
    for (size_t i = 0; i < imu_s.size(); ++i) {
        double a = imu_s[i] - mean_imu;  //IMU信号去均值
        double b = vis_s[i] - mean_vis;  //视觉信号去均值
        num += a*b; denom1 += a*a; denom2 += b*b;  //计算互相关公式的分子和分母
    }
    //避免信号无波动时分母为0，返回0表示无相关
    if (denom1 <= 0.0 || denom2 <= 0.0) return 0.0;
    //计算归一化互相关系数
    return num / std::sqrt(denom1 * denom2);
}

//estimateBiasFromSegment：给定同一时间段内IMU与Vis数据，遍历搜索bias，取系数最大的偏移作为最终结果，进行时间偏移量的估算
double estimateBiasFromSegment(const std::vector<ImuSample>& imu_seg, const std::vector<VisSample>& vis) {
    if (imu_seg.empty() || vis.empty()) return 0.0;

    //提取IMU段的时间序列和三轴角速度
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

    //提取视觉三轴角速度
    std::vector<double> vis_x, vis_y, vis_z;
    vis_x.reserve(vis.size()); vis_y.reserve(vis.size()); vis_z.reserve(vis.size());
    for (const auto& v : vis) { vis_x.push_back(v.wx); vis_y.push_back(v.wy); vis_z.push_back(v.wz); }

    //初始化最优偏移和最优互相关系数
    double best_bias = 0.0;
    double best_rho = -1.0;

    //对三轴分别搜索最佳偏移
    for (const std::string& axis : {"x","y","z"}) {
        //锁定当前轴的IMU和视觉数据
        const std::vector<double>* imu_s_ptr = nullptr;
        const std::vector<double>* vis_s_ptr = nullptr;
        if (axis == "x") { imu_s_ptr = &imu_x; vis_s_ptr = &vis_x; }
        else if (axis == "y") { imu_s_ptr = &imu_y; vis_s_ptr = &vis_y; }
        else { imu_s_ptr = &imu_z; vis_s_ptr = &vis_z; }

        //初始化当前轴的最优偏移和系数
        double best_axis_bias = 0.0;
        double best_axis_rho = -1.0;

        //确定搜索范围和搜索步长，遍历可能的偏移值
        for (double bias = -0.025; bias <= 0.025 + 1e-12; bias += 0.001) {
            //将视觉插值到imu时间上，对齐时间
            auto vis_interp = interpolateVisToImu(vis, imu_t, *vis_s_ptr, bias);
            //计算对齐后IMU与视觉数据的互相关系数
            double rho = computeCorrelation(*imu_s_ptr, vis_interp);
            //更新当前轴的最优偏移
            if (rho > best_axis_rho) {
                best_axis_rho = rho;
                best_axis_bias = bias;
            }
        }
        ROS_INFO("axis %s best bias = %.3f ms (rho=%.6f)", axis.c_str(), best_axis_bias*1000.0, best_axis_rho);
        //取三轴中系数最大的偏移更新全局最优偏移
        if (best_axis_rho > best_rho) {
            best_rho = best_axis_rho;
            best_bias = best_axis_bias;
        }
    }

    ROS_INFO("best overall bias = %.3f ms (rho=%.6f)", best_bias*1000.0, best_rho);
    return best_bias;
}

//IMU消息回调函数imuCallback：收到/imu_raw话题消息时触发
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    //将ROS IMU消息转换为自定义的格式，只保留时间戳和角速度
    ImuSample s;
    s.t = msg->header.stamp.toSec();
    s.gx = msg->angular_velocity.x;
    s.gy = msg->angular_velocity.y;
    s.gz = msg->angular_velocity.z;
    //加互斥锁保护imu_buffer，避免多线程同时读写
    std::lock_guard<std::mutex> lock(buf_mutex);
    imu_buffer.push_back(s);
}

//图像消息回调函数imageCallback：把整个在线时间戳粗对齐过程分为校准阶段和修正阶段两个阶段。校准阶段收集前3000帧用于估计,修正阶段在校准阶段估计完成后将新到来的每一帧修正并发布
void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    //提取图像时间戳
    double t = msg->header.stamp.toSec();

    //将ROS图像消息sensor_msgs/Image转为OpenCV的cv::Mat格式
    cv::Mat img;
    try {
        //尝试按bgr8编码转换，bgr8常为彩色图像
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        img = cv_ptr->image.clone();
    } catch (cv_bridge::Exception& e) {
        try {
            //不是bgr8编码的话尝试按mono8编码转换，mono8常为灰度图像
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "mono8");
            cv::cvtColor(cv_ptr->image, img, cv::COLOR_GRAY2BGR);
        } catch (...) {
            //两种编码都失败的话打印警告并跳过该帧
            ROS_WARN("imageCallback: unsupported image encoding, skipping this image");
            return;
        }
    }

    //加互斥锁保护全局数据
    std::lock_guard<std::mutex> lock(buf_mutex);

    //第一帧图像到来时候标记为开始收集图像数据，意味着进入校准阶段
    if (!collected) {
        collected = true;
        ROS_INFO("First image received - start collecting %d images for bias estimation", TARGET_IMAGES);
    }

    //数据收集阶段只缓存图像，不发布
    if (!ready_to_publish) {
        ImageSample s; s.t = t; s.img = img;
        image_buffer.push_back(s);

        //检查是否达到目标帧数，当刚好等于TARGET_IMAGES所设置的目标帧数时在另一个线程中触发计算
        if ((int)image_buffer.size() == TARGET_IMAGES && !computing) {
            computing = true;
            ROS_INFO("Collected %d images - launching bias computation thread", TARGET_IMAGES);
            std::thread compute_thread([](){
                //复制用于计算的数据，起到加锁作用，避免原缓存被修改
                std::vector<ImageSample> imgs_copy;
                std::vector<ImuSample> imu_copy;
                {
                    std::lock_guard<std::mutex> lock(buf_mutex);
                    imgs_copy = image_buffer;  
                    imu_copy = imu_buffer;  
                }

                //安全检查确保图像数至少有TARGET_IMAGES帧，此处为3000帧
                if ((int)imgs_copy.size() < TARGET_IMAGES) {
                    ROS_ERROR("compute thread: unexpected small image buffer");
                    computing = false;
                    return;
                }

                //确定计算用的时间范围，使用前TARGET_IMAGES帧的时间范围[t0,t1]
                double t0 = imgs_copy.front().t;
                double t1 = imgs_copy[TARGET_IMAGES - 1].t;

                //截取在[t0,t1]时间段的IMU
                std::vector<ImuSample> imu_seg;
                imu_seg.reserve(imu_copy.size());
                for (const auto& s : imu_copy) {
                    if (s.t >= t0 - 1e-9 && s.t <= t1 + 1e-9) imu_seg.push_back(s);
                }

                //检查IMU样本数是否大于等于于2，因为至少2个才够计算
                if (imu_seg.size() < 2) {
                    ROS_ERROR("Not enough IMU samples in the image interval to estimate bias (needed >=2, got %zu)", imu_seg.size());
                    computing = false;
                    return;
                }

                //计算视觉角速度，仅使用前TARGET_IMAGES帧
                std::vector<ImageSample> imgs_for_vis(imgs_copy.begin(), imgs_copy.begin() + TARGET_IMAGES);
                auto vis = computeVisualAngularVelocity(imgs_for_vis);
                
                //检查相机样本数是否大于等于于2，因为至少2个才够计算
                if (vis.size() < 2) {
                    ROS_ERROR("Not enough visual angular velocity samples to estimate bias (got %zu)", vis.size());
                    computing = false;
                    return;
                }

                //计算最优时间偏移
                double bias = estimateBiasFromSegment(imu_seg, vis);

                //更新全局状态
                {
                    std::lock_guard<std::mutex> lock(buf_mutex);  //保存最优偏移
                    estimated_bias = bias;  //设置修正阶段的时间偏移
                    ready_to_publish = true;  //标记可以开始发布了
                    computing = false;  //标记校准阶段的计算结束
                    image_buffer.clear();  //清空image_buffer，因为此处的逻辑是前TARGET_IMAGES帧是只参与计算的，所以发布阶段要把它们清除
                    
                    //打印最终时间偏移量b
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
        //不发布在校准阶段到达的图像
        return;
    }

    //修正阶段：对到达的每帧图像进行时间戳修正并发布
    sensor_msgs::ImagePtr out_msg;
    {
        //创建sensor_msgs::Image类型的消息并填充
        std_msgs::Header hdr;
        hdr.stamp = ros::Time(t + estimated_bias);
        //将OpenCV图像转为ROS消息，相机是彩色的所以编码为bgr8
        cv_bridge::CvImage cv_out(hdr, "bgr8", img);
        out_msg = cv_out.toImageMsg();
    }
    //发布修正后的图像
    pub_correct.publish(out_msg);
}

//主函数
int main(int argc, char** argv) {
    //初始化ROS节点
    ros::init(argc, argv, "timestamp_correct_node_portion");
    //创建节点句柄
    ros::NodeHandle nh;
    //订阅
    ros::Subscriber sub_imu = nh.subscribe("/imu_raw", 20000, imuCallback);
    ros::Subscriber sub_img = nh.subscribe("/image_timestamp_raw", 2000, imageCallback);
    //发布
    pub_correct = nh.advertise<sensor_msgs::Image>("/image_correct_raw", 2000);
    ROS_INFO("timestamp_correct_node_portion started. Waiting for images and imu...");
    ros::spin();
    return 0;
}

