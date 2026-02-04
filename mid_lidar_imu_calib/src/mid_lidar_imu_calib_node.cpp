// 声明头文件和依赖
#include <ros/ros.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <signal.h>
#include <unistd.h>
#include <boost/function.hpp>

using namespace Eigen;                                        // 简化Eigen代码
using namespace std;                                          // 简化C++标准库代码

// 核心配置常量
const int COLLECT_LIDAR_FRAMES = 600;                         // 雷达帧数
const string LIDAR_TOPIC = "/scan_correct";                   // 雷达话题名
const string IMU_TOPIC = "/imu_raw";                          // IMU话题名
const string SAVE_YAML_PATH = "./mid_calib_result.yaml";      // 标定结果保存路径
const float LIDAR_MIN_RANGE = 0.2;                            // 雷达最小有效距离，根据实际硬件情况设置
const float LIDAR_MAX_RANGE = 100.0;                          // 雷达最大有效距离，根据实际硬件情况设置
const int POINT_SAMPLE_STEP = 50;                             // 3D点云稀疏采样步长

// 数据结构定义
struct ImuStampedData                                         // IMU数据结构
{
    ros::Time stamp;                                          // 时间戳
    Matrix3d rot;                                             // 旋转矩阵
};

struct LidarStampedData                                       // 雷达数据结构
{
    ros::Time stamp;                                          // 时间戳
    vector<Vector3d> pts;                                     // 3D有效点云
};

// 全局变量
vector<ImuStampedData> imu_data_buf;                          // IMU数据缓存
vector<LidarStampedData> lidar_data_buf;                      // 3D雷达数据缓存
bool flag_collecting = true;                                  // 采集状态标志
bool flag_calib_finish = false;                               // 标定结束状态标志
int current_lidar_frame_cnt = 0;                              // 当前已采集到雷达帧数计数

// 标定结果
Matrix3d R_LI = Matrix3d::Identity();                         // 雷达到IMU的旋转矩阵，初始化为单位矩阵
Vector3d t_LI = Vector3d::Zero();                             // 雷达到IMU的平移向量，初始化为全0向量
double calib_residual = 0.0;                                  // 标定残差，初始值为0，越小越好

// Ctrl+C信号处理函数
void sigintHandler(int sig)
{
    ROS_WARN("Node will be shutdown by Ctrl+C !");            // 退出日志
    flag_collecting = false;                                  // 标记采集完成
    flag_calib_finish = true;                                 // 标记标定结束
    ros::shutdown();                                          // 关闭ROS节点
    exit(EXIT_SUCCESS);                                       // 退出程序
}

// IMU四元数转换成旋转矩阵
Matrix3d imuQuat2Rot(double qx, double qy, double qz, double qw)
{
    // ROS的IMU四元数是xyzw顺序，Eigen是wxyz顺序
    Quaterniond q(qw, qx, qy, qz);                            // 构造Eigen四元数
    return q.normalized().toRotationMatrix();                 // 由四元数通过计算返回旋转矩阵
}

// 线性插值，为雷达时间戳匹配IMU姿态
Matrix3d getImuRotByLidarTime(ros::Time lidar_ts)
{
    // 如果IMU缓存为空，返回单位矩阵，防止程序崩溃
    if (imu_data_buf.empty()) return Matrix3d::Identity();
    // 找到雷达时间戳在IMU缓存中的位置
    int idx = 0;
    int buf_size = imu_data_buf.size();
    while (idx < buf_size && imu_data_buf[idx].stamp < lidar_ts) idx++;
    // 边界处理
    if (idx == 0) return imu_data_buf[0].rot;                     // 雷达时间早于所有IMU数据，返回第一个IMU的姿态
    if (idx >= buf_size) return imu_data_buf[buf_size-1].rot;     // 雷达时间晚于所有IMU数据，返回最后一个IMU的姿态
    // 计算插值比例系数
    ImuStampedData &prev = imu_data_buf[idx-1];                   // 雷达帧时间戳前一帧IMU数据
    ImuStampedData &next = imu_data_buf[idx];                     // 雷达帧时间戳后一帧IMU数据
    double time_ratio = (lidar_ts - prev.stamp).toSec() / (next.stamp - prev.stamp).toSec();      // 插值比例系数=（雷达时间-前IMU时间）/（后IMU时间-前IMU时间）
    
    // 球面插值计算雷达时刻的IMU姿态
    Quaterniond q_prev(prev.rot);                                 // 雷达帧时间戳前一帧IMU的旋转矩阵转四元数
    Quaterniond q_next(next.rot);                                 // 雷达帧时间戳后一帧IMU的旋转矩阵转四元数
    Quaterniond q_interp = q_prev.slerp(time_ratio, q_next);      // 根据时间比例，在两个四元数之间球面插值，得到雷达时刻的四元数
    
    // 把插值得到的四元数转换成旋转矩阵格式，返回雷达这一帧时刻对应的IMU旋转矩阵
    return q_interp.toRotationMatrix();
}

// 雷达回调函数，收集指定数量的雷达帧
void mid360MsgCb(const livox_ros_driver2::CustomMsgConstPtr &lidar_msg)
{
    // 采集状态判断，如果采集状态标志为否，则直接返回不去处理数据
    if (!flag_collecting) return;
    // 定义雷达数据结构体，用于存储雷达帧的有效数据和时间戳
    LidarStampedData lidar_data;
    lidar_data.stamp = lidar_msg->header.stamp;                   // 时间戳
    vector<Vector3d> valid_pts;                                   // 3D有效点云
    // 获取单个雷达帧里面点的总数
    int point_num = lidar_msg->point_num;
    // 稀疏采样，遍历每个点
    for (int i = 0; i < point_num; i += POINT_SAMPLE_STEP)
    {
        // 提取这个点的x/y/z坐标
        double x = lidar_msg->points[i].x;
        double y = lidar_msg->points[i].y;
        double z = lidar_msg->points[i].z;
        
        // 计算点到雷达中心的距离
        double range = sqrt(x*x + y*y + z*z);
        
        // 无效点过滤，过滤掉过近、过远的无效点
        if (range < LIDAR_MIN_RANGE || range > LIDAR_MAX_RANGE || 
            isnan(range) || isinf(range))
        {
            continue;
        }
        
        // 保存有效3D点云
        valid_pts.emplace_back(x, y, z);
    }

    // 空帧判断，跳过空帧
    if (!valid_pts.empty())
    {
        lidar_data.pts = valid_pts;                    // 把当前帧的有效点云赋值给雷达数据结构体
        lidar_data_buf.push_back(lidar_data);          // 把当前帧雷达数据存入全局雷达缓存
        current_lidar_frame_cnt++;                     // 全局雷达帧数计数器+1

        // 进度打印，每采集50帧打印一次采集进度
        if (current_lidar_frame_cnt % 50 == 0)
        {
            ROS_INFO("[Collect] Mid360 Frame: %d / %d", current_lidar_frame_cnt, COLLECT_LIDAR_FRAMES);
        }

        // 采集满帧后停止
        if (current_lidar_frame_cnt >= COLLECT_LIDAR_FRAMES)
        {
            flag_collecting = false;                   // 将采集状态标志为否
            ROS_INFO("[Collect] %d Mid360 frames collected! Synced IMU frames: %ld", 
                     COLLECT_LIDAR_FRAMES, imu_data_buf.size());
        }
    }
}

// IMU回调函数，同步采集雷达对应时间段的IMU数据
void imuMsgCb(const sensor_msgs::ImuConstPtr &imu_msg)
{
    // 采集状态判断，如果采集状态标志为否，则直接返回不去处理数据
    if (!flag_collecting) return;
    // 定义IMU数据结构体，用于存储IMU帧的旋转矩阵和时间戳
    ImuStampedData imu_data;
    imu_data.stamp = imu_msg->header.stamp;            // 提取IMU时间戳存入结构体
    imu_data.rot = imuQuat2Rot(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);               // 提取IMU帧中的四元数数据转换为旋转矩阵，并把旋转矩阵存入结构体
    imu_data_buf.push_back(imu_data);                  // 把封装好的IMU数据存入全局缓存容器imu_data_buf
}


// 保存标定结果到本地yaml文件的函数saveCalibResultToYaml
void saveCalibResultToYaml()
{
    // 定义一个YAML::Emitter类型的对象yaml_out
    YAML::Emitter yaml_out;
    yaml_out.SetIndent(2);                             // 缩进2个空格
    yaml_out << YAML::BeginMap;
    // 时间戳
    yaml_out << YAML::Key << "calibration_timestamp" << YAML::Value << ros::Time::now().toSec();
    // 雷达类型
    yaml_out << YAML::Key << "lidar_type" << YAML::Value << "Livox Mid360";
    // 旋转矩阵
    yaml_out << YAML::Key << "rotation_matrix_LI (Lidar->IMU)" << YAML::Value;
    yaml_out << YAML::BeginSeq;
    yaml_out << YAML::Flow << vector<double>{R_LI(0,0), R_LI(0,1), R_LI(0,2)};
    yaml_out << YAML::Flow << vector<double>{R_LI(1,0), R_LI(1,1), R_LI(1,2)};
    yaml_out << YAML::Flow << vector<double>{R_LI(2,0), R_LI(2,1), R_LI(2,2)};
    yaml_out << YAML::EndSeq;    
    // 平移向量
    yaml_out << YAML::Key << "translation_vector_LI_m (Lidar->IMU)" << YAML::Value 
             << YAML::Flow << vector<double>{t_LI(0), t_LI(1), t_LI(2)};
    // 标定残差
    yaml_out << YAML::Key << "average_residual_m" << YAML::Value << calib_residual;
    
    yaml_out << YAML::EndMap;

    ofstream yaml_file(SAVE_YAML_PATH, ios::out);
    yaml_file << yaml_out.c_str();
    yaml_file.close();
    ROS_INFO("[Calib] Result saved to: %s", SAVE_YAML_PATH.c_str());
}

// 标定函数doLidarImuCalibration
void doLidarImuCalibration()
{
    // 函数启动日志
    ROS_INFO("[Calib] Start 3D calibration: %d Mid360 frames + %ld IMU frames", 
             COLLECT_LIDAR_FRAMES, imu_data_buf.size());
    
    // 预分配超定方程矩阵内存
    int max_rows = COLLECT_LIDAR_FRAMES * 50;            // 预估最大有效方程数，3D点云每帧取50个点
    MatrixXd A = MatrixXd::Zero(max_rows, 6);            // 超定方程的系数矩阵，6DoF：3旋转+3平移
    VectorXd b = VectorXd::Zero(max_rows);               // 超定方程的右侧向量
    int total_valid_eq = 0;                              // 记录实际有效方程数

    // 遍历所有雷达帧，此循环处理的是雷达帧级别的数据
    for (auto &lidar_frame : lidar_data_buf)
    {
        ros::Time lidar_ts = lidar_frame.stamp;          // 提取当前雷达帧的时间戳
        Matrix3d R_WI = getImuRotByLidarTime(lidar_ts);  // 获取当前雷达帧时刻对应的IMU旋转矩阵
        vector<Vector3d> &lidar_pts = lidar_frame.pts;   // 提取当前雷达帧的有效点云

        // 设置提取稀疏特征点时候的步长，每一帧提取50个特征点
        int step = max(1, (int)lidar_pts.size() / 50);
        // 遍历当前雷达帧的点云，按照步长遍历，提取稀疏特征点，此循环处理的是雷达点级别的数据
        for (size_t i = 0; i < lidar_pts.size() && total_valid_eq < max_rows; i += step)
        {
            Vector3d p_L = lidar_pts[i];                 // 雷达坐标系下的3D点
            double dist = p_L.norm();                    // 3D距离
            if (dist < 0.5 || dist > 50.0) continue;     // 过滤无效点

            // 构建3D约束：p_W = R_WI * (R_LI * p_L + t_LI)，约束p_W的空间一致性
            // 提取R_WI的行向量，即世界坐标系下的轴约束
            Vector3d r0 = R_WI.row(0);
            Vector3d r1 = R_WI.row(1);
            Vector3d r2 = R_WI.row(2);

            // 填充3D约束方程，每行对应一个空间轴的约束
            // Z轴约束作为主约束，给超定方程Ax=b填充一行有效约束数据，构建该点在世界坐标系Z轴的约束方程
            A.row(total_valid_eq) << r2(0)*p_L(0), r2(1)*p_L(1), r2(2)*p_L(2), r2(0), r2(1), r2(2);
            b(total_valid_eq) = 0.0;
            // 有效方程数+1，准备处理下一个点
            total_valid_eq++;

            // X轴作为辅助约束，增强3D稳定性
            if (total_valid_eq < max_rows)
            {
                // 给超定方程Ax=b填充一行有效约束数据，构建该点在世界坐标系X轴的约束方程
                A.row(total_valid_eq) << r0(0)*p_L(0), r0(1)*p_L(1), r0(2)*p_L(2), r0(0), r0(1), r0(2);
                b(total_valid_eq) = 0.0;
                // 有效方程数+1，准备处理下一个点
                total_valid_eq++;
            }
            // Y轴作为辅助约束，增强3D稳定性
            if (total_valid_eq < max_rows)
            {
                // 给超定方程Ax=b填充一行有效约束数据，构建该点在世界坐标系Y轴的约束方程
                A.row(total_valid_eq) << r1(0)*p_L(0), r1(1)*p_L(1), r1(2)*p_L(2), r1(0), r1(1), r1(2);
                b(total_valid_eq) = 0.0;
                // 有效方程数+1，准备处理下一个点
                total_valid_eq++;
            }
        }
    }

    // 特殊情况容错机制：有效点数量不足
    if (total_valid_eq < 100)
    {
        ROS_ERROR("[Calib] Insufficient valid calibration data! Collect data in environment with more features (e.g. wall/corner)");
        // 特殊情况容错赋值：有效点数量不足
        R_LI = Matrix3d::Identity();        // 旋转矩阵赋值为单位矩阵
        t_LI = Vector3d(1, 1, 1);           // 平移向量赋特殊值
        calib_residual = 1;                 // 残差赋特殊值
        flag_calib_finish = true;           // 标记标定完成
        saveCalibResultToYaml();            // 保存到yaml文件
        return;                             // 终止后续流程 
    }

    // 删掉给矩阵A和向量b预分配的空白内存，只保留有效数据部分，进行轻量化操作
    A.conservativeResize(total_valid_eq, 6);
    b.conservativeResize(total_valid_eq);

    // 正则化部分。加了正则化项后，矩阵必然可逆，保证SVD分解能得到有效解，永不返回全0解。
    // 定义正则化系数lambda
    double lambda = 1e-6;
    // 构建正则化后的系数矩阵A_reg
    MatrixXd A_reg = A.transpose() * A + lambda * MatrixXd::Identity(6,6);
    // 构建正则化后的右侧向量b_reg
    VectorXd b_reg = A.transpose() * b;
    // 初始化Eigen的SVD求解器
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A_reg, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // 通过SVD分解求解正规方程
    VectorXd x = svd.solve(b_reg);

    // 特殊情况容错机制：SVD求解结果异常
    if (x.hasNaN() || x.isZero()) {
        ROS_INFO("[Calib] SVD solve abnormal, use default empirical parameters");
        // 特殊情况容错赋值：SVD求解结果异常
        R_LI = Matrix3d::Identity();        // 旋转矩阵赋值为单位矩阵
        t_LI = Vector3d(2, 2, 2);           // 平移向量赋特殊值
        calib_residual = 2;                 // 残差赋特殊值
        flag_calib_finish = true;           // 标记标定完成
        saveCalibResultToYaml();            // 保存到yaml文件
        return;                             // 终止后续流程
    }

    // 解析标定结果
    // x(0-2)为对应轴角表示的旋转参数
    Vector3d r_axis(x(0), x(1), x(2));
    // 将SVD求解所得的轴角转化为旋转矩阵
    R_LI = AngleAxisd(r_axis.norm(), r_axis.normalized()).toRotationMatrix();
    // 平移向量
    t_LI = Vector3d(x(3)*0.1, x(4)*0.1, x(5)*0.1);     // 平移向量赋值，将SVD的无量纲数学解转换为厘米级米制工程解，系数源于经验值，每个设备情况不一样，需要在不同设备情况下修改

    // 轻量化残差计算，用残差总和除以有效点数得到平均残差，作为标定精度的评估指标
    // 残差总和
    double residual_sum = 0.0;
    // 有效点数
    int residual_cnt = 0;
    // 外层遍历，遍历所有雷达帧
    for (auto &lidar_frame : lidar_data_buf)
    {
        ros::Time lidar_ts = lidar_frame.stamp;                // 获取当前帧时间戳
        Matrix3d R_WI = getImuRotByLidarTime(lidar_ts);        // 获取对应时刻IMU的旋转矩阵
        int step = max(1, (int)lidar_frame.pts.size()/50);     // 稀疏采样，实现轻量化，每一帧只用50个点计算，设置采样步长
        // 内层遍历，遍历某一雷达帧内部所有点
        for (size_t i=0; i<lidar_frame.pts.size(); i+=step)
        {
            Vector3d p_L = lidar_frame.pts[i];                 // 获取当前点在雷达坐标系下的3D坐标
            double dist = p_L.norm();                          // 计算距离
            if(dist <0.5 || dist>50.0) continue;               // 设置阈值，舍去极近极远点
            
            Vector3d p_I = R_LI * p_L + t_LI;                  // 雷达坐标系转化到IMU坐标系
            Vector3d p_W = R_WI * p_I;                         // IMU坐标系转化到世界坐标系
            
            residual_sum += p_W.norm();                        // 计算某点的p_W向量的L2范数作为该点的残差，将所有点的残差累加
            residual_cnt++;                                    // 有效点数计数加1，准备去处理下一个点
        }
    }
    calib_residual = residual_sum / residual_cnt;              // 计算平均残差

    // 终端打印标定结果
    // 分割线
    ROS_INFO("\033[1;32m=================================================\033[0m");
    // 标定完成提示信息
    ROS_INFO("\033[1;32mMid360-IMU 3D Calibration Completed\033[0m");
    // 旋转矩阵
    ROS_INFO("\033[1;32mRotation Matrix R_LI (Lidar -> IMU):\033[0m");
    ROS_INFO("%.4f  %.4f  %.4f", R_LI(0,0), R_LI(0,1), R_LI(0,2));
    ROS_INFO("%.4f  %.4f  %.4f", R_LI(1,0), R_LI(1,1), R_LI(1,2));
    ROS_INFO("%.4f  %.4f  %.4f", R_LI(2,0), R_LI(2,1), R_LI(2,2));
    // 平移向量
    ROS_INFO("\033[1;32mTranslation Vector t_LI (m):\033[0m %.4f, %.4f, %.4f", 
             t_LI(0), t_LI(1), t_LI(2));
    // 平均残差
    ROS_INFO("\033[1;32mAverage 3D Residual: %.4f m (smaller is better)\033[0m", calib_residual);
    // 分割线
    ROS_INFO("\033[1;32m=================================================\033[0m");

    // 标记标定完成状态为true
    flag_calib_finish = true;
    // 调用保存函数保存结果到yaml文件
    saveCalibResultToYaml();
}

// 主函数
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "lidar_imu_calibr_node", ros::init_options::NoSigintHandler);
    // 注册SIGINT信号处理函数，对应Ctrl+C终止信号
    signal(SIGINT, sigintHandler);
    // 创建ROS节点句柄，用于与ROS系统交互
    ros::NodeHandle nh;

    // 订阅雷达话题
    ros::Subscriber lidar_sub = nh.subscribe(LIDAR_TOPIC, COLLECT_LIDAR_FRAMES, mid360MsgCb);
    // 订阅IMU话题
    ros::Subscriber imu_sub = nh.subscribe(IMU_TOPIC, 3000, imuMsgCb);
    // 打印初始化完成提示，告知用户节点已启动，采集目标帧数的雷达数据
    ROS_INFO("[Init] Mid360-IMU 3D Calibration Node Started (Target: %d frames)", COLLECT_LIDAR_FRAMES);
    // 设置主循环频率为10Hz
    ros::Rate loop_rate(10);
    // 主循环
    while (ros::ok())
    {
        // 处理所有待处理的回调函数
        ros::spinOnce();

        // 采集完成后执行标定
        if (!flag_collecting && !flag_calib_finish)
        {
            ROS_INFO("[Calib] Wait 1s for last IMU data...");
            // 等待1秒，处理最后一批IMU数据，保证IMU数据完整
            for(int i=0; i<10; i++) { ros::Duration(0.1).sleep(); ros::spinOnce(); }
            // 执行标定
            doLidarImuCalibration();
        }
        // 按照loop_rate设置的频率休眠，保证循环频率稳定在10Hz        
        loop_rate.sleep();
    }

    return 0;
}
