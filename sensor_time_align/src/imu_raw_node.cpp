#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

//MPU6050寄存器地址定义(源于官方数据手册)
#define SMPLRT_DIV      0x19    
#define PWR_MGMT_1      0x6B    
#define CONFIG          0x1A    
#define ACCEL_CONFIG    0x1C    
#define GYRO_CONFIG     0x1B    

//加速度计数据寄存器(源于官方数据手册)
#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40

//陀螺仪数据寄存器(源于官方数据手册)
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48

//硬件与ROS参数配置(源于官方数据手册)
#define MPU6050_ADDR    0x68
#define ROS_NODE_NAME   "imu_raw_node"
#define ROS_IMU_TOPIC   "imu_raw"
#define IMU_FRAME_ID    "imu_link"
#define ROS_PUB_RATE    125     

//物理单位转换参数(源于官方数据手册)
#define ACCEL_SENSITIVITY 16384.0  //加速度计灵敏度(±2g量程下，1g对应16384 LSB)
#define GRAVITY           9.81     //重力加速度(g转换为m/s²的系数)
#define GYRO_SENSITIVITY  131.0    //陀螺仪灵敏度(±250°/s量程下，1°/s对应131 LSB)
#define DEG2RAD           0.0174533 //角度转弧度系数(°/s转换为rad/s)

//工具函数声明
static int mpu6050_init(int i2c_fd, uint8_t dev_addr);  //MPU6050初始化函数
static int i2c_reg_write(int i2c_fd, uint8_t dev_addr, uint8_t reg, uint8_t val);  //I2C写寄存器函数
static int i2c_reg_read(int i2c_fd, uint8_t dev_addr, uint8_t reg, uint8_t *val);  //I2C读寄存器函数
static short sensor_data_read(int i2c_fd, uint8_t dev_addr, uint8_t start_reg);  //读取传感器16位数据函数
static double get_highres_timestamp(void);  //获取高精度时间戳函数

//主函数
int main(int argc, char *argv[])
{
    int i2c_fd = -1;

    //初始化ROS节点，创建发布者
    ros::init(argc, argv, ROS_NODE_NAME);
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(ROS_IMU_TOPIC, 100);
    ros::Rate pub_rate(ROS_PUB_RATE);

    //命令行参数检查,看I2C设备路径是否成功传入
    if (argc < 2) {
        ROS_ERROR("[Parameter Error] Missing I2C device path!");
        ROS_ERROR("Usage: rosrun <your_package_name> %s <I2C_device_path>", argv[0]);
        ROS_ERROR("Example: rosrun mpu6050_ros %s /dev/i2c-6", argv[0]);
        exit(EXIT_FAILURE);
    }

    //打开I2C设备
    i2c_fd = open(argv[1], O_RDWR);
    if (i2c_fd < 0) {
        ROS_FATAL("[I2C Error] Failed to open device %s: %s (Code: %d)", 
                  argv[1], strerror(errno), errno);
        exit(EXIT_FAILURE);
    }
    ROS_INFO("[I2C Info] Opened I2C device: %s", argv[1]);

    //初始化MPU6050
    if (mpu6050_init(i2c_fd, MPU6050_ADDR) != 0) {
        ROS_FATAL("[MPU6050 Error] Initialization failed!");
        close(i2c_fd);
        exit(EXIT_FAILURE);
    }

    //启动日志
    ROS_INFO("=============================================================");
    ROS_INFO("[MPU6050 Node] %s started (125Hz)", ROS_NODE_NAME);
    ROS_INFO("ROS Topic: %s (physical units: m/s², rad/s)", ROS_IMU_TOPIC);
    ROS_INFO("Press Ctrl+C to stop");
    ROS_INFO("=============================================================");

    //主循环
    while (ros::ok()) {
        //原始LSB单位数据变量
        short acc_x_lsb, acc_y_lsb, acc_z_lsb;
        short gyro_x_lsb, gyro_y_lsb, gyro_z_lsb;
        //转换后的物理单位变量
        double acc_x_mps2, acc_y_mps2, acc_z_mps2;
        double gyro_x_rads, gyro_y_rads, gyro_z_rads;
        sensor_msgs::Imu imu_msg;

        //读取原始LSB单位数据变量
        acc_x_lsb = sensor_data_read(i2c_fd, MPU6050_ADDR, ACCEL_XOUT_H);
        acc_y_lsb = sensor_data_read(i2c_fd, MPU6050_ADDR, ACCEL_YOUT_H);
        acc_z_lsb = sensor_data_read(i2c_fd, MPU6050_ADDR, ACCEL_ZOUT_H);
        gyro_x_lsb = sensor_data_read(i2c_fd, MPU6050_ADDR, GYRO_XOUT_H);
        gyro_y_lsb = sensor_data_read(i2c_fd, MPU6050_ADDR, GYRO_YOUT_H);
        gyro_z_lsb = sensor_data_read(i2c_fd, MPU6050_ADDR, GYRO_ZOUT_H);

        //转换为物理单位变量
        acc_x_mps2 = (acc_x_lsb / ACCEL_SENSITIVITY) * GRAVITY;
        acc_y_mps2 = (acc_y_lsb / ACCEL_SENSITIVITY) * GRAVITY;
        acc_z_mps2 = (acc_z_lsb / ACCEL_SENSITIVITY) * GRAVITY;
        gyro_x_rads = (gyro_x_lsb / GYRO_SENSITIVITY) * DEG2RAD;
        gyro_y_rads = (gyro_y_lsb / GYRO_SENSITIVITY) * DEG2RAD;
        gyro_z_rads = (gyro_z_lsb / GYRO_SENSITIVITY) * DEG2RAD;

        //填充ROS消息并且发布
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = IMU_FRAME_ID;

        //角速度填充到IMU消息的angular_velocity字段
        imu_msg.angular_velocity.x = gyro_x_rads;
        imu_msg.angular_velocity.y = gyro_y_rads;
        imu_msg.angular_velocity.z = gyro_z_rads;
        imu_msg.angular_velocity_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};

        //加速度填充到IMU消息的linear_acceleration字段
        imu_msg.linear_acceleration.x = acc_x_mps2;
        imu_msg.linear_acceleration.y = acc_y_mps2;
        imu_msg.linear_acceleration.z = acc_z_mps2;
        //加速度协方差矩阵
        imu_msg.linear_acceleration_covariance = {1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4};
        //因为MPU6050无磁力计所以设置为姿态数据无效
        imu_msg.orientation_covariance[0] = -1.0;
        //发布IMU消息到ROS话题
        imu_pub.publish(imu_msg);

        ros::spinOnce();
        pub_rate.sleep();
    }

    //资源清理退出
    close(i2c_fd);
    ROS_INFO("[Node Terminated] Resources released");
    return EXIT_SUCCESS;
}

//工具函数
//高精度时间戳函数get_highres_timestamp
static double get_highres_timestamp(void)
{
    struct timeval tv;
    if (gettimeofday(&tv, NULL) != 0) {
        ROS_WARN("[Timestamp Warning] Get failed: %s (Code: %d)", strerror(errno), errno);
        return -1.0;
    }
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

//MPU6050初始化函数mpu6050_init
static int mpu6050_init(int i2c_fd, uint8_t dev_addr)
{
    //唤醒设备：PWR_MGMT_1寄存器写0x00
    if (i2c_reg_write(i2c_fd, dev_addr, PWR_MGMT_1, 0x00) != 0) {
        ROS_ERROR("[MPU6050 Error] PWR_MGMT_1 init failed");
        return -1;
    }
    usleep(10000);

    //设置采样率:125Hz=(1kHz/(1+7))
    if (i2c_reg_write(i2c_fd, dev_addr, SMPLRT_DIV, 0x07) != 0) {
        ROS_ERROR("[MPU6050 Error] SMPLRT_DIV init failed");
        return -1;
    }

    //设置低通滤波器：CONFIG=0x03，截止频率42Hz，过滤高频噪声
    if (i2c_reg_write(i2c_fd, dev_addr, CONFIG, 0x03) != 0) {
        ROS_ERROR("[MPU6050 Error] CONFIG init failed");
        return -1;
    }

    //设置加速度计量程：±2g，对应灵敏度16384 LSB/g
    if (i2c_reg_write(i2c_fd, dev_addr, ACCEL_CONFIG, 0x00) != 0) {
        ROS_ERROR("[MPU6050 Error] ACCEL_CONFIG init failed");
        return -1;
    }

    //设置陀螺仪量程：±250°/s，对应灵敏度131 LSB/(°/s)
    if (i2c_reg_write(i2c_fd, dev_addr, GYRO_CONFIG, 0x00) != 0) {
        ROS_ERROR("[MPU6050 Error] GYRO_CONFIG init failed");
        return -1;
    }

    ROS_INFO("[MPU6050 Info] Initialized (125Hz, ±2g, ±250°/s)");
    return 0;
}

//I2C写寄存器函数i2c_reg_write
static int i2c_reg_write(int i2c_fd, uint8_t dev_addr, uint8_t reg, uint8_t val)
{
    uint8_t write_buf[2] = {reg, val};
    //配置I2C为7位地址模式，因为MPU6050支持7位地址
    if (ioctl(i2c_fd, I2C_TENBIT, 0) != 0) {
        ROS_ERROR("[I2C Error] 7-bit mode failed: %s", strerror(errno));
        return -1;
    }
    //设置I2C从设备地址
    if (ioctl(i2c_fd, I2C_SLAVE, dev_addr) != 0) {
        ROS_ERROR("[I2C Error] Set slave 0x%02X failed: %s", dev_addr, strerror(errno));
        return -1;
    }
    //设置I2C通信重试次数为2次，避免临时干扰导致失败
    ioctl(i2c_fd, I2C_RETRIES, 2);
    //将write_buf的2个字节写入I2C设备
    if (write(i2c_fd, write_buf, sizeof(write_buf)) != sizeof(write_buf)) {
        ROS_ERROR("[I2C Error] Write reg 0x%02X failed: %s", reg, strerror(errno));
        return -1;
    }
    return 0;
}

//I2C读寄存器函数i2c_reg_read
static int i2c_reg_read(int i2c_fd, uint8_t dev_addr, uint8_t reg, uint8_t *val)
{
    //设置I2C从设备地址
    if (ioctl(i2c_fd, I2C_SLAVE, dev_addr) != 0) {
        ROS_ERROR("[I2C Error] Read slave 0x%02X failed: %s", dev_addr, strerror(errno));
        return -1;
    }
    //写入要读取的寄存器地址
    if (write(i2c_fd, &reg, 1) != 1) {
        ROS_ERROR("[I2C Error] Write read addr 0x%02X failed: %s", reg, strerror(errno));
        return -1;
    }
    //读取寄存器的值
    if (read(i2c_fd, val, 1) != 1) {
        ROS_ERROR("[I2C Error] Read reg 0x%02X failed: %s", reg, strerror(errno));
        return -1;
    }
    return 0;
}

//传感器数据读取函数sensor_data_read
static short sensor_data_read(int i2c_fd, uint8_t dev_addr, uint8_t start_reg)
{
    uint8_t data_high = 0, data_low = 0;
    //读取高8位数据
    if (i2c_reg_read(i2c_fd, dev_addr, start_reg, &data_high) != 0) {
        ROS_WARN("[Data Warning] Read high reg 0x%02X failed", start_reg);
        return 0;
    }
    //读取低8位数据
    if (i2c_reg_read(i2c_fd, dev_addr, start_reg + 1, &data_low) != 0) {
        ROS_WARN("[Data Warning] Read low reg 0x%02X failed", start_reg + 1);
        return 0;
    }
    //拼接高8位和低8位，得到short类型16位有符号数据
    return (short)((data_high << 8) | data_low);
}

