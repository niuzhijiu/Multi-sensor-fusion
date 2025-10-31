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
// ROS-related headers
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

/* MPU6050 register address definitions */
#define SMPLRT_DIV      0x19    // Sample rate divider register
#define PWR_MGMT_1      0x6B    // Power management register 1
#define CONFIG          0x1A    // Configuration register (low-pass filter)
#define ACCEL_CONFIG    0x1C    // Accelerometer configuration register
#define GYRO_CONFIG     0x1B    // Gyroscope configuration register

// Accelerometer data registers
#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40

// Gyroscope data registers
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48

/* Hardware and ROS parameter configuration (125Hz) */
#define MPU6050_ADDR    0x68
#define ROS_NODE_NAME   "imu_raw_node"
#define ROS_IMU_TOPIC   "imu_raw"
#define IMU_FRAME_ID    "imu_link"
#define ROS_PUB_RATE    125     // Publishing frequency: 125Hz

/* Physical unit conversion parameters */
#define ACCEL_SENSITIVITY 16384.0  // LSB/g (±2g range)
#define GRAVITY           9.81     // g to m/s²
#define GYRO_SENSITIVITY  131.0    // LSB/(°/s) (±250°/s range)
#define DEG2RAD           0.0174533 // °/s to rad/s

/* Function declarations */
static int mpu6050_init(int i2c_fd, uint8_t dev_addr);
static int i2c_reg_write(int i2c_fd, uint8_t dev_addr, uint8_t reg, uint8_t val);
static int i2c_reg_read(int i2c_fd, uint8_t dev_addr, uint8_t reg, uint8_t *val);
static short sensor_data_read(int i2c_fd, uint8_t dev_addr, uint8_t start_reg);
static double get_highres_timestamp(void);

int main(int argc, char *argv[])
{
    int i2c_fd = -1;

    /* Initialize ROS node */
    ros::init(argc, argv, ROS_NODE_NAME);
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(ROS_IMU_TOPIC, 100);
    ros::Rate pub_rate(ROS_PUB_RATE);

    /* Check command line arguments */
    if (argc < 2) {
        ROS_ERROR("[Parameter Error] Missing I2C device path!");
        ROS_ERROR("Usage: rosrun <your_package_name> %s <I2C_device_path>", argv[0]);
        ROS_ERROR("Example: rosrun mpu6050_ros %s /dev/i2c-6", argv[0]);
        exit(EXIT_FAILURE);
    }

    /* Open I2C device */
    i2c_fd = open(argv[1], O_RDWR);
    if (i2c_fd < 0) {
        ROS_FATAL("[I2C Error] Failed to open device %s: %s (Code: %d)", 
                  argv[1], strerror(errno), errno);
        exit(EXIT_FAILURE);
    }
    ROS_INFO("[I2C Info] Opened I2C device: %s", argv[1]);

    /* Initialize MPU6050 (125Hz) */
    if (mpu6050_init(i2c_fd, MPU6050_ADDR) != 0) {
        ROS_FATAL("[MPU6050 Error] Initialization failed!");
        close(i2c_fd);
        exit(EXIT_FAILURE);
    }

    /* Startup message */
    ROS_INFO("=============================================================");
    ROS_INFO("[MPU6050 Node] %s started (125Hz)", ROS_NODE_NAME);
    ROS_INFO("ROS Topic: %s (physical units: m/s², rad/s)", ROS_IMU_TOPIC);
    ROS_INFO("Press Ctrl+C to stop");
    ROS_INFO("=============================================================");

    /* Main loop (125Hz) */
    while (ros::ok()) {
        // Raw LSB data
        short acc_x_lsb, acc_y_lsb, acc_z_lsb;
        short gyro_x_lsb, gyro_y_lsb, gyro_z_lsb;
        // Converted physical units
        double acc_x_mps2, acc_y_mps2, acc_z_mps2;
        double gyro_x_rads, gyro_y_rads, gyro_z_rads;
        sensor_msgs::Imu imu_msg;

        /* Read raw LSB data */
        acc_x_lsb = sensor_data_read(i2c_fd, MPU6050_ADDR, ACCEL_XOUT_H);
        acc_y_lsb = sensor_data_read(i2c_fd, MPU6050_ADDR, ACCEL_YOUT_H);
        acc_z_lsb = sensor_data_read(i2c_fd, MPU6050_ADDR, ACCEL_ZOUT_H);
        gyro_x_lsb = sensor_data_read(i2c_fd, MPU6050_ADDR, GYRO_XOUT_H);
        gyro_y_lsb = sensor_data_read(i2c_fd, MPU6050_ADDR, GYRO_YOUT_H);
        gyro_z_lsb = sensor_data_read(i2c_fd, MPU6050_ADDR, GYRO_ZOUT_H);

        /* Convert to physical units */
        acc_x_mps2 = (acc_x_lsb / ACCEL_SENSITIVITY) * GRAVITY;
        acc_y_mps2 = (acc_y_lsb / ACCEL_SENSITIVITY) * GRAVITY;
        acc_z_mps2 = (acc_z_lsb / ACCEL_SENSITIVITY) * GRAVITY;
        gyro_x_rads = (gyro_x_lsb / GYRO_SENSITIVITY) * DEG2RAD;
        gyro_y_rads = (gyro_y_lsb / GYRO_SENSITIVITY) * DEG2RAD;
        gyro_z_rads = (gyro_z_lsb / GYRO_SENSITIVITY) * DEG2RAD;

        /* Populate and publish ROS message */
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = IMU_FRAME_ID;

        // Angular velocity (rad/s)
        imu_msg.angular_velocity.x = gyro_x_rads;
        imu_msg.angular_velocity.y = gyro_y_rads;
        imu_msg.angular_velocity.z = gyro_z_rads;
        imu_msg.angular_velocity_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};

        // Linear acceleration (m/s²)
        imu_msg.linear_acceleration.x = acc_x_mps2;
        imu_msg.linear_acceleration.y = acc_y_mps2;
        imu_msg.linear_acceleration.z = acc_z_mps2;
        imu_msg.linear_acceleration_covariance = {1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4};

        // Orientation data invalid (no magnetometer)
        imu_msg.orientation_covariance[0] = -1.0;

        imu_pub.publish(imu_msg);

        ros::spinOnce();
        pub_rate.sleep();
    }

    /* Cleanup */
    close(i2c_fd);
    ROS_INFO("[Node Terminated] Resources released");
    return EXIT_SUCCESS;
}

/* Helper functions */
static double get_highres_timestamp(void)
{
    struct timeval tv;
    if (gettimeofday(&tv, NULL) != 0) {
        ROS_WARN("[Timestamp Warning] Get failed: %s (Code: %d)", strerror(errno), errno);
        return -1.0;
    }
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

static int mpu6050_init(int i2c_fd, uint8_t dev_addr)
{
    // Wake up device
    if (i2c_reg_write(i2c_fd, dev_addr, PWR_MGMT_1, 0x00) != 0) {
        ROS_ERROR("[MPU6050 Error] PWR_MGMT_1 init failed");
        return -1;
    }
    usleep(10000);

    // Set sampling rate: 125Hz (1kHz/(1+7))
    if (i2c_reg_write(i2c_fd, dev_addr, SMPLRT_DIV, 0x07) != 0) {
        ROS_ERROR("[MPU6050 Error] SMPLRT_DIV init failed");
        return -1;
    }

    // Set low-pass filter: 42Hz
    if (i2c_reg_write(i2c_fd, dev_addr, CONFIG, 0x03) != 0) {
        ROS_ERROR("[MPU6050 Error] CONFIG init failed");
        return -1;
    }

    // Accelerometer range: ±2g
    if (i2c_reg_write(i2c_fd, dev_addr, ACCEL_CONFIG, 0x00) != 0) {
        ROS_ERROR("[MPU6050 Error] ACCEL_CONFIG init failed");
        return -1;
    }

    // Gyroscope range: ±250°/s
    if (i2c_reg_write(i2c_fd, dev_addr, GYRO_CONFIG, 0x00) != 0) {
        ROS_ERROR("[MPU6050 Error] GYRO_CONFIG init failed");
        return -1;
    }

    ROS_INFO("[MPU6050 Info] Initialized (125Hz, ±2g, ±250°/s)");
    return 0;
}

static int i2c_reg_write(int i2c_fd, uint8_t dev_addr, uint8_t reg, uint8_t val)
{
    uint8_t write_buf[2] = {reg, val};
    if (ioctl(i2c_fd, I2C_TENBIT, 0) != 0) {
        ROS_ERROR("[I2C Error] 7-bit mode failed: %s", strerror(errno));
        return -1;
    }
    if (ioctl(i2c_fd, I2C_SLAVE, dev_addr) != 0) {
        ROS_ERROR("[I2C Error] Set slave 0x%02X failed: %s", dev_addr, strerror(errno));
        return -1;
    }
    ioctl(i2c_fd, I2C_RETRIES, 2);
    if (write(i2c_fd, write_buf, sizeof(write_buf)) != sizeof(write_buf)) {
        ROS_ERROR("[I2C Error] Write reg 0x%02X failed: %s", reg, strerror(errno));
        return -1;
    }
    return 0;
}

static int i2c_reg_read(int i2c_fd, uint8_t dev_addr, uint8_t reg, uint8_t *val)
{
    if (ioctl(i2c_fd, I2C_SLAVE, dev_addr) != 0) {
        ROS_ERROR("[I2C Error] Read slave 0x%02X failed: %s", dev_addr, strerror(errno));
        return -1;
    }
    if (write(i2c_fd, &reg, 1) != 1) {
        ROS_ERROR("[I2C Error] Write read addr 0x%02X failed: %s", reg, strerror(errno));
        return -1;
    }
    if (read(i2c_fd, val, 1) != 1) {
        ROS_ERROR("[I2C Error] Read reg 0x%02X failed: %s", reg, strerror(errno));
        return -1;
    }
    return 0;
}

static short sensor_data_read(int i2c_fd, uint8_t dev_addr, uint8_t start_reg)
{
    uint8_t data_high = 0, data_low = 0;
    if (i2c_reg_read(i2c_fd, dev_addr, start_reg, &data_high) != 0) {
        ROS_WARN("[Data Warning] Read high reg 0x%02X failed", start_reg);
        return 0;
    }
    if (i2c_reg_read(i2c_fd, dev_addr, start_reg + 1, &data_low) != 0) {
        ROS_WARN("[Data Warning] Read low reg 0x%02X failed", start_reg + 1);
        return 0;
    }
    return (short)((data_high << 8) | data_low);
}

