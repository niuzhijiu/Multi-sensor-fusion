#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cerrno>
#include <cmath>
#include <string>
#include <vector>

#include "bw_ros_driver/f3_parser.hpp"
#include "bw_ros_driver/serial_port.hpp"

using bw::F3Parser;
using bw::F3Sample;
using bw::SerialPort;

int main(int argc, char** argv) {
  ros::init(argc, argv, "bw_ros_driver_f3_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Parameters
  std::string port = "/dev/ttyUSB0";
  int baud = 115200;
  std::string frame_id = "imu_link";
  std::string imu_topic = "/imu/data";
  std::string mag_topic = "/imu/mag";
  std::string mag_unit = "gauss";
  bool debug = true;

  double cov_orientation = -1.0;
  double cov_gyr = -1.0;
  double cov_acc = -1.0;

  pnh.param("port", port, port);
  pnh.param("baud", baud, baud);
  pnh.param("frame_id", frame_id, frame_id);
  pnh.param("imu_topic", imu_topic, imu_topic);
  pnh.param("mag_topic", mag_topic, mag_topic);
  pnh.param("mag_unit", mag_unit, mag_unit);
  pnh.param("debug", debug, debug);
  pnh.param("cov_orientation", cov_orientation, cov_orientation);
  pnh.param("cov_angular_velocity", cov_gyr, cov_gyr);
  pnh.param("cov_linear_acceleration", cov_acc, cov_acc);
  // Open serial port
  SerialPort sp(port, baud);
  if (!sp.openSerial()) {
    ROS_FATAL("Open %s failed", port.c_str());
    return 1;
  }
  ROS_INFO("Opened %s @ %d (F3 protocol), publishing %s & %s", port.c_str(),
           baud, imu_topic.c_str(), mag_topic.c_str());
  F3Parser parser;

  // publishers
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 200);
  ros::Publisher mag_pub =
      nh.advertise<sensor_msgs::MagneticField>(mag_topic, 200);

  const double DEG2RAD = M_PI / 180.0;
  const double G2MS2 = 9.80665;

  //- Single-thread main loop: read -> parse -> publish
  ros::Rate idle(1000.0);

  ros::Time t0 = ros::Time::now();
  uint64_t last_ok = 0;

  while (ros::ok()) {
    uint8_t buf[1024];
    ssize_t n = sp.readSome(buf, sizeof(buf));

    if (n < 0 && errno != EAGAIN) {
      if (debug) {
        ROS_WARN_THROTTLE(1.0, "F3 read() error: %d", errno);
      }
    }

    if (n > 0) {
      std::vector<F3Sample> outs;
      outs.reserve(8);

      size_t m = parser.feed(buf, static_cast<size_t>(n), outs);

      for (size_t i = 0; i < m; ++i) {
        const F3Sample& s = outs[i];
        if (!s.valid) {
          continue;
        }
        //IMU
        sensor_msgs::Imu msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id;

        for (int k = 0; k < 9; ++k) {
          msg.orientation_covariance[k] = 0.0;
          msg.angular_velocity_covariance[k] = 0.0;
          msg.linear_acceleration_covariance[k] = 0.0;
        }
        msg.orientation_covariance[0] = cov_orientation;
        msg.angular_velocity_covariance[0] = cov_gyr;
        msg.linear_acceleration_covariance[0] = cov_acc;

        tf2::Quaternion q;
        double roll_rad = static_cast<double>(s.roll_deg) * DEG2RAD;
        double pitch_rad = static_cast<double>(s.pitch_deg) * DEG2RAD;
        double yaw_rad = static_cast<double>(s.yaw_deg) * DEG2RAD;
        double s_fun[3] = {sin(pitch_rad / 2), sin(roll_rad / 2),
                           sin(yaw_rad / 2)};
        double c_fun[3] = {cos(pitch_rad / 2), cos(roll_rad / 2),
                           cos(yaw_rad / 2)};
        msg.orientation.w = c_fun[0] * c_fun[1] * c_fun[2] - s_fun[0] * s_fun[1] * s_fun[2];
        msg.orientation.x = s_fun[0] * c_fun[1] * c_fun[2] - c_fun[0] * s_fun[1] * s_fun[2];
        msg.orientation.y = s_fun[0] * c_fun[1] * s_fun[2] + c_fun[0] * s_fun[1] * c_fun[2];
        msg.orientation.z = c_fun[0] * c_fun[1] * s_fun[2] + s_fun[0] * s_fun[1] * c_fun[2];

        // Angular Velocity
        msg.angular_velocity.x = static_cast<double>(s.gyro_x_dps) * DEG2RAD;
        msg.angular_velocity.y = static_cast<double>(s.gyro_y_dps) * DEG2RAD;
        msg.angular_velocity.z = static_cast<double>(s.gyro_z_dps) * DEG2RAD;

        msg.linear_acceleration.x = static_cast<double>(s.acc_x_g) * G2MS2;
        msg.linear_acceleration.y = static_cast<double>(s.acc_y_g) * G2MS2;
        msg.linear_acceleration.z = static_cast<double>(s.acc_z_g) * G2MS2;

        imu_pub.publish(msg);

        // Magnetic Field
        double scale = 1.0;  // Tesla
        if (mag_unit == "gauss") {
          scale = 1e-4;
        } else if (mag_unit == "uT" || mag_unit == "ut") {
          scale = 1e-6;
        }

        sensor_msgs::MagneticField mmsg;
        mmsg.header = msg.header;
        mmsg.magnetic_field.x = static_cast<double>(s.mag_x) * scale;
        mmsg.magnetic_field.y = static_cast<double>(s.mag_y) * scale;
        mmsg.magnetic_field.z = static_cast<double>(s.mag_z) * scale;
        mag_pub.publish(mmsg);

        // Debug output
        if (debug) {
          double dt = (ros::Time::now() - t0).toSec();
          if (dt >= 1.0) {
            uint64_t ok = parser.ok();
            double hz = dt > 0.0 ? (ok - last_ok) / dt : 0.0;
            last_ok = ok;
            t0 = ros::Time::now();

            ROS_INFO(
                "F3 RX ~ %.1f Hz (ok=%lu bad=%lu crc_bad=%lu) "
                "Euler[deg]=[P=%+.2f R=%+.2f Y=%+.2f] "
                "G[dps]=[%+.2f %+.2f %+.2f] "
                "A[g]=[%+.3f %+.3f %+.3f] "
                "M=[%+.3f %+.3f %+.3f] "
                "Temp[C]=%.2f",
                hz, (unsigned long)parser.ok(), (unsigned long)parser.bad(),
                (unsigned long)parser.crcBad(), s.pitch_deg, s.roll_deg,
                s.yaw_deg, s.gyro_x_dps, s.gyro_y_dps, s.gyro_z_dps, s.acc_x_g,
                s.acc_y_g, s.acc_z_g, s.mag_x, s.mag_y, s.mag_z, s.temperature_deg_c);
          }
        }
      }
    } else {
      if (debug && parser.ok() == 0) {
        ROS_INFO_THROTTLE(2.0, "F3: waiting for frames on %s ...",
                          port.c_str());
      }
    }

    ros::spinOnce();
    idle.sleep();
  }

  sp.closeSerial();
  return 0;
}