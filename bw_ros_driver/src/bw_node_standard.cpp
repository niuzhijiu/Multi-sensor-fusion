#include <fcntl.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <termios.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <string>
#include <vector>

#include "bw_ros_driver/bcd_utils.hpp"
#include "bw_ros_driver/serial_port.hpp"
#include "bw_ros_driver/bws_parser.hpp"
#include "bw_ros_driver/f3_parser.hpp"
using bw::DataSample;
using bw::SerialPort;
using bw::BwsParser;

int main(int argc, char** argv) {
  ros::init(argc, argv, "bw_ros_driver");
  ros::NodeHandle nh;        
  ros::NodeHandle pnh("~");  

  //setup parameters
  std::string port = "/dev/ttyUSB0", frame_id = "imu_link",
              imu_topic = "/imu/data";
  std::string mag_unit = "gauss";  
  std::string protocol = "auto";
  int baud = 115200;
  bool debug = true;
  double ori_cov = -1.0, gyr_cov = -1.0, acc_cov = -1.0;  
  pnh.param("port", port, port);
  pnh.param("baud", baud, baud);
  pnh.param("frame_id", frame_id, frame_id);
  pnh.param("topic", imu_topic, imu_topic);
  pnh.param("mag_unit", mag_unit, mag_unit);
  pnh.param("debug", debug, debug);
  pnh.param("cov_orientation", ori_cov, ori_cov);
  pnh.param("cov_angular_velocity", gyr_cov, gyr_cov);
  pnh.param("cov_linear_acceleration", acc_cov, acc_cov);
  pnh.param("protocol", protocol, protocol);

  // open serial port
  bw::SerialPort serial_port(port, baud);
  if (!serial_port.openSerial()) {
    ROS_ERROR("Open %s failed", port.c_str());
    return 1;
  }
  ROS_INFO("Opened %s @ %d, publishing %s", port.c_str(), baud,
           imu_topic.c_str());

  // publishers ： IMU & Mag
  ros::Publisher pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 200);
  ros::Publisher mag_pub =
      nh.advertise<sensor_msgs::MagneticField>("/imu/mag", 200);

  const double DEG2RAD = M_PI / 180.0;
  const double G2MS2 = 9.80665;
  
  bw::BwsParser parser;
  
  ros::Time t0 = ros::Time::now();

  uint64_t last_ok = 0;

  bw::DataSample last_dbg{};
  bool last_dbg_valid = false;

  ros::Rate idle(50);

  while (ros::ok()) {
    uint8_t tmp[1024];
    ssize_t n = serial_port.readSome(tmp, sizeof(tmp));
    if (n < 0 && errno != EAGAIN) {
      if (debug) 
        ROS_WARN_THROTTLE(1.0, "read() error: %d", errno);
    }

    if (n > 0) {
      std::vector<bw::DataSample> out;
      out.reserve(8);
      size_t m = parser.feed(tmp, static_cast<size_t>(n), out);

      for (size_t i = 0; i < m; ++i) {
        const bw::DataSample & s = out[i];

        // publish IMU message
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = frame_id;
        for (int k = 0; k < 9; ++k) {
          imu_msg.orientation_covariance[k] = 0;
          imu_msg.angular_velocity_covariance[k] = 0; 
          imu_msg.linear_acceleration_covariance[k] = 0;
        }
        imu_msg.orientation_covariance[0] = ori_cov;
        imu_msg.angular_velocity_covariance[0] = gyr_cov;
        imu_msg.linear_acceleration_covariance[0] = acc_cov;
        if (s.has_quat) {
          imu_msg.orientation.w = s.q0;
          imu_msg.orientation.x = s.q1;
          imu_msg.orientation.y = s.q2;
          imu_msg.orientation.z = s.q3;
        } else if (s.has_euler) {
          double roll_rad = static_cast<double>(s.R) * DEG2RAD;
          double pitch_rad = static_cast<double>(s.P) * DEG2RAD;
          double yaw_rad = static_cast<double>(s.Y) * DEG2RAD;
          double s_fun[3] = {sin(pitch_rad / 2), sin(roll_rad / 2),
                             sin(yaw_rad / 2)};
          double c_fun[3] = {cos(pitch_rad / 2), cos(roll_rad / 2),
                             cos(yaw_rad / 2)};
          imu_msg.orientation.w =
              c_fun[0] * c_fun[1] * c_fun[2] - s_fun[0] * s_fun[1] * s_fun[2];
          imu_msg.orientation.x =
              s_fun[0] * c_fun[1] * c_fun[2] - c_fun[0] * s_fun[1] * s_fun[2];
          imu_msg.orientation.y =
              s_fun[0] * c_fun[1] * s_fun[2] + c_fun[0] * s_fun[1] * c_fun[2];
          imu_msg.orientation.z =
              c_fun[0] * c_fun[1] * s_fun[2] + s_fun[0] * s_fun[1] * c_fun[2];
          // tf2::Quaternion q;
          // q.setRPY(s.R * DEG2RAD, s.P * DEG2RAD, s.Y * DEG2RAD);
          // imu_msg.orientation.x = q.x();
          // imu_msg.orientation.y = q.y();
          // imu_msg.orientation.z = q.z();
          // imu_msg.orientation.w = q.w();
        } else {
          imu_msg.orientation_covariance[0] = -1;
        }

        // publish angular velocity (rad/s)
        if (s.has_gyro) {
          imu_msg.angular_velocity.x = s.gx_dps * DEG2RAD;
          imu_msg.angular_velocity.y = s.gy_dps * DEG2RAD;
          imu_msg.angular_velocity.z = s.gz_dps * DEG2RAD;
        }
        else {
          imu_msg.angular_velocity_covariance[0] = -1;
        }

        // publish linear acceleration (m/s^2)
        if (s.has_acc) {
          imu_msg.linear_acceleration.x = s.ax_g * G2MS2;
          imu_msg.linear_acceleration.y = s.ay_g * G2MS2;
          imu_msg.linear_acceleration.z = s.az_g * G2MS2;
        } else {
          imu_msg.linear_acceleration_covariance[0] = -1;
        }

        pub.publish(imu_msg);

        // publish mag if available
        if (s.has_mag) {
          double scale = 1.0; 
          if (mag_unit == "gauss")
            scale = 1e-4;
          else if (mag_unit == "uT" || mag_unit == "ut")
            scale = 1e-6;

          sensor_msgs::MagneticField mmsg;
          mmsg.header = imu_msg.header;
          mmsg.magnetic_field.x = s.mx * scale;
          mmsg.magnetic_field.y = s.my * scale;
          mmsg.magnetic_field.z = s.mz * scale;
          mag_pub.publish(mmsg);
        }

        last_dbg = s;
        last_dbg_valid = true;
      }
    }

    if (debug) {
      static double last_print = 0.0;
      const double now = ros::Time::now().toSec();
      const double dt_print = now - last_print;

      // print every 1 second
      if (dt_print >= 1.0) {
        const double dt = (ros::Time::now() - t0).toSec();
        const uint64_t ok_now = parser.ok();
        const uint64_t bad_now = parser.bad();
        const double hz = dt > 0.0 ? (ok_now - last_ok) / dt : 0.0;

        if (last_dbg_valid) {
          // print last received data
          ROS_INFO(
              "AUTO BCD52 %.1f Hz  "
              "P=%+.2f R=%+.2f Y=%+.2f  "
              "G[deg/s]=[%+.2f %+.2f %+.2f]  "
              "A[g]=[%+.2f %+.2f %+.2f]  "
              "M=[%+.5f %+.5f %+.5f]  "
              "q=[%+.6f %+.6f %+.6f %+.6f]  "
              "(ok=%lu bad=%lu)",
              hz, last_dbg.P, last_dbg.R, last_dbg.Y, last_dbg.gx_dps,
              last_dbg.gy_dps, last_dbg.gz_dps, last_dbg.ax_g, last_dbg.ay_g,
              last_dbg.az_g, last_dbg.mx, last_dbg.my, last_dbg.mz, last_dbg.q0,
              last_dbg.q1, last_dbg.q2, last_dbg.q3, (unsigned long)ok_now,
              (unsigned long)bad_now);
        } else {
          ROS_INFO("AUTO BCD52 %.1f Hz  (waiting frames...)  (ok=%lu bad=%lu)",
                   hz, (unsigned long)ok_now, (unsigned long)bad_now);
        }

        last_ok = ok_now;
        t0 = ros::Time::now();
        last_print = now;
      }
    }
  }

  serial_port.closeSerial();
  return 0;
}