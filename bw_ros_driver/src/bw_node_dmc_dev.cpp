// #include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/MagneticField.h>
// #include <tf2/LinearMath/Quaternion.h>

// #include <atomic>
// #include <cmath>
// #include <mutex>
// #include <string>
// #include <thread>
// #include <vector>

// #include "bw_ros_driver/f3_parser.hpp"
// #include "bw_ros_driver/serial_port.hpp"

// using bw::F3Parser;
// using bw::F3Sample;
// using bw::SerialPort;

// namespace {

// struct Shared
// {
//   std::mutex mtx;
//   F3Sample latest;
//   std::atomic<bool> have{false};
// };

// }  // namespace

// int main(int argc, char** argv) {
//   ros::init(argc, argv, "bw_ros_driver_f3_node");
//   ros::NodeHandle nh;
//   ros::NodeHandle pnh("~");

//   //--- parameters ---//
//   std::string port = "/dev/ttyUSB0";
//   int baud = 115200;
//   std::string frame_id = "imu_link";
//   std::string imu_topic = "/imu/data_f3";
//   std::string mag_topic = "/imu/mag_f3";
//   std::string mag_unit = "gauss";

//   double publish_rate = 500.0;
//   bool debug = true;

//   double cov_orientation = -1.0;
//   double cov_gyr = -1.0;
//   double cov_acc = -1.0;

//   pnh.param("port", port, port);
//   pnh.param("baud", baud, baud);
//   pnh.param("frame_id", frame_id, frame_id);
//   pnh.param("imu_topic", imu_topic, imu_topic);
//   pnh.param("mag_topic", mag_topic, mag_topic);
//   pnh.param("mag_unit", mag_unit, mag_unit);
//   pnh.param("publish_rate", publish_rate, publish_rate);
//   pnh.param("debug", debug, debug);
//   pnh.param("cov_orientation", cov_orientation, cov_orientation);
//   pnh.param("cov_angular_velocity", cov_gyr, cov_gyr);
//   pnh.param("cov_linear_acceleration", cov_acc, cov_acc);

//   //--- open serial port ---//
//   SerialPort sp(port, baud);
//   if (!sp.openSerial()) {
//     ROS_FATAL("Open %s failed", port.c_str());
//     return 1;
//   }
//   ROS_INFO("Opened %s @ %d (F3 protocol), publishing %s & %s", port.c_str(),
//            baud, imu_topic.c_str(), mag_topic.c_str());

//   F3Parser parser;
//   Shared shared;

//   //-------------------------------------------------------------------//
//   //- Reader thread: serial -> F3FloatPaser -> latest sample -//
//   //-------------------------------------------------------------------//

//   std::atomic<bool> stop{false};
//   std::thread reader([&] {
//     uint8_t buf[1024];
//     ros::Time t0 = ros::Time::now();
//     uint64_t last_ok = 0;

//     while (!stop.load() && ros::ok()) {
//       ssize_t n = sp.readSome(buf, sizeof(buf));
//       if (n > 0) {
//         std::vector<F3Sample> outs;
//         outs.reserve(4);
//         size_t m = parser.feed(buf, static_cast<size_t>(n), outs);
//         if (m > 0) {
//           std::lock_guard<std::mutex> lk(shared.mtx);
//           shared.latest = outs.back();
//           shared.have.store(true, std::memory_order_release);
//         }
//       } else if (n == 0) {
//         std::this_thread::sleep_for(std::chrono::milliseconds(1));
//       }

//       if (debug) {
//         double dt = (ros::Time::now() - t0).toSec();
//         if (dt >= 1.0) {
//           uint64_t ok = parser.ok();
//           double hz = dt > 0.0 ? (ok - last_ok) / dt : 0.0;
//           ROS_INFO("F3 RX ~ %.1f Hz (ok=%lu bad=%lu crc_bad=%lu)", hz,
//                    (unsigned long)ok, (unsigned long)parser.bad(),
//                    (unsigned long)parser.crcBad());
//           last_ok = ok;
//           t0 = ros::Time::now();
//         }
//       }
//     }
//   });

//   //-------------------------------------------------------------------//
//   //- Publishers                                                       -//
//   //-------------------------------------------------------------------//

//   ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 50);
//   ros::Publisher mag_pub =
//       nh.advertise<sensor_msgs::MagneticField>(mag_topic, 50);

//   const double DEG2RAD = M_PI / 180.0;
//   const double G2MS2 = 9.80665;

//   //-------------------------------------------------------------------//
//   //- Timer: fixed rate IMU publishing                                 -//
//   //-------------------------------------------------------------------//

//   ros::Timer timer = nh.createTimer(
//       ros::Duration(1.0 / std::max(1.0, publish_rate)),
//       [&](const ros::TimerEvent&) {
//         if (!shared.have.load(std::memory_order_acquire)) {
//           ROS_DEBUG_THROTTLE(2.0, "F3: waiting for first frame...");
//           return;
//         }

//         F3Sample s;
//         {
//           std::lock_guard<std::mutex> lk(shared.mtx);
//           s = shared.latest;
//         }
//         if (!s.valid) {
//           return;
//         }

//         sensor_msgs::Imu msg;
//         msg.header.stamp = ros::Time::now();
//         msg.header.frame_id = frame_id;

//         for (int i = 0; i < 9; ++i) {
//           msg.orientation_covariance[i] = 0.0;
//           msg.angular_velocity_covariance[i] = 0.0;
//           msg.linear_acceleration_covariance[i] = 0.0;
//         }
//         msg.orientation_covariance[0] = cov_orientation;
//         msg.angular_velocity_covariance[0] = cov_gyr;
//         msg.linear_acceleration_covariance[0] = cov_acc;

//         // Orientation: Euler (roll, pitch, yaw) in deg -> quaternion
//         tf2::Quaternion q;
//         double roll_rad = static_cast<double>(s.roll_deg) * DEG2RAD;
//         double pitch_rad = static_cast<double>(s.pitch_deg) * DEG2RAD;
//         double yaw_rad = static_cast<double>(s.yaw_deg) * DEG2RAD;
//         q.setRPY(roll_rad, pitch_rad, yaw_rad);
//         msg.orientation.x = q.x();
//         msg.orientation.y = q.y();
//         msg.orientation.z = q.z();
//         msg.orientation.w = q.w();

//         // Angular velocity: deg/s -> rad/s
//         msg.angular_velocity.x = static_cast<double>(s.gyro_x_dps) * DEG2RAD;
//         msg.angular_velocity.y = static_cast<double>(s.gyro_y_dps) * DEG2RAD;
//         msg.angular_velocity.z = static_cast<double>(s.gyro_z_dps) * DEG2RAD;

//         // Linear acceleration: g -> m/s^2
//         msg.linear_acceleration.x = static_cast<double>(s.acc_x_g) * G2MS2;
//         msg.linear_acceleration.y = static_cast<double>(s.acc_y_g) * G2MS2;
//         msg.linear_acceleration.z = static_cast<double>(s.acc_z_g) * G2MS2;

//         imu_pub.publish(msg);

//         // Magnetic field: gauss / uT -> Tesla
//         double scale = 1.0;  // Tesla
//         if (mag_unit == "gauss") {
//           scale = 1e-4;
//         } else if (mag_unit == "uT" || mag_unit == "ut") {
//           scale = 1e-6;
//         }

//         sensor_msgs::MagneticField m;
//         m.header = msg.header;
//         m.magnetic_field.x = static_cast<double>(s.mag_x) * scale;
//         m.magnetic_field.y = static_cast<double>(s.mag_y) * scale;
//         m.magnetic_field.z = static_cast<double>(s.mag_z) * scale;
//         mag_pub.publish(m);
//       },
//       false);

//   //-------------------------------------------------------------------//
//   //- Spin                                                             -//
//   //-------------------------------------------------------------------//

//   ros::spin();
//   stop.store(true);
//   if (reader.joinable()) {
//     reader.join();
//   }
//   return 0;
// }