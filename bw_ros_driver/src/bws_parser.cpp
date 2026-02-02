#include "bw_ros_driver/bws_parser.hpp"
#include <ros/ros.h>
using bw::BwsParser;

BwsParser::BwsParser()
    : st_(WAIT_77), buf_(), need_(0), ok_cnt_(0), bad_cnt_(0) {}

void BwsParser::reset() {
  st_ = WAIT_77;
  buf_.clear();
  need_ = 0;
}

size_t BwsParser::feed(const uint8_t* data, size_t n,
                       std::vector<DataSample>& out) {
  size_t produced = 0;
  for (size_t i = 0; i < n; ++i) {
    uint8_t byte = data[i];
    if (st_ == WAIT_77) {
      if (byte == 0x77) {
        buf_.clear();
        buf_.push_back(byte);
        st_ = READ_LEN;
      }
    } else if (st_ == READ_LEN) {
      buf_.push_back(byte);
      need_ = byte;
      st_ = READ_PAYLOAD;
    } else {
      buf_.push_back(byte);
      // collected all bytes for this frame
      if (buf_.size() == static_cast<size_t>(1 + need_)) {
        uint8_t cs = sum8(&buf_[1], need_ - 1);
        uint8_t chk = buf_.back();
        if (cs == chk) {
          DataSample s;
          if (onFrame(s)) {
            out.push_back(s);
            ++produced;
          }
          ++ok_cnt_;
        } else {
          ++bad_cnt_;
        }
        st_ = WAIT_77;
      }
    }
  }
  return produced;
}

bool BwsParser::onFrame(DataSample& out) {
  if (buf_.size() < 5) 
    return false;
  
  const uint8_t* p = buf_.data();
  if (p[0] != 0x77)
    return false;
  
  uint8_t len = p[1];
  uint8_t cmd = p[3];
  const uint8_t* d = &p[4];
  // Number of bytes in the data field
  size_t data_len = len - 4;  

  // Decode based on CMD
  if (cmd == 0x59 || cmd == 0x60) {
    if (data_len == 52) {
      out.P = bcd3_angle_or_dps(d[0], d[1], d[2]);
      out.R = bcd3_angle_or_dps(d[3], d[4], d[5]);
      out.Y = bcd3_angle_or_dps(d[6], d[7], d[8]);
      out.has_euler = true;

      out.ax_g = bcd3_acc_g(d[9], d[10], d[11]);
      out.ay_g = bcd3_acc_g(d[12], d[13], d[14]);
      out.az_g = bcd3_acc_g(d[15], d[16], d[17]);
      out.has_acc = true;

      out.gx_dps = bcd3_angle_or_dps(d[18], d[19], d[20]);
      out.gy_dps = bcd3_angle_or_dps(d[21], d[22], d[23]);
      out.gz_dps = bcd3_angle_or_dps(d[24], d[25], d[26]);
      out.has_gyro = true;

      out.mx = bcd3_mag_frac5(d[27], d[28], d[29]);
      out.my = bcd3_mag_frac5(d[30], d[31], d[32]);
      out.mz = bcd3_mag_frac5(d[33], d[34], d[35]);
      out.has_mag = true;

      out.q0 = bcd4_q_to_double(d[36], d[37], d[38], d[39]);
      out.q1 = bcd4_q_to_double(d[40], d[41], d[42], d[43]);
      out.q2 = bcd4_q_to_double(d[44], d[45], d[46], d[47]);
      out.q3 = bcd4_q_to_double(d[48], d[49], d[50], d[51]);
      out.has_quat = true;

      return true;
    } else if (data_len == 43) {
      out.P = bcd3_angle_or_dps(d[0], d[1], d[2]);
      out.R = bcd3_angle_or_dps(d[3], d[4], d[5]);
      out.Y = bcd3_angle_or_dps(d[6], d[7], d[8]);
      out.has_euler = true;

      out.ax_g = bcd3_acc_g(d[9], d[10], d[11]);
      out.ay_g = bcd3_acc_g(d[12], d[13], d[14]);
      out.az_g = bcd3_acc_g(d[15], d[16], d[17]);
      out.has_acc = true;

      out.gx_dps = bcd3_angle_or_dps(d[18], d[19], d[20]);
      out.gy_dps = bcd3_angle_or_dps(d[21], d[22], d[23]);
      out.gz_dps = bcd3_angle_or_dps(d[24], d[25], d[26]);
      out.has_gyro = true;

      out.q0 = bcd4_q_to_double(d[27], d[28], d[29], d[30]);
      out.q1 = bcd4_q_to_double(d[31], d[32], d[33], d[34]);
      out.q2 = bcd4_q_to_double(d[35], d[36], d[37], d[38]);
      out.q3 = bcd4_q_to_double(d[39], d[40], d[41], d[42]);
      out.has_quat = true;

      return true;
    }
  }
  return false;
}

