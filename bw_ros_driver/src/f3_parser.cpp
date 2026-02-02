#include "bw_ros_driver/f3_parser.hpp"
#include <cstring>  
namespace bw {

// helper: read little-endian float from byte array
static float readFloatLe(const uint8_t* p) {
  uint32_t u =
      static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
      (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
  float f;
  std::memcpy(&f, &u, sizeof(float));
  return f;
}

// calculates CRC8 for given data
uint8_t F3Parser::computeCrc8(const uint8_t* p, size_t len) {
  uint8_t crc = 0x00;         // CRC8_INITVALUE
  const uint8_t poly = 0x07;  // CRC8_POLY

  for (size_t i = 0; i < len; ++i) {
    crc ^= p[i];
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 0x80u) {
        crc = static_cast<uint8_t>((crc << 1u) ^ poly);
      } else {
        crc <<= 1u;
      }
    }
  }
  return crc;
}

F3Parser::F3Parser()
    : state_(WAIT_HEAD),
      buf_(),
      expected_len_(0),
      ok_cnt_(0),
      bad_cnt_(0),
      crc_bad_cnt_(0) {}

void F3Parser::reset() {
  state_ = WAIT_HEAD;
  buf_.clear();
  expected_len_ = 0;
}

size_t F3Parser::feed(const uint8_t* data, size_t n,
                      std::vector<F3Sample>& out) {
  size_t produced = 0;
  for (size_t i = 0; i < n; ++i) {
    uint8_t b = data[i];
    switch (state_) {
      case WAIT_HEAD: {
        if (b == 0xF3) {
          buf_.clear();
          buf_.push_back(b); 
          state_ = READ_HEADER;
        }
        break;
      }

      case READ_HEADER: {
        buf_.push_back(b); 
        if (buf_.size() == 3) {
          // [0]=Head, [1]=CmdID, [2]=DataLen
          uint8_t data_len = buf_[2];
          expected_len_ = static_cast<size_t>(2u + data_len);
          
          if (expected_len_ < 10) {
            ++bad_cnt_;
            state_ = WAIT_HEAD;
            buf_.clear();
          } else {
            state_ = READ_FRAME;
          }
        }
        break;
      }

      case READ_FRAME: {
        buf_.push_back(b);

        if (buf_.size() == expected_len_) {
          const std::vector<uint8_t>& frame = buf_;

          bool frame_ok = false;

          // Head + CmdID + DataLen + Count + CRC8 + Data + CheckSum
          if (frame.size() >= 2 + 1 + 1 + 1 + 56 + 1) {
            // ---- 1) CRC8：from Head to Count ----
            // [0]=Head, [1]=CmdID, [2]=DataLen, [3]=Count, [4]=CRC8
            uint8_t crc_calc = computeCrc8(&frame[0], 4);
            uint8_t crc_recv = frame[4];
            if (crc_calc == crc_recv) {
              // ---- 2) CheckSum：CmdID to last Data ----
              // last_data_index = frame.size() - 2
              uint8_t sum = 0;
              for (size_t k = 1; k < frame.size() - 1; ++k) {
                sum = static_cast<uint8_t>(sum + frame[k]);
              }
              uint8_t checksum_recv = frame.back();
              if (sum == checksum_recv) {
                // pass both CRC8 and CheckSum
                F3Sample sample;
                if (parseFrame(frame, sample)) {
                  ++ok_cnt_;
                  frame_ok = true;
                  out.push_back(sample);
                  ++produced;
                } else {
                  ++bad_cnt_;
                }
              } else {
                ++bad_cnt_;
              }
            } else {
              ++crc_bad_cnt_;
            }
          } else {
            ++bad_cnt_;
          }
          // reset for next frame
          state_ = WAIT_HEAD;
          buf_.clear();
          expected_len_ = 0;
        }
        break;
      }
    }
  }
  return produced;
}

bool F3Parser::parseFrame(const std::vector<uint8_t>& frame, F3Sample& s) {
  if (frame.size() < 2 + 1 + 1 + 1 + 56 + 1) {
    return false;
  }

  // [0] Head(0xF3) | [1] CmdID | [2] DataLen | [3] Count | [4] CRC8
  // [5..60] Data (56 bytes)： XYZ angles, acc, gyro, mag, temp, time
  // [61] CheckSum 
  s.cmd_id = frame[1];
  s.count = frame[3];

  const uint8_t* d = &frame[5];  // Data 起始

  s.pitch_deg = readFloatLe(d + 0);
  s.roll_deg = readFloatLe(d + 4);
  s.yaw_deg = readFloatLe(d + 8);

  s.acc_x_g = readFloatLe(d + 12);
  s.acc_y_g = readFloatLe(d + 16);
  s.acc_z_g = readFloatLe(d + 20);

  s.gyro_x_dps = readFloatLe(d + 24);
  s.gyro_y_dps = readFloatLe(d + 28);
  s.gyro_z_dps = readFloatLe(d + 32);

  s.mag_x = readFloatLe(d + 36);
  s.mag_y = readFloatLe(d + 40);
  s.mag_z = readFloatLe(d + 44);

  s.temperature_deg_c = readFloatLe(d + 48);
  {
    uint32_t tu = static_cast<uint32_t>(d[52]) |
                  (static_cast<uint32_t>(d[53]) << 8) |
                  (static_cast<uint32_t>(d[54]) << 16) |
                  (static_cast<uint32_t>(d[55]) << 24);
    s.time_us = tu;
  }

  s.valid = true;
  return true;
}
} 
