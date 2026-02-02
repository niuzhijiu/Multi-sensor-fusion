/*!
 *  \file         f3_parser.hpp
 *  \author       BW
 *  \date         22/11/2025
 *  \brief        f3 protocol parser of bwsensing devices.
 *
 *  Parser for the f3 protocol of bwsensing devices.
 *
 *  \section CodeCopyright Copyright Notice
 *  MIT License
 *
 *  Copyright (C) 2025, BEWIS SENSING. All rights reserved.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
#pragma once

#include <cstdint>
#include <vector>

namespace bw {

struct F3Sample {
  bool valid = false;
  uint8_t cmd_id = 0;
  uint8_t count = 0;
  float pitch_deg = 0.0f, roll_deg = 0.0f, yaw_deg = 0.0f;
  float acc_x_g = 0.0f, acc_y_g = 0.0f, acc_z_g = 0.0f;
  float gyro_x_dps = 0.0f, gyro_y_dps = 0.0f, gyro_z_dps = 0.0f;
  float mag_x = 0.0f, mag_y = 0.0f, mag_z = 0.0f;
  float temperature_deg_c = 0.0f;
  uint32_t time_us = 0;
};

// Bwsensing f3 protocol parser
//
// Frame format (automatic output mode):
// Head(0xF3) | CmdID | DataLen | Count | CRC8 | Data | CheckSum
class F3Parser {
 private:
  enum State { WAIT_HEAD, READ_HEADER, READ_FRAME };
  State state_ = WAIT_HEAD;
  std::vector<uint8_t> buf_{};
  size_t expected_len_;  
  uint64_t ok_cnt_= 0, bad_cnt_ = 0, crc_bad_cnt_= 0;

  // calculates CRC8 for given data
  static uint8_t computeCrc8(const uint8_t* p, size_t len);

  // parse a complete frame into F3Sample
  bool parseFrame(const std::vector<uint8_t>& frame, F3Sample& sample);

 public:
  F3Parser();
  void reset();

  /*!
   * \brief Feeds raw bytes into the parser and extracts complete samples.
   *
   * This method can be called repeatedly with arbitrary chunks of data
   * from a stream (e.g. partial serial reads). Any complete frames found
   * in the input are decoded into DataSample objects and appended to "out".
   *
   * \param[in] data Pointer to the input data buffer.
   * \param[in] n    Number of bytes available in the input buffer.
   * \param[out] out Vector that receives decoded data samples. New samples
   *                 are appended; existing contents are preserved.
   * \return Number of DataSample objects appended to "out".
   */
  size_t feed(const uint8_t* data, size_t n, std::vector<F3Sample>& out);

  uint64_t ok() const { return ok_cnt_; }
  uint64_t bad() const { return bad_cnt_; }
  uint64_t crcBad() const { return crc_bad_cnt_; }
};
}
