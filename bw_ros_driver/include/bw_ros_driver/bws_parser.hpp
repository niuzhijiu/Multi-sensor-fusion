/*!
 *  \file         bws_parser.hpp
 *  \author       BW
 *  \date         15/10/2025
 *  \brief        Standard protocol parser of bwsensing devices.
 *
 *  Parser for the standard protocol of bwsensing devices.
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
#include "bw_ros_driver/bcd_utils.hpp"

namespace bw 
{
// Data sample extracted from one bwsensing protocol frame
//
// The "has_*" flags indicate which fields are valid for this sample.
// A DataSample may contain multiple sensor modalities in a single frame.
struct DataSample
{
  bool has_euler=false, has_acc=false, has_gyro=false, has_mag=false, has_quat=false;
  // Euler angles in degrees (P: pitch, R: roll, Y: yaw).
  double P=0,R=0,Y=0;
  // Linear acceleration in units of g.
  double ax_g=0, ay_g=0, az_g=0;      
  // Angular velocity in units of degrees per second.
  double gx_dps=0, gy_dps=0, gz_dps=0;
  // Magnetic field
  double mx=0, my=0, mz=0;
  // Orientation as a unit quaternion (w, x, y, z).
  double q0=1, q1=0, q2=0, q3=0;      
};

// Bwsensing standard protocol parser
//
// Frame format (automatic output mode):
// 0x77 | LEN | ADDR | CMD | DATA... | CHK 
class BwsParser {
private:
  enum State { WAIT_77, READ_LEN, READ_PAYLOAD };  // Parser state machine states
  State st_ = WAIT_77;                             // Current state
  std::vector<uint8_t> buf_{};                     // Buffer for accumulating frame data
  uint8_t need_ = 0;                               // Number of bytes needed to complete current state
  uint64_t ok_cnt_ = 0, bad_cnt_ = 0;

  // Attempts to decode the current buffer into a DataSample.
  // 
  // This is called internally once a full frame has been accumulated.
  // Returns true and fills "out" on successful decoding; returns false if
  // the frame is invalid (e.g. checksum error or unsupported payload).
  bool onFrame(DataSample& out);

public:
  BwsParser();
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
  size_t feed(const uint8_t* data, size_t n, std::vector<DataSample>& out);

  uint64_t ok()  const { return ok_cnt_; }
  uint64_t bad() const { return bad_cnt_; }
};
}