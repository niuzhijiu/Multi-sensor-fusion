/*!
*  \file         serial_port.hpp
*  \author       BW 
*  \date         15/10/2025
*  \brief        Serial port configuration.
*
*  Structure to store serial port configuration.
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
#include <string>
#include <termios.h>

namespace bw
{
// Serial Prot Settings Class
class SerialPort
{
private:
  // Serial port parameters
  std::string port_;
  int baud_;
  int fd_;

  // Configures the termios settings for the current file descriptor.
  // Returns true on success.
  bool configureTermios();
  // Maps an integer baud rate to a termios speed_t constant.
  static speed_t mapBaud(int baud);

public:
  explicit SerialPort(std::string port = "/dev/ttyUSB0", int baud = 9600);
  ~SerialPort();

  bool openSerial();
  void closeSerial();
  bool reOpenSerial();
  bool isOpen() const { return fd_ >= 0; };
  int fd() const { return fd_; };
  bool setBaud(int baud);
  int getBaud() const { return baud_; };
  void setPort(const std::string& p) { port_ = p; }
  const std::string& getPort() const { return port_; }

  // Writes exactly len bytes from data to the serial port.
  //
  // Attempts to write until all bytes are sent or an error 
  // occurs. Returns the number of bytes written, or -1 on error.
  ssize_t writeAll(const uint8_t* data, size_t len);

  // Reads up to max bytes from the serial port into buf.
  ssize_t readSome(uint8_t* buf, size_t max);
};
}
