#pragma once
#include <cmath>
#include <cstdint>

// 求和校验（8位），对 [长度..数据域] 求和，不含 0x77，不含校验字节本身
inline uint8_t sum8(const uint8_t* p, size_t n) {
  unsigned s = 0;
  for (size_t i = 0; i < n; ++i) s += p[i];
  return static_cast<uint8_t>(s & 0xFF);
}

// 3字节 BCD（角度/角速度）：SXXXYY -> int(3) + frac(2)
inline double bcd3_angle_or_dps(uint8_t b0, uint8_t b1, uint8_t b2) {
  int sign = (b0 >> 4) & 0x1;
  int d2 = (b0 & 0x0F), d1 = (b1 >> 4) & 0x0F, d0 = (b1 & 0x0F);
  int c1 = (b2 >> 4) & 0x0F, c2 = (b2 & 0x0F);
  double v = (d2 * 100 + d1 * 10 + d0) + (c1 * 10 + c2) / 100.0;
  return sign ? -v : v;
}

// 3字节 BCD（加速度 g）：S X.YYYY -> 1 位整数 + 4 位小数
inline double bcd3_acc_g(uint8_t b0, uint8_t b1, uint8_t b2) {
  int sign = (b0 >> 4) & 0x1;
  int I = (b0 & 0x0F);
  int f1 = (b1 >> 4) & 0x0F, f2 = b1 & 0x0F;
  int f3 = (b2 >> 4) & 0x0F, f4 = b2 & 0x0F;
  double v = I + (f1 * 1000 + f2 * 100 + f3 * 10 + f4) / 10000.0;
  return sign ? -v : v;
}

// 3字节 BCD（磁场）：S 0.YYYYY -> 5 位小数（单位由上层解释）
inline double bcd3_mag_frac5(uint8_t b0, uint8_t b1, uint8_t b2) {
  int sign = (b0 >> 4) & 0x1;
  int f1 = (b0 & 0x0F), f2 = (b1 >> 4) & 0x0F, f3 = b1 & 0x0F;
  int f4 = (b2 >> 4) & 0x0F, f5 = b2 & 0x0F;
  double v = (f1 * 10000 + f2 * 1000 + f3 * 100 + f4 * 10 + f5) /
             100000.0;  // 0.f1f2f3f4f5
  return sign ? -v : v;
}

// 4字节 BCD: S X YYYYYY -> double（用于四元数分量）
inline double bcd4_q_to_double(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  int sign = (b0 >> 4) & 0x1;  // 符号
  int X = (b0 & 0x0F);         // 整数部分一位
  int y1 = (b1 >> 4) & 0x0F, y2 = b1 & 0x0F;
  int y3 = (b2 >> 4) & 0x0F, y4 = b2 & 0x0F;
  int y5 = (b3 >> 4) & 0x0F, y6 = b3 & 0x0F;
  double frac =
      (y1 * 100000 + y2 * 10000 + y3 * 1000 + y4 * 100 + y5 * 10 + y6) / 1e6;
  double v = X + frac;
  return sign ? -v : v;
}