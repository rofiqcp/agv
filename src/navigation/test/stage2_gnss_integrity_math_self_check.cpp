#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

static void check(bool ok, const char *msg) {
  if (!ok) {
    std::cerr << "FAIL " << msg << "\n";
    std::exit(1);
  }
}

static bool validCov(double xx, double xy, double yy, double min_v, double max_v) {
  if (!std::isfinite(xx) || !std::isfinite(xy) || !std::isfinite(yy)) return false;
  if (xx < min_v || yy < min_v || xx > max_v || yy > max_v) return false;
  return xx * yy - xy * xy >= -1.0e-10;
}

static void rotCov(double c, double s, double xx, double xy, double yy,
                   double &ox, double &oxy, double &oy) {
  ox = c*c*xx - 2.0*c*s*xy + s*s*yy;
  oxy = c*s*(xx-yy) + (c*c-s*s)*xy;
  oy = s*s*xx + 2.0*c*s*xy + c*c*yy;
}

int main() {
  constexpr double min_v=1.0e-6, max_v=1.0;
  check(validCov(0.02, 0.003, 0.03, min_v, max_v), "valid covariance rejected");
  check(!validCov(0.02, 0.04, 0.03, min_v, max_v), "non-PSD covariance accepted");
  check(!validCov(0.0, 0.0, 0.02, min_v, max_v), "zero variance accepted");
  check(!validCov(2.0, 0.0, 0.02, min_v, max_v), "excess variance accepted");

  // Covariance rotation must preserve trace and determinant.
  const double xx=0.02, xy=0.003, yy=0.03;
  const double a=37.0 * M_PI / 180.0;
  double rx=0.0, rxy=0.0, ry=0.0;
  rotCov(std::cos(a), std::sin(a), xx, xy, yy, rx, rxy, ry);
  check(std::abs((rx+ry)-(xx+yy)) < 1e-12, "covariance trace changed on rotation");
  check(std::abs((rx*ry-rxy*rxy)-(xx*yy-xy*xy)) < 1e-12,
        "covariance determinant changed on rotation");

  // Antenna lever arm: antenna 0.165 m in front, +0.5 rad/s CCW.
  // omega x r contributes +0.0825 m/s lateral at antenna, so subtracting it
  // recovers zero lateral velocity at the base origin.
  const double omega=0.5, rx_ant=0.165, ry_ant=0.0;
  const double ant_vx=1.0 - omega*ry_ant;
  const double ant_vy=0.0 + omega*rx_ant;
  const double base_vx=ant_vx + omega*ry_ant;
  const double base_vy=ant_vy - omega*rx_ant;
  check(std::abs(base_vx-1.0) < 1e-12, "lever-arm longitudinal correction");
  check(std::abs(base_vy) < 1e-12, "lever-arm lateral correction");

  // Timestamp acceptance examples for max future=0.1, max age=1.0, regression=0.02.
  const double now=100.0, last=99.95;
  auto stamp_ok=[&](double stamp) {
    const double age=now-stamp;
    if (age < -0.1 || age > 1.0) return false;
    if (last-stamp > 0.02) return false;
    if (std::abs(last-stamp) < 1e-12) return false;
    return true;
  };
  check(stamp_ok(99.97), "valid forward timestamp rejected");
  check(!stamp_ok(100.2), "future timestamp accepted");
  check(!stamp_ok(98.8), "stale timestamp accepted");
  check(!stamp_ok(99.90), "regressing timestamp accepted");
  check(!stamp_ok(last), "duplicate timestamp accepted");

  // Evidence gating: COG can only be active after velocity + COG certifications.
  auto vel_allowed=[](bool require_cert, bool cert){ return !require_cert || cert; };
  auto cog_allowed=[&](bool req_vel, bool vel, bool req_cog, bool cog){
    return (!req_cog || cog) && vel_allowed(req_vel, vel);
  };
  check(!vel_allowed(true,false), "velocity evidence gate bypassed");
  check(vel_allowed(true,true), "valid velocity evidence blocked");
  check(!cog_allowed(true,true,true,false), "COG evidence gate bypassed");
  check(!cog_allowed(true,false,true,true), "COG allowed without velocity evidence");
  check(cog_allowed(true,true,true,true), "fully certified COG blocked");

  std::cout << "PASS stage2_gnss_integrity_math_self_check\n";
  return 0;
}
