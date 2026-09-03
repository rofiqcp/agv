#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

static double wrap(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}
static void check(bool ok, const char * msg) {
  if (!ok) { std::cerr << "FAIL " << msg << "\n"; std::exit(1); }
}
int main() {
  // 1) COG ENU yaw + map rotation must stay normalized.
  const double cog_enu = 170.0 * M_PI / 180.0;
  const double map_rot = 25.0 * M_PI / 180.0;
  const double map_yaw = wrap(cog_enu + map_rot);
  check(std::abs(map_yaw - (-165.0 * M_PI / 180.0)) < 1e-12, "COG map yaw wrap");

  // 2) headAcc covariance clamping: 1deg is raised to 2deg floor; 40deg is capped at 30deg.
  const double min_var = std::pow(2.0*M_PI/180.0, 2);
  const double max_var = std::pow(30.0*M_PI/180.0, 2);
  double v1 = std::clamp(std::pow(1.0*M_PI/180.0,2), min_var, max_var);
  double v2 = std::clamp(std::pow(40.0*M_PI/180.0,2), min_var, max_var);
  check(std::abs(v1-min_var) < 1e-12, "COG variance floor");
  check(std::abs(v2-max_var) < 1e-12, "COG variance cap");

  // 3) Same-time map->odom pairing: a 100ms-old GNSS pose should pair with 100ms-old odom.
  // Vehicle moves +0.03m during 100ms. True map->odom translation is zero.
  const double gnss_x_at_t = 10.0;
  const double odom_x_at_t = 10.0;
  const double odom_x_now = 10.03;
  const double correct_anchor = gnss_x_at_t - odom_x_at_t;
  const double wrong_anchor = gnss_x_at_t - odom_x_now;
  check(std::abs(correct_anchor) < 1e-12, "time-aligned anchor");
  check(std::abs(wrong_anchor + 0.03) < 1e-12, "latency false shift example");

  // 4) Gated velocity covariance clamp.
  const double min_v = 0.0025, max_v = 0.25;
  check(std::abs(std::clamp(0.0001,min_v,max_v)-min_v)<1e-12, "velocity variance floor");
  check(std::abs(std::clamp(1.0,min_v,max_v)-max_v)<1e-12, "velocity variance cap");

  std::cout << "PASS gnss_global_fusion_math_self_check\n";
  return 0;
}
