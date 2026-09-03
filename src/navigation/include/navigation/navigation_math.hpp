#ifndef NAVIGATION__NAVIGATION_MATH_HPP_
#define NAVIGATION__NAVIGATION_MATH_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace navigation_math
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0;

// Fungsi: Membatasi sudut ke rentang [-pi, pi].
inline double normalizeAngle(double angle)
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

// Fungsi: Mengubah derajat menjadi radian tanpa ketergantungan library tambahan.
inline double deg2rad(double degree)
{
  return degree * kPi / 180.0;
}

// Fungsi: Memeriksa semua komponen pose planar agar tidak ada NaN/Inf masuk kontrol.
inline bool finite3(double a, double b, double c)
{
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}

struct EnuPoint
{
  double east_m{0.0};
  double north_m{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// Fungsi: Proyeksi WGS84 lokal ke ENU. Untuk area kampus <1 km, aproksimasi
// tangent-plane ini lebih ringan dan numeriknya memadai dibanding membuat
// pipeline GeographicLib terpisah hanya untuk translasi lokal.
inline EnuPoint wgs84ToEnu(
  double latitude_deg, double longitude_deg,
  double reference_latitude_deg, double reference_longitude_deg)
{
  const double lat = deg2rad(latitude_deg);
  const double lon = deg2rad(longitude_deg);
  const double ref_lat = deg2rad(reference_latitude_deg);
  const double ref_lon = deg2rad(reference_longitude_deg);
  const double mean_lat = 0.5 * (lat + ref_lat);

  EnuPoint out;
  out.east_m = (lon - ref_lon) * std::cos(mean_lat) * kEarthRadiusM;
  out.north_m = (lat - ref_lat) * kEarthRadiusM;
  return out;
}

// Fungsi: Mengubah ENU geografis menjadi koordinat raster map dengan satu
// rotasi yang sudah dikalibrasi dan satu titik kontrol WGS84<->map.
inline Pose2D enuToMap(
  const EnuPoint & enu, double reference_map_x_m, double reference_map_y_m,
  double map_yaw_from_enu_rad, double yaw_enu_rad)
{
  const double c = std::cos(map_yaw_from_enu_rad);
  const double s = std::sin(map_yaw_from_enu_rad);
  Pose2D out;
  out.x = reference_map_x_m + c * enu.east_m - s * enu.north_m;
  out.y = reference_map_y_m + s * enu.east_m + c * enu.north_m;
  out.yaw = normalizeAngle(yaw_enu_rad + map_yaw_from_enu_rad);
  return out;
}

// Fungsi: Mengoreksi posisi antena GNSS menjadi posisi base_footprint.
inline Pose2D antennaToBase(
  const Pose2D & antenna_pose_map, double antenna_x_m, double antenna_y_m)
{
  const double c = std::cos(antenna_pose_map.yaw);
  const double s = std::sin(antenna_pose_map.yaw);
  Pose2D out = antenna_pose_map;
  out.x -= c * antenna_x_m - s * antenna_y_m;
  out.y -= s * antenna_x_m + c * antenna_y_m;
  return out;
}

// Fungsi: Menghitung T_map_odom dari T_map_base dan T_odom_base sehingga hanya
// satu publisher yang memiliki TF map->odom.
inline Pose2D mapOdomFromBase(const Pose2D & map_base, const Pose2D & odom_base)
{
  const double yaw = normalizeAngle(map_base.yaw - odom_base.yaw);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Pose2D out;
  out.yaw = yaw;
  out.x = map_base.x - (c * odom_base.x - s * odom_base.y);
  out.y = map_base.y - (s * odom_base.x + c * odom_base.y);
  return out;
}

// Fungsi: Menghitung T_map_base dari anchor map->odom dan odometri lokal.
inline Pose2D mapBaseFromOdom(const Pose2D & map_odom, const Pose2D & odom_base)
{
  const double c = std::cos(map_odom.yaw);
  const double s = std::sin(map_odom.yaw);
  Pose2D out;
  out.x = map_odom.x + c * odom_base.x - s * odom_base.y;
  out.y = map_odom.y + s * odom_base.x + c * odom_base.y;
  out.yaw = normalizeAngle(map_odom.yaw + odom_base.yaw);
  return out;
}

// Fungsi: Median scalar kecil untuk menahan outlier GNSS startup tanpa filter
// berproses terus-menerus setelah anchor sudah terbentuk.
inline double median(std::vector<double> values)
{
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  const auto middle = values.begin() + static_cast<long>(values.size() / 2U);
  std::nth_element(values.begin(), middle, values.end());
  double value = *middle;
  if ((values.size() % 2U) == 0U) {
    const auto lower = std::max_element(values.begin(), middle);
    value = 0.5 * (value + *lower);
  }
  return value;
}

// Fungsi: Interpolasi pose anchor dengan interpolasi sudut terpendek.
inline Pose2D blendPose(const Pose2D & current, const Pose2D & target, double alpha)
{
  alpha = std::clamp(alpha, 0.0, 1.0);
  Pose2D out;
  out.x = current.x + alpha * (target.x - current.x);
  out.y = current.y + alpha * (target.y - current.y);
  out.yaw = normalizeAngle(current.yaw + alpha * normalizeAngle(target.yaw - current.yaw));
  return out;
}

// Fungsi: Menghitung jarak Euclidean planar untuk gate map/correction.
inline double distance2D(const Pose2D & a, const Pose2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace navigation_math

#endif  // NAVIGATION__NAVIGATION_MATH_HPP_
