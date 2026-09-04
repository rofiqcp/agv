#include <cassert>
#include <cmath>
#include <vector>

#include "perception/perception_safety_core.hpp"

int main()
{
  using namespace perception::safety;

  // Obstacle recenter gate: object kecil/noise tidak boleh memblokir.
  ObstacleGateConfig config;
  config.minimum_obstacle_width_m = 0.50;
  const std::vector<ObstacleMetric> small{{1, 1.0, 0.0, 0.20}};
  assert(!obstacleBlocksRecenter(0.0, small, config).blocked);

  // Object valid yang memotong swept lateral corridor harus memblokir recenter.
  const std::vector<ObstacleMetric> valid{{2, 1.0, 0.0, 0.50}};
  assert(obstacleBlocksRecenter(0.0, valid, config).blocked);

  // Object di luar forward gate tidak boleh memblokir recenter.
  const std::vector<ObstacleMetric> far{{3, 4.0, 0.0, 0.80}};
  assert(!obstacleBlocksRecenter(0.0, far, config).blocked);

  LaneThresholds thresholds;

  // Di area aman, Nav2 tetap NORMAL.
  const auto normal = classifyLaneState(true, 1.5, 1.5, 0.0, NORMAL, thresholds);
  assert(normal.state == NORMAL);
  assert(!normal.critical);

  // Mepet kanan berarti harus geser kiri; 0.35 m juga masuk critical <= 0.40 m.
  const auto near_right = classifyLaneState(true, 1.6, 0.35, 0.30, NORMAL, thresholds);
  assert(near_right.state == RECENTER_LEFT);
  assert(near_right.critical);

  // Mepet kiri berarti harus geser kanan.
  const auto near_left = classifyLaneState(true, 0.35, 1.6, -0.30, NORMAL, thresholds);
  assert(near_left.state == RECENTER_RIGHT);
  assert(near_left.critical);

  // Geometry invalid tetap ditandai lane lost, bukan menghasilkan steering liar.
  const auto lost = classifyLaneState(false, 0.0, 0.0, 0.0, NORMAL, thresholds);
  assert(lost.state == LANE_LOST);

  PixelCorridorConfig pixel;
  // Jauh dari safety line: aman dan tidak ada koreksi.
  const auto pixel_safe = updatePixelCorridorSide(true, 60.0, false, pixel);
  assert(pixel_safe.status == "GREEN" && !pixel_safe.latched);
  assert(std::abs(pixel_safe.correction_m) < 1.0e-12);

  // Mendekati garis hanya warning; belum boleh mengubah steering.
  const auto pixel_warning = updatePixelCorridorSide(true, 20.0, false, pixel);
  assert(pixel_warning.status == "YELLOW" && !pixel_warning.latched);
  assert(std::abs(pixel_warning.correction_m) < 1.0e-12);

  // Tepat menyentuh safety line harus langsung latch dan menghasilkan koreksi > 0.
  const auto pixel_touch = updatePixelCorridorSide(true, pixel.touch_gap_px, false, pixel);
  assert(pixel_touch.status == "RED" && pixel_touch.latched);
  assert(pixel_touch.correction_m > 0.0);

  // Setelah touch, latch tetap aktif selama lane belum keluar melewati release gap.
  const auto pixel_recover = updatePixelCorridorSide(true, 40.0, true, pixel);
  assert(pixel_recover.status == "YELLOW" && pixel_recover.latched);
  assert(pixel_recover.correction_m > 0.0);

  // Baru setelah gap >= release, status kembali hijau dan koreksi nol.
  const auto pixel_released = updatePixelCorridorSide(true, pixel.release_gap_px, true, pixel);
  assert(pixel_released.status == "GREEN" && !pixel_released.latched);
  assert(std::abs(pixel_released.correction_m) < 1.0e-12);

  // Konvensi arah: lane kiri touch -> center error negatif -> belok kanan.
  // Lane kanan touch -> center error positif -> belok kiri.
  MixerConfig corridor_mixer;
  const auto steer_right = mixRecenterCommand(
    0.20, 0.0, -pixel_touch.correction_m, 0.0, false, corridor_mixer);
  const auto steer_left = mixRecenterCommand(
    0.20, 0.0, pixel_touch.correction_m, 0.0, false, corridor_mixer);
  assert(steer_right.angular_z < 0.0);
  assert(steer_left.angular_z > 0.0);

  // Mixer harus mempertahankan arah maju, membatasi speed recenter, dan finite.
  MixerConfig mixer;
  const auto cmd = mixRecenterCommand(0.30, 0.0, 0.50, 0.0, false, mixer);
  (void)cmd;  // Tetap dianggap used saat assert dihapus oleh NDEBUG pada Release build.
  assert(cmd.linear_x > 0.0 && cmd.linear_x <= mixer.recenter_speed_mps + 1.0e-9);
  assert(std::isfinite(cmd.angular_z));
  assert(cmd.angular_z > 0.0);

  return 0;
}
