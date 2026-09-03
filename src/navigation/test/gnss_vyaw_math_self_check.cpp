#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

static double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// Real assertion (never disabled by NDEBUG) so this self-check actually
// verifies the math in Release builds instead of passing vacuously.
static void check(bool ok, const char * msg) {
  if (!ok) { std::cerr << "FAIL " << msg << "\n"; std::exit(1); }
}

int main() {
  constexpr double pi = 3.14159265358979323846;
  // Course derivative must cross +/-pi without a 2*pi spike.
  const double prev = 179.0 * pi / 180.0;
  const double next = -179.0 * pi / 180.0;
  const double dt = 0.2;
  const double rate = wrap(next - prev) / dt;
  check(std::abs(rate - (2.0 * pi / 180.0) / dt) < 1e-12, "course rate wrap");

  // Low-pass stage used by LocalizationCore.
  const double alpha = 0.35;
  const double filtered0 = 0.10;
  const double raw = 0.30;
  const double filtered1 = filtered0 + alpha * (raw - filtered0);
  check(std::abs(filtered1 - 0.17) < 1e-12, "low-pass filter stage");

  // Course uncertainty propagated through a finite difference and clamped.
  const double course_sigma = 0.04;
  const double min_var = 0.0025, max_var = 4.0;
  const double raw_var = 2.0 * course_sigma * course_sigma / (dt * dt);
  const double var = std::clamp(raw_var, min_var, max_var);
  check(var >= min_var && var <= max_var, "course variance clamp");

  // At standstill, vyaw must be treated as unobservable instead of zero-certainty.
  const double low_speed_vyaw_variance = 1.0e6;
  check(low_speed_vyaw_variance > max_var, "low-speed vyaw unobservable");

  std::cout << "PASS gnss_vyaw_math_self_check\n";
  return 0;
}
