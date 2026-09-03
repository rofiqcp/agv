#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

// Real assertion (never disabled by NDEBUG) so this self-check actually
// verifies the LUT math in Release builds instead of passing vacuously.
static void check(bool ok, const char *msg) {
  if (!ok) { std::cerr << "FAIL " << msg << "\n"; std::exit(1); }
}

static double interp(
  double x,
  const std::vector<double> & xs,
  const std::vector<double> & ys)
{
  if (x <= xs.front()) {
    return ys.front();
  }
  if (x >= xs.back()) {
    return ys.back();
  }
  const auto it = std::upper_bound(xs.begin(), xs.end(), x);
  const std::size_t hi = static_cast<std::size_t>(std::distance(xs.begin(), it));
  const std::size_t lo = hi - 1U;
  const double ratio = (x - xs[lo]) / (xs[hi] - xs[lo]);
  return ys[lo] + ratio * (ys[hi] - ys[lo]);
}

int main()
{
  const std::vector<double> physical{-30, -20, -10, 0, 10, 20, 28};
  const std::vector<double> cmd_inc{-88, -63, -34, -4, 26, 57, 84};
  const std::vector<double> cmd_dec{-86, -61, -32, -2, 28, 59, 86};
  const std::vector<double> fb_inc{-154, -110, -61, -3, 22, 43, 59};
  const std::vector<double> fb_dec{-151, -107, -58, 0, 25, 46, 62};

  check(std::abs(interp(-15, physical, cmd_inc) - (-48.5)) < 1.0e-9, "lut inc interp");
  check(std::abs(interp(15, physical, cmd_dec) - 43.5) < 1.0e-9, "lut dec interp");
  check(std::abs(interp(22, fb_inc, physical) - 10.0) < 1.0e-9, "lut fb inc interp");
  check(std::abs(interp(0, fb_dec, physical)) < 1.0e-9, "lut fb dec interp");
  check(std::abs((interp(20, physical, cmd_inc) - interp(20, physical, cmd_dec)) - (-2.0)) < 1.0e-9, "lut inc/dec asymmetry");
  return 0;
}
