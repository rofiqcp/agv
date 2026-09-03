#include <cmath>
#include <cstdlib>
#include <iostream>

// Real assertion (never disabled by NDEBUG) so this self-check actually
// verifies the steering geometry in Release builds instead of passing vacuously.
static void check(bool ok, const char *msg) {
  if (!ok) { std::cerr << "FAIL " << msg << "\n"; std::exit(1); }
}
int main(){
  const double track=0.48, leff=0.735, steer_deg=28.0;
  const double d=steer_deg*3.14159265358979323846/180.0;
  const double r_center=0.5*track+leff/std::tan(d);
  // Build an exact 90-degree arc endpoint and recover the radius/wheelbase.
  const double x=r_center, y=r_center;
  const double recovered_r=(x*x+y*y)/(2.0*std::abs(y));
  const double recovered_l=(recovered_r-0.5*track)*std::tan(std::abs(d));
  check(std::abs(recovered_r-r_center)<1e-12, "recovered radius");
  check(std::abs(recovered_l-leff)<1e-12, "recovered wheelbase");
  const double left_r=1.55,right_r=1.62,margin=0.10;
  const double certified=std::max(left_r,right_r)*(1.0+margin);
  check(std::abs(certified-1.782)<1e-12, "certified radius");
  return 0;
}
