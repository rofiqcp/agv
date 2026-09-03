#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

// Real assertion (never disabled by NDEBUG) so this self-check actually
// verifies the steering math in Release builds instead of passing vacuously.
static void check(bool ok, const char *msg) {
  if (!ok) { std::cerr << "FAIL " << msg << "\n"; std::exit(1); }
}

static double protocolFromPhysical(double physical, double leftPhysical, double rightPhysical,
                                   double leftCmd, double centerCmd, double rightCmd) {
  if (std::abs(physical) < 1e-12) return centerCmd;
  if (physical > 0) {
    double r=std::clamp(physical/rightPhysical,0.0,1.0);
    return centerCmd+r*(rightCmd-centerCmd);
  }
  double r=std::clamp(physical/leftPhysical,0.0,1.0);
  return centerCmd+r*(leftCmd-centerCmd);
}
static double physicalFromFeedback(double fb, double leftPhysical, double rightPhysical,
                                   double leftFb, double centerFb, double rightFb) {
  const double d=fb-centerFb;
  if (std::abs(d)<1e-12) return 0;
  const double rs=rightFb-centerFb, ls=leftFb-centerFb;
  if (d*rs>0) return std::clamp(d/rs,0.0,1.0)*rightPhysical;
  if (d*ls>0) return std::clamp(d/ls,0.0,1.0)*leftPhysical;
  return 0;
}
int main(){
  const double lphys=-31.5,rphys=28.0,lcmd=-90.0,ccmd=-4.84,rcmd=90.0;
  const double lfb=-155.78,cfb=-3.16,rfb=60.56;
  check(std::abs(protocolFromPhysical(0,lphys,rphys,lcmd,ccmd,rcmd)-ccmd)<1e-9, "center protocol");
  check(std::abs(protocolFromPhysical(rphys,lphys,rphys,lcmd,ccmd,rcmd)-rcmd)<1e-9, "right protocol");
  check(std::abs(protocolFromPhysical(lphys,lphys,rphys,lcmd,ccmd,rcmd)-lcmd)<1e-9, "left protocol");
  check(std::abs(physicalFromFeedback(cfb,lphys,rphys,lfb,cfb,rfb))<1e-9, "center feedback");
  check(std::abs(physicalFromFeedback(rfb,lphys,rphys,lfb,cfb,rfb)-rphys)<1e-9, "right feedback");
  check(std::abs(physicalFromFeedback(lfb,lphys,rphys,lfb,cfb,rfb)-lphys)<1e-9, "left feedback");
  // Halfway each side must preserve asymmetric physical limits.
  check(std::abs(physicalFromFeedback((cfb+rfb)/2,lphys,rphys,lfb,cfb,rfb)-14.0)<1e-9, "right halfway feedback");
  check(std::abs(physicalFromFeedback((cfb+lfb)/2,lphys,rphys,lfb,cfb,rfb)+15.75)<1e-9, "left halfway feedback");
  return 0;
}
