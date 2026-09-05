#pragma once

// ArduinoCore-STM32 3.x keeps itoa/ltoa declarations in api/itoa.h,
// while TFT_eSPI 2.5.43 expects them to be visible from Arduino.h.
// Force-including this project-local compatibility header keeps the fix
// reproducible without modifying .pio/libdeps.
#include <api/itoa.h>
