# Steering Physical Calibration Part 2 — Multi-Point LUT + Hysteresis

Part 2 upgrades the safe three-point physical calibration from Part 1 into two directional piecewise-linear LUTs. ESC protocol values remain protocol units; `/esc/steering_actual_rad` remains a physical inner-wheel angle.

## Field procedure

1. Complete Part 1 first and confirm the vehicle is stationary.
2. Enter steering calibration mode. Drive output is inhibited by the existing ESC calibration gate.
3. Select **Sweep KIRI → KANAN (increasing)**. Start at the left mechanical side and move only toward the right.
4. At each chosen wheel angle (recommended approximately every 5 degrees), measure the physical **inner-wheel** angle relative to the chassis centerline, enter that physical angle in the GUI, hold the mechanism steady for about one second, then press **Capture Titik LUT**.
5. Include at least five points, with one point near 0 degrees and good coverage of both left and right sides. 7–13 points is preferred.
6. Select **Sweep KANAN → KIRI (decreasing)** and repeat the same physical points while moving only toward the left.
7. The GUI rejects incomplete or non-monotonic data. Review `ΔCMD` and `ΔFB`; these are measured hysteresis/backlash in the ESC protocol/feedback domains.
8. Export the LUT CSV for the test report, then press **TERAPKAN LUT PART 2**.
9. The controller saves both directional LUTs. Command mapping chooses the LUT from the requested steering motion direction. Feedback mapping uses the same direction so the physical wheel angle is not forced through a single linear curve.
10. If the LUT is invalid or disabled, the Part-1 three-point physical calibration remains the fallback.

## Recommended points

Use real measured angles, not assumed targets. A practical initial set is approximately `-30,-25,-20,-15,-10,-5,0,+5,+10,+15,+20,+25,+28 deg`, clipped to the actual mechanical endpoints measured in Part 1.

## Important

Do not move back and forth inside one sweep. Backlash identification only makes sense if KIRI→KANAN and KANAN→KIRI are collected as separate monotonic passes.
