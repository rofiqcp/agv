# ESC / Steering / Ackermann Audit & Implementation Plan

## 1. Target final
- Nav2 dan velocity_smoother tetap bekerja dalam `Twist`: linear.x = m/s, angular.z = rad/s.
- `esc_ackermann` adalah satu-satunya node yang mengubah command kendaraan menjadi command ESC.
- RIGHT: `m/s -> wheel RPM -> motor RPM -> eRPM -> COMM_SET_RPM`.
- LEFT: steering fisik `-30..0..+30 deg -> COMM_SET_POS 0..180..360 deg`.
- LEFT VESC position harus saturating, bukan wrapping: <0 menjadi 0; >360 menjadi 360.
- Mechanical hard-stop span diukur F103; runtime memakai 95% span sebagai safe span.
- Boot target logical selalu 180, tetapi incremental ABI tidak boleh dianggap absolute tanpa reference policy yang eksplisit.

## 2. Tiga jenis invert harus dipisahkan
1. `foc_encoder_inverted`: electrical ABI/FOC phase direction. Ini milik motor-control dan bukan tombol membalik kiri/kanan kendaraan.
2. `m_invert_direction`: VESC user-facing motor direction. Jangan dipakai sebagai steering endpoint swap jika electrical loop sudah benar.
3. `steering_logical_inverted`: hanya menukar label logical LEFT/RIGHT, yaitu POS 0 <-> POS 360.

Keputusan: bila steering fisik terbalik tetapi FOC stabil, gunakan `steering logical invert`, bukan `FOC Encoder Inverted`.
VESC Tool: jangan klik/toggle FOC Encoder Inverted hanya untuk memperbaiki kiri/kanan. Gunakan Terminal `steering invert 0|1` atau kontrol ROS Web yang akan ditambahkan.

## 3. Temuan bug saat audit
- Boot sekarang memanggil `mc_interface_steering_boot_home()` dan merebase posisi saat power-on sebagai center; ini bisa membuat 180 salah bila roda tidak benar-benar lurus saat boot.
- Steering path masih memiliki legacy ROS calibration/LUT/center-hold code walaupun STM32 path sudah memakai canonical VESC 0..360.
- `pid_pos_raw_to_user()` masih memakai wrap helpers (`signed_pos_deg`/`norm_pos_deg`) yang tidak cocok untuk logical steering saturating 0..360.
- Generic `RESETPOS` menghapus count, tetapi belum menjadi operasi steering-specific `Set Current Position as 180` yang transactional dan persistent.
## 4. Span calibration final
- `Detect Encoder` harus SPAN-ONLY: tidak boleh mengubah offset electrical, ratio, atau `foc_encoder_inverted`.
- Lakukan 2 sweep penuh hard-stop kiri/kanan.
- Tolak hasil jika endpoint/span repeatability >2%.
- Simpan `measured_span_counts` sebagai hasil ukur asli.
- Derive `safe_span_counts = floor(abs(measured_span_counts) * 0.95)`.
- Derive `safe_half_span = safe_span_counts / 2`.
- Contoh measured 4000 -> safe 3800 -> kiri -1900, center 0, kanan +1900.
- Runtime position target harus clamp ke safe endpoints, sehingga motor tidak menekan hard-stop hasil detect.
- Web harus menampilkan sweep1, sweep2, measured span, safe span, safe half span, repeatability %, dan raw TIM4 count.

## 5. Logical steering coordinate
Internal steering coordinate tetap center-zero untuk controller:
- internal `0 count` = logical POS 180.
- internal `-safe_half..+safe_half` = salah satu arah ke POS 0/360 sesuai `steering_logical_inverted`.
- Logical VESC output: `pos360 = clamp(180 + sign * 180 * count / safe_half, 0, 360)`.
- Physical ROS feedback: `deg = clamp((pos360 - 180)/6, -30, +30)`.
- Tidak ada modulo/wrap untuk steering. Raw encoder counter boleh rollover hardware; accumulated signed count ditangani delta wrap seperti sekarang.

## 6. Center / zero adjustment
Tambahkan operasi khusus `Set Current as 180`:
- hanya di maintenance/calibration mode;
- drive RIGHT harus 0 dan steering velocity/current harus settle;
- capture current steering count;
- rebase current count menjadi 0, target juga 0, reset position PID;
- simpan `center_trim_counts` relatif terhadap midpoint kalibrasi bila absolute reference tersedia;
- UI memungkinkan operator bergerak ke mis. POS185 sampai roda lurus lalu klik `Set Current as 180`.
- Setelah klik, posisi saat itu langsung menjadi logical 180 tanpa menghapus measured span.
## 7. Boot policy penting untuk encoder incremental A/B
Encoder A/B tanpa index/absolute reference tidak dapat mengetahui posisi mekanik absolut setelah power-off.
Karena itu `stored raw center count` sendirian tidak cukup menjamin center fisik setelah reboot.
Mode yang aman harus eksplisit:
- `ASSUME_CURRENT_CENTER`: operator memastikan roda lurus sebelum power-on; setelah ABI sync, posisi boot dijadikan logical 180 lalu controller hold.
- `AUTO_HOME`: seek satu mechanical reference secara bounded lalu bergerak ke calibrated center; lebih absolut tetapi steering bergerak saat boot.
- `REQUIRE_CENTER_CONFIRM`: boot tidak memberi torque posisi sampai operator menekan Confirm/Set 180; paling aman untuk commissioning.
Rekomendasi awal: `REQUIRE_CENTER_CONFIRM` selama tuning, lalu `ASSUME_CURRENT_CENTER` hanya jika SOP selalu memastikan roda lurus sebelum power-on.
Jika ingin 180 selalu benar tanpa SOP/homing, hardware harus punya index/limit/absolute encoder.

## 8. Position-loop robustness
- Pertahankan fix anti-damping pada calibrated steering count mode.
- `foc_encoder_inverted` hanya mengoreksi electrical phase/feedback sign pada layer yang memang membutuhkan electrical sign.
- Jangan double-apply inversion pada P, D-process, telemetry, dan logical mapping.
- Steering current ceiling 3 A dan internal Kp multiplier x3 menjadi baseline aman; nilai tuning VESC tetap source-of-truth dan tidak diubah diam-diam.
- Tambah oscillation guard: bila crossing center berulang/kecepatan steering tinggi tanpa settling, release motor dan fault `STEERING_OSCILLATION`.
- Tambah hard endpoint guard berbasis safe span sebelum current command.

## 9. Ackermann conversion
Drive canonical formula saat ini sudah benar:
`eRPM_per_mps = 60 * gear_ratio * pole_pairs / (2*pi*wheel_radius_m)`.
`target_eRPM = target_mps * eRPM_per_mps / drive_odometry_calibration_scale`.
Dengan r=0.145 m, pp=15, gear=1: sekitar 987.858 eRPM per m/s sebelum calibration scale.
Steering canonical formula:
`vesc_pos_deg = clamp((physical_deg + 30) * 6, 0, 360)`.
Hapus pengaruh legacy steering feedback LUT/center hold pada STM32 production transport agar tidak ada mapping kedua.
## 10. Protocol / TCP architecture
Current runtime Ackermann path is NOT TCP:
`esc_ackermann -> /stmf4/vesc/runtime_tx -> stmf4_hmi_bridge -> USB CDC 1 Mbaud -> F411 -> USART 115200 -> F103`.
Ports 65101/65102 belong to `vesc_tool_bridge` maintenance ownership:
- 65101 = Python maintenance/debug priority.
- 65102 = VESC Tool desktop.
Recommendation: do NOT route Nav2/Ackermann realtime through a new 65103 TCP socket. TCP maintenance route changes ownership and is less deterministic than the existing ROS runtime topic.
If 65103 is ever added, make it read-only telemetry/diagnostic, not actuator authority.

## 11. Launch architecture
`src/esc/launch/esc.launch.py` already exists and `autonomous.launch.py` already includes it.
Current gap: `esc.launch.py` starts motor_teleop + ackermann + vesc_tool_bridge, but it does not start `stmf4_hmi_bridge`; therefore it is not yet a complete standalone physical ESC launch.
Plan:
- add optional `start_stmf4_bridge` and HMI serial arguments to `esc.launch.py`;
- default standalone `esc.launch.py` starts F411 bridge + Ackermann + VESC maintenance bridge;
- `autonomous.launch.py` calls this one ESC launch and must not instantiate a duplicate stmf4 bridge;
- preserve optional STM32 GNSS publishing through arguments because F411 is shared gateway.
Result: `ros2 launch esc esc.launch.py` is the minimal actuator commissioning launch; `autonomous.launch.py` composes it.

## 12. ROS Web ESC page
Move/expose all actuator calibration controls under ESC:
- physical target -30..+30;
- VESC target 0..360;
- raw TIM4 encoder;
- accumulated encoder count;
- measured span / safe span / safe half;
- endpoint sweep 1/2 and repeatability;
- logical invert NORMAL/INVERTED toggle;
- electrical FOC encoder invert read-only with red warning: do not use for vehicle left/right;
- Set Current as 180 / Zero Center;
- reset span; run span detect; home/confirm center;
- drive radius, pole pairs, gear ratio, eRPM/m/s factor, odometry calibration scale;
- steering current ceiling, Kp/Ki/Kd, process-D/filter, operational ±28 and mechanical ±30.
Navigation page only shows actuator status, not calibration editors.

## 13. Acceptance sequence after implementation
1. Full offline build/tests, no PSU motion.
2. Read MC/App config hash; electrical encoder config must stay unchanged.
3. Power on in safe commissioning mode, current command 0.
4. Span double-sweep; require repeatability <=2%; derive 95% safe span.
5. Confirm/set center 180.
6. Low-energy position steps: 180 -> 165/195 -> 150/210 -> 90/270; measure overshoot/current/settling.
7. Only after stable, verify endpoint commands 0/360 stop at safe endpoints, never physical hard-stop.
8. Toggle logical invert once and prove only LEFT/RIGHT labels swap; FOC stability must be unchanged.
9. Reboot tests for selected boot policy.
10. Ackermann dry contract test: -30/0/+30 -> 0/180/360 and m/s -> eRPM.
11. Minimal `esc.launch.py` test, then full `autonomous.launch.py` Nav2 chain test.
