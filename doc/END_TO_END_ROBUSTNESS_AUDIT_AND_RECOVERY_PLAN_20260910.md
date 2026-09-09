# AGV End-to-End Robustness Audit & Recovery Plan

Tanggal audit: 2026-09-10
Target: ROSWeb localhost:5000 -> ROS/Ackermann -> STM32F411 -> STM32F103RCT6 -> dual FOC motor, dengan command dan feedback realtime 50 Hz yang deterministik, fail-safe, dan recoverable.

## 1. Scope

Tree yang diaudit:
- `/home/sirobo/agv`
- `/home/sirobo/agv/hoverboard-vesc`
- `/home/sirobo/agv/f411_pio_arduino`
- `/home/sirobo/agv/F4gateway`
- ROS packages `src/esc`, `src/stmf4`, `src/navigation/web`

Audit membandingkan 10-15 commit terakhir AGV dan hoverboard-vesc, histori F411 lama, F4gateway baru, working tree saat ini, serta hasil pengukuran hardware live sebelumnya.

## 2. Executive conclusion

Masalah utama saat ini bukan kapasitas 115200 baud dan bukan F103 parser/FOC.
Regresi terbesar muncul ketika gateway berpindah dari implementasi STM32 Arduino yang matang ke implementasi native HAL di `f411_pio`/`F4gateway`.

Arduino core lama memiliki TX ring yang self-chaining langsung dari TX-complete IRQ. Native `HalUartPort` awal hanya mengosongkan `tx_busy_` pada callback dan bergantung pada pemanggilan `service()` berikutnya. `Board_Service()` juga tidak dipanggil dari main loop awal. Ini dapat membuat backlog dan recovery storm di bawah burst ROS 50 Hz.
