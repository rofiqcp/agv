# ADV Web GUI + USB Routing — 2026-09-02

## Runtime layout

- Browser HMI: `http://localhost:5000`
- Backend: `navigation/agv_web_gui` (C++17 + rclcpp + Qt Network)
- Static frontend: HTML/CSS/JavaScript, fully local/offline, no npm/CDN.
- `autonomous.launch.py` and `gui.launch.py` start the web GUI by default.
- Default bind is `127.0.0.1`. For a trusted LAN only, explicitly set `web_bind_address:=0.0.0.0`.

## USB routing on the current mini-PC

Current USB-UART identities are distinct and are the primary automatic selectors:

| Device | Primary `/dev/serial/by-id` selector | Fallback `/dev/serial/by-path` | observed tty |
|---|---|---|---|
| IMU | `Silicon_Labs_CP2102` (`10c4:ea60`) | `usb-0:3.1:1.0` | `/dev/ttyUSB0` |
| GNSS | `1a86_USB_Serial` (`1a86:7523`) | `usb-0:3.4:1.0` | `/dev/ttyUSB1` |
| ESC | `Prolific_Technology_Inc._USB-Serial_Controller` (`067b:2303`) | `usb-0:1.1:1.0` | `/dev/ttyUSB2` |

`ttyUSB` numbers are diagnostic only. Every driver resolves a unique stable by-id first, so kernel renumbering or moving the adapter to another USB socket does not swap devices. The configured physical by-path is used only when the by-id selector is unavailable or ambiguous. If neither selector resolves exactly one device, auto-routing fails closed instead of guessing another serial port. GNSS still requires an NMEA/UBX protocol probe, IMU requires a valid WIT `0x55` checksum stream, and ESC is not considered ready until fresh firmware ACK/ready flags are received.

## ESC no-motion diagnosis

The ESC UART is a Prolific PL2303 (`067b:2303`). The revised ESC node first resolves its unique `/dev/serial/by-id` entry and only falls back to the configured physical path when identity matching is unavailable or ambiguous. The browser ESC page shows serial state, active path, ACK, firmware readiness, command source, target command, E-STOP and feedback.

Read-only USB identity validation can be run before launch:

```bash
cd ~/ros
python3 src/navigation/tools/serial_usb_preflight.py --workspace ~/ros
```

Recommended validation after launch:

```bash
cd ~/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
bash src/verify_minipc_runtime.sh
ros2 topic echo /esc/status --once
ros2 topic echo /esc/feedback_valid --once
ros2 topic echo /esc/ready --once
ros2 topic echo /esc/mux/active_source --once
ros2 topic echo /esc/drive_target_mps --once
ros2 topic echo /esc/drive_actual_mps --once
```
