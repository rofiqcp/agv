# ADV Web GUI + USB Routing — 2026-09-02

## Runtime layout

- Browser HMI: `http://localhost:5000`
- Backend: `navigation/agv_web_gui` (C++17 + rclcpp + Qt Network)
- Static frontend: HTML/CSS/JavaScript, fully local/offline, no npm/CDN.
- `autonomous.launch.py` and `gui.launch.py` start the web GUI by default.
- Default bind is `127.0.0.1`. For a trusted LAN only, explicitly set `web_bind_address:=0.0.0.0`.

## USB routing on the current mini-PC

GNSS and ESC are both CH340 `1a86:7523` and expose the same `ID_SERIAL=1a86_USB_Serial`.
Therefore `/dev/serial/by-id` cannot distinguish them safely. Automatic routing is based on physical USB topology:

| Device | Runtime selector | observed tty |
|---|---|---|
| IMU | `usb-0:3.1:1.0` | `/dev/ttyUSB0` |
| GNSS | `usb-0:3.4:1.0` | `/dev/ttyUSB1` |
| ESC | `usb-0:1.1:1.0` | `/dev/ttyUSB2` |

The `ttyUSB` numbers are diagnostic only and are never the automatic authority. If a device is moved to another physical socket, the corresponding node fails closed until the YAML selector is deliberately updated or an explicit launch `port` is provided.

## ESC no-motion diagnosis

The old source still expected a Prolific adapter, so the serial worker could never open the current CH340 ESC. The revised ESC node first selects `/dev/serial/by-path/*usb-0:1.1:1.0*`, then requires fresh firmware ACK and ready flags. The browser ESC page shows serial state, ACK, firmware readiness, command source, target command, E-STOP and feedback.

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
