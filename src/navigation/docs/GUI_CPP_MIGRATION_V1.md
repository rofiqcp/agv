# GUI C++/Qt Migration V1

## Tujuan

Mengganti `gui/agv_gui.py` (PyQt5/rclpy) secara penuh dengan executable native `agv_gui` berbasis C++17, Qt5 dan rclcpp pada ROS 2 Humble.

## Perubahan arsitektur

1. ROS bridge memakai `rclcpp::executors::MultiThreadedExecutor` pada thread tersendiri.
2. UI hanya diperbarui pada thread Qt melalui signal/slot.
3. Semua subscription disimpan sebagai `SubscriptionBase::SharedPtr` selama bridge hidup.
4. 377 spesifikasi parameter dari 17 halaman dimigrasikan ke `agv_gui_specs.hpp`.
5. YAML dibaca dengan yaml-cpp dan ditulis atomik; editor berusaha melakukan line-patch agar komentar/format file yang ada tetap terjaga.
6. GUI tidak lagi memiliki dependency runtime `rclpy` atau `python3-pyqt5`.
7. `CMakeLists.txt` membangun target `agv_gui`, mengaktifkan AUTOMOC, dan menautkan Qt5 Widgets/Network + yaml-cpp.
8. `package.xml` diperbarui untuk dependency native.

## Fungsi yang dipertahankan

- status koneksi GNSS/IMU/Camera/ESC/Nav2;
- live telemetry, plots dan raw diagnostics;
- saved target, patrol, goal/cancel navigation;
- PGM map, OSM overlay, ground truth dan `/initialpose`;
- steering measured calibration LEFT/CENTER/RIGHT dan atomic runtime apply;
- ESC distance/turning calibration;
- IMU stationary calibration dan CSV;
- GNSS stationary/COG validation, GNSS Part 1–3 fusion status dan enable/disable gate;
- EKF/localization diagnostics;
- Navigation/MPPI A/B/C profile dan sweep pack;
- velocity-smoother qualification;
- camera calibration canvas dan metric safety certification;
- obstacle/lane/trajectory/collision diagnostics;
- CSV + metadata statistik, PNG, rosbag, config snapshot dan CPU/GPU profiling;
- YAML autosave serta Config-vs-Runtime verification.

## Optimasi dibanding Python

- tidak ada GIL pada ROS callback path;
- tidak ada `rclpy` spin thread;
- typed ROS C++ message callback langsung;
- callback ROS terpisah dari Qt event loop;
- redraw halaman memakai timer terukur, bukan callback sensor langsung;
- parameter specification dikompilasi sebagai data C++ statis;
- conversion JSON/KV hanya dilakukan pada status string yang memang membutuhkannya.

## Batas validasi

Source sudah menjalani self-check migrasi/static consistency di environment pembuatan artefak. Final compile harus dijalankan pada Jetson ROS 2 Humble karena environment pembuatan artefak tidak memiliki Qt5/ROS Humble development headers lengkap.
