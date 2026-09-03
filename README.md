# AGV ROS 2 Workspace

Repository ini berisi source code workspace ROS 2 untuk Autonomous Ground Vehicle (AGV) pada `/home/otomasi/ros`.

## Target Environment

- Ubuntu Linux
- ROS 2 Humble
- C++17 / Python 3
- Nav2
- Qt5 + RViz2
- OpenCV
- YOLOPv2 CPU (LibTorch) / GPU (TensorRT, opsional)

## Struktur Repository

```text
agv/
├── src/
│   ├── navigation/   # Nav2, localization, GUI, GNSS fusion, safety supervisor
│   ├── perception/   # Kamera + YOLOPv2 OFF/CPU/GPU
│   └── esc/          # Bridge ESC, command mux, teleoperation
├── models/
│   ├── README.md
│   └── model.sh      # Downloader model YOLOPv2
├── .gitignore
└── README.md
```

Folder hasil build ROS 2 (`build/`, `install/`, dan `log/`) tidak disimpan ke Git.

## Persiapan Model

Model binary tidak masuk repository karena ukurannya besar. Unduh model resmi YOLOPv2 dengan:

```bash
cd /home/otomasi/ros/models
./model.sh
```

Model akan tersedia sebagai:

```text
/home/otomasi/ros/models/yolopv2.pt
```

## Build

```bash
cd /home/otomasi/ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Untuk build paket utama saja:

```bash
colcon build --symlink-install --packages-select esc perception navigation
```

## Menjalankan Autonomous Stack

```bash
cd /home/otomasi/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch navigation autonomous.launch.py
```

Simpan log runtime ke folder lokal `log/` bila diperlukan. Folder tersebut sengaja di-ignore oleh Git.

## Package

### `navigation`
Ackermann Nav2 runtime, localization, GNSS fusion, Qt5/RViz operator GUI, Smac Planner, MPPI Controller, dan safety supervision.

### `perception`
Pipeline kamera dengan mode OFF/CPU/GPU. CPU membaca model TorchScript `yolopv2.pt`; GPU dapat menggunakan TensorRT bila tersedia.

### `esc`
Bridge serial ESC/STM32, Ackermann command mux, dan motor teleoperation.

## Git Branch

Branch utama repository ini adalah `v1`.
