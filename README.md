# AGV ROS 2 Workspace

## Clone ke PC Baru

Clone repository AGV **beserta seluruh Git submodule** dengan satu perintah:

```bash
git clone --recurse-submodules -b v1 https://github.com/rofiqcp/agv.git $HOME/agv
cd $HOME/agv
git submodule status
```

Jika repository sudah terlanjur di-clone tanpa submodule:

```bash
cd $HOME/agv
git submodule update --init --recursive
```

Repository firmware dapat dibuka langsung di: [rofiqcp/hoverboard-vesc](https://github.com/rofiqcp/hoverboard-vesc).
Folder `hoverboard-vesc` yang tampil di GitHub sebagai submodule akan membuka commit SHA yang dipin oleh repository AGV; hal ini memang perilaku standar GitHub untuk menjaga versi firmware tetap reproducible.

Repository ini berisi source code workspace ROS 2 untuk Autonomous Ground Vehicle (AGV) pada `$AGV_ROOT`.

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
├── hoverboard-vesc/  # Git submodule firmware STM32F103 dual FOC / VESC 6.00
├── .gitmodules
├── .gitignore
└── README.md
```

Folder hasil build ROS 2 (`build/`, `install/`, dan `log/`) tidak disimpan ke Git.

## Firmware `hoverboard-vesc` sebagai Git Submodule

Firmware STM32F103 dipisahkan dari source ROS 2 dan ditautkan sebagai Git submodule:

- Path lokal: `$AGV_ROOT/hoverboard-vesc`
- Repository: `https://github.com/rofiqcp/hoverboard-vesc`
- Branch firmware: `v1`
- Repository AGV menyimpan pointer commit firmware yang sudah dipilih, bukan menyalin seluruh riwayat firmware.

Clone workspace beserta firmware:

```bash
git clone --recurse-submodules -b v1 https://github.com/rofiqcp/agv.git ros
```

Jika repository AGV sudah terlanjur di-clone tanpa submodule:

```bash
cd $AGV_ROOT
git submodule update --init --recursive
```

Untuk mengikuti commit terbaru branch `v1` firmware lalu menyimpan pointer baru di AGV:

```bash
cd $AGV_ROOT
git submodule update --remote --merge hoverboard-vesc
git add .gitmodules hoverboard-vesc
git commit -m "chore: update hoverboard-vesc submodule"
```

Perubahan source firmware harus di-commit dan di-push dari dalam folder
`hoverboard-vesc` terlebih dahulu sebelum pointer submodule di repository AGV diperbarui.

## Persiapan Model

Model binary tidak masuk repository karena ukurannya besar. Unduh model resmi YOLOPv2 dengan:

```bash
cd $AGV_ROOT/models
./model.sh
```

Model akan tersedia sebagai:

```text
$AGV_ROOT/models/yolopv2.pt
```

## Build

```bash
cd $AGV_ROOT
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
cd $AGV_ROOT
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
