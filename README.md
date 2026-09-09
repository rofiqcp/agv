# AGV ROS 2 Workspace

Workspace ROS 2 Humble untuk Autonomous Ground Vehicle Ackermann. Root project selalu memakai `AGV_ROOT`; default portable-nya adalah `$HOME/agv`, sehingga username Linux boleh berbeda tanpa mengubah source.

## 1. Clone Pertama Kali

```bash
git clone --recurse-submodules -b v1 https://github.com/rofiqcp/agv.git "$HOME/agv"
cd "$HOME/agv"
git submodule update --init --recursive
```

Jika repository sudah ada:

```bash
cd "$HOME/agv"
git pull --ff-only
git submodule update --init --recursive
```

Firmware STM32F103 berada pada Git submodule `hoverboard-vesc` dan repository AGV menyimpan commit firmware yang dipin agar versi hardware reproducible.

## 2. Instalasi Otomatis

Entry point instalasi adalah `install.sh`. Script ini idempotent dan boleh dijalankan ulang.
```bash
cd "$HOME/agv"
./install.sh --with-model --with-browser-qa
```

Default build installer dibatasi untuk menjaga NUC/PC lain tetap responsif:

- CPU maksimal 50% mesin.
- RAM build maksimal 50% RAM fisik.
- `Release` build.
- Compile `j1` untuk translation unit LibTorch/Qt yang berat.
- Package dibuild dari `src/` saja agar repository referensi/submodule tidak dipindai sebagai package ROS.

Opsi penting:

```bash
./install.sh --no-build
./install.sh --build-percent 50 --memory-percent 50
./install.sh --with-model
./install.sh --with-browser-qa
```

Installer memasang dependency ROS/system/Python, `rosdep`, PlatformIO, model opsional, browser QA opsional, dan managed environment block ke `~/.bashrc`.

## 3. Environment Shell
Setelah installer selesai, buka shell baru atau jalankan:

```bash
source ~/.bashrc
```

Managed block AGV menetapkan dan men-source environment berikut:

```text
AGV_ROOT=$HOME/agv
ROS_DISTRO=humble
ROS_DOMAIN_ID=42
```

`$AGV_ROOT/scripts/agv_env.sh` otomatis source `/opt/ros/humble/setup.bash` dan `$AGV_ROOT/install/setup.bash` bila tersedia. Karena itu command ROS dapat dijalankan dari direktori mana pun.

Verifikasi:

```bash
cd /tmp
echo "$AGV_ROOT"
echo "$ROS_DOMAIN_ID"
ros2 pkg prefix navigation
ros2 pkg prefix esc
command -v ros2
```

## 4. Struktur Utama

```text
agv/
├── src/navigation/   # Nav2, localization, EKF, ROS Web, calibration
├── src/perception/   # Astra Pro + YOLOPv2 CPU/GPU
├── src/esc/          # Ackermann command mux + VESC bridge
├── src/stmf4/        # Bridge Mini-PC ↔ STM32F411 ↔ GNSS/VESC
├── hoverboard-vesc/  # Git submodule firmware STM32F103 dual FOC
├── models/           # Model YOLOPv2 dan downloader
├── scripts/          # Environment, portability check, browser QA
├── install.sh
└── README.md
```

Folder `build/`, `install/`, dan `log/` adalah hasil lokal dan tidak menjadi source authority.

## 5. Model YOLOPv2

Jika belum memakai `--with-model`:

```bash
cd "$AGV_ROOT/models"
./model.sh
```

Model CPU default:

```text
$AGV_ROOT/models/yolopv2.pt
```

Override path bila diperlukan menggunakan environment `YOLOPV2_PT_PATH` atau `YOLOP_ENGINE_PATH`; source tidak boleh memakai hardcoded `/home/<user>/...`.

## 6. Build dan Test
Gunakan installer untuk build resource-limited yang aman:

```bash
cd "$AGV_ROOT"
./install.sh --build-percent 50 --memory-percent 50
```

Atau untuk incremental build manual setelah environment aktif:

```bash
cd "$AGV_ROOT"
colcon build --base-paths src --symlink-install --executor sequential \
  --allow-overriding perception --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Setelah build:

```bash
source "$AGV_ROOT/install/setup.bash"
colcon test --base-paths src --executor sequential
colcon test-result --verbose
```

Target project adalah 0 warning/error build dan 0 failure test. Portability contract juga dapat dijalankan langsung:

```bash
python3 "$AGV_ROOT/scripts/check_portable_paths.py"
```

## 7. Menjalankan Autonomous Stack
Dari direktori mana pun setelah `source ~/.bashrc`:

```bash
ros2 launch navigation autonomous.launch.py
```

Simpan log runtime:

```bash
ros2 launch navigation autonomous.launch.py | tee "$AGV_ROOT/log/$(date +%Y%m%d_%H%M%S).txt"
```

Untuk QA tanpa risiko command gerak gunakan mode read-only:

```bash
ros2 launch navigation autonomous.launch.py \
  mode:=web web_read_only:=true enable_rviz:=false \
  enable_joystick:=false enable_keyboard:=false
```

ROS Web tersedia di `http://127.0.0.1:5000`.

## 8. Browser QA Playwright

Install sekali lewat:

```bash
cd "$AGV_ROOT"
./install.sh --no-build --with-browser-qa
```

Dengan runtime read-only aktif, jalankan:
```bash
cd "$AGV_ROOT"
node scripts/qa_ros_web_playwright.js
```

QA memeriksa:

- HTTP 200 dan SSE live.
- Tidak ada console/page error.
- Tidak ada HTTP 4xx/5xx atau request failure.
- Tidak ada POST otomatis saat backend read-only.
- Map Navigation terlihat.
- Dua grafik ESC terlihat.
- Frame kamera valid.
- Configuration dan Test workspace dapat dibuka.
- Data Health terisi.
- Layout mobile tidak overflow horizontal.

## 9. Git dan Submodule

Branch utama AGV adalah `v1`. Perubahan firmware harus di-commit/push di dalam `hoverboard-vesc` terlebih dahulu, lalu pointer submodule di repository AGV diperbarui.

```bash
cd "$AGV_ROOT/hoverboard-vesc"
git status
# commit/push firmware bila ada perubahan

cd "$AGV_ROOT"
git add hoverboard-vesc
git commit -m "chore: update hoverboard-vesc submodule"
```
