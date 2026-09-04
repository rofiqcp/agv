#!/usr/bin/env bash
set -euo pipefail

# Setup/audit reproducible untuk Mini-PC ROS2 + YOLOPv2 CPU.
# Build sengaja dibatasi agar tidak memenuhi CPU/RAM mesin otomasi.
WS="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
cd "$WS"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[ERROR] ROS2 Humble tidak ditemukan" >&2
  exit 2
fi
source /opt/ros/humble/setup.bash

# Bersihkan overlay lama agar prefix package yang sudah dihapus tidak bocor.
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash

echo "[1/6] Timestamp future dinormalisasi"
now=$(date +%s)
find src -type f -newermt "@$((now + 5))" -print0 | while IFS= read -r -d '' f; do
  touch -d "@$now" "$f"
done

echo "[2/6] Verifikasi model models/yolopv2.pt"
MODEL="$WS/models/yolopv2.pt"
if [[ ! -s "$MODEL" ]]; then
  echo "[ERROR] Model tidak ada/kosong: $MODEL" >&2
  echo "Jalankan models/model.sh untuk menyiapkan model resmi." >&2
  exit 3
fi
echo "[3/6] Build maksimal sekitar 70% CPU"
cores=$(nproc)
workers=$(( cores * 70 / 100 ))
(( workers < 1 )) && workers=1
# Untuk 4 core hasilnya 2 worker; sengaja dibulatkan ke bawah demi headroom ROS/web.
export CMAKE_BUILD_PARALLEL_LEVEL="$workers"
export MAKEFLAGS="-j$workers"
colcon build --parallel-workers "$workers" --executor parallel \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo "[4/6] Aktifkan overlay hasil build"
source install/setup.bash

echo "[5/6] Jalankan regression test utama"
colcon test --packages-select esc perception navigation --parallel-workers "$workers"
colcon test-result --verbose

echo "[6/6] Preflight hardware/runtime"
python3 src/navigation/tools/minipc_nav2_preflight.py --workspace .

echo "[PASS] Setup Mini-PC selesai. Untuk validasi hardware ketat gunakan:"
echo "python3 src/navigation/tools/minipc_nav2_preflight.py --workspace . --require-hardware --require-esc"
