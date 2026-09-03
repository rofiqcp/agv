# Integrasi Nav2 MAP di Dalam GUI

## Lokasi tampilan

Tekan tombol **NAV2 MAP** di kiri atas, tepat di samping tombol **OVERVIEW**. Panel tengah berubah menjadi halaman peta dengan dua subtab:

1. **NAV2 LIVE • MAP / URDF / A* / MPPI** — visualisasi Nav2/RViz native.
2. **GROUND TRUTH • PGM / OSM / TARGET** — kanvas peta lama untuk target, ground truth, OSM, dan ekspor data.

`gui.launch.py` tetap memasukkan `autonomous.launch.py`, tetapi meneruskan `enable_rviz:=false`. Artinya semua node autonomous yang sama tetap berjalan, sedangkan renderer RViz dimasukkan langsung ke proses GUI. Tidak ada window RViz kedua.

## Yang terlihat pada NAV2 LIVE

| Tampilan | Topik / sumber | Default | Makna |
|---|---|---:|---|
| MAP | `/map` | ON | Peta statis dari map server |
| URDF kendaraan | `/robot_description` + TF | ON | Model kendaraan dan pose aktual |
| Goal Pose | `/navigation/goal_request` | ON | Tujuan yang dikirim operator |
| Smac Hybrid-A* Global Path | `/plan` | ON | Jalur global hasil planner |
| MPPI Transformed Plan | `/controller_server/transformed_global_plan` | ON | Jalur lokal yang sedang dipakai controller |
| Kandidat trajectory MPPI | `/trajectories` | ON | Rollout kandidat tersampling dari MPPI |
| Footprint | `/local_costmap/published_footprint` | ON | Batas fisik kendaraan |
| Global Costmap | `/global_costmap/costmap` | OFF | Layer costmap global; aktifkan saat tuning |
| Local Costmap | `/local_costmap/costmap` | **ON** | Obstacle + inflation lokal 8×8 m; ringan dan langsung terlihat |

Planner sebenarnya adalah **Smac Hybrid-A*** dengan motion model `DUBIN`. Jadi istilah “A*” pada tampilan adalah global path dari plugin Smac tersebut, bukan planner baru atau planner duplikat.


## Perbaikan aliran data pada revisi ini

1. **Crash executor dihapus.** `rviz_common::VisualizationManager` pada ROS 2 Humble sudah memiliki `SingleThreadedExecutor` internal dan memanggil `spin_some()` setiap update. GUI sebelumnya menambahkan node RViz yang sama ke executor kedua, sehingga muncul `Node '/agv_gui_process' has already been added to an executor`. Executor kedua sekarang dihapus.
2. **Remap nama proses dihapus.** `gui.launch.py` tidak lagi memberi `name="agv_gui_process"`, karena `launch_ros` mengubah itu menjadi remap `__node` untuk semua node rclcpp di proses yang sama. GUI dan embedded RViz sekarang tetap bernama `/agv_gui` dan `/agv_gui_embedded_rviz`.
3. **URDF late-subscriber aman.** `Description Topic` memakai Reliable + Transient Local untuk `/robot_description`.
4. **Joint URDF benar-benar bergerak.** Node `joint_state_visualizer` menerbitkan steering Ackermann kiri/kanan dan fase roda ke `/joint_states` dari feedback ESC. Saat ESC belum siap, posisi nol tetap dipublikasikan agar model lengkap tetap terlihat.
5. **Inflation lokal langsung terlihat.** `/local_costmap/costmap` default ON dan update incremental `/local_costmap/costmap_updates` ikut disubscribe. Global costmap tetap OFF untuk menjaga mini PC ringan.

## Cara mengirim Goal Pose

1. Tunggu status bawah menunjukkan `NAV2 READY`.
2. Klik **2D GOAL POSE**.
3. Klik pada titik tujuan di peta, tahan, lalu tarik ke arah heading yang diinginkan.
4. Lepaskan mouse. Goal dikirim ke `/navigation/goal_request`.
5. Klik **Geser / Zoom** untuk kembali menggeser peta.
6. Gunakan **Batalkan Goal** bila pengujian harus dihentikan.

`NavigationCore` tetap menjadi gate: ia hanya meneruskan goal ke action `navigate_to_pose` setelah map, localization, calibration, dan safety prerequisite memenuhi kebijakan yang ada.

## Kenapa tetap ringan

- RViz/OGRE dibuat **lazy**, hanya saat halaman `NAV2 MAP` dibuka.
- Update renderer berhenti saat operator kembali ke Overview/tabel/grafik.
- Frame rate renderer dibatasi 10 FPS.
- Global costmap UNDIP yang besar default OFF; **local costmap 8×8 m default ON** agar obstacle/inflation terlihat tanpa beban besar.
- Gambar kamera dan point cloud persepsi berat tidak dimuat di halaman ini.
- MPPI tetap mengoptimasi `batch_size: 1000`, tetapi visualisasi hanya menerbitkan 1 dari setiap 20 rollout dan 1 dari setiap 4 time step.
- Kanvas PGM/OSM lama juga dibuat lazy ketika subtab Ground Truth dibuka.

## Build dan jalankan

```bash
cd /home/otomasi/ros
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select esc perception navigation
source install/setup.bash
ros2 launch navigation gui.launch.py perception_mode:=cpu
```

Untuk menguji tanpa inference:

```bash
ros2 launch navigation gui.launch.py perception_mode:=off
```

## Diagnosis bila suatu layer belum muncul

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 topic echo /robot_description --once
ros2 topic hz /plan
ros2 topic hz /controller_server/transformed_global_plan
ros2 topic hz /trajectories
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

- MAP kosong: cek `map_server` ACTIVE dan QoS `/map` Transient Local.
- URDF tidak muncul: cek `/robot_description`, `/joint_states`, serta TF `map → odom → base_footprint`. `joint_state_visualizer` sekarang mengubah feedback `/esc/steering_actual_rad` + `/esc/drive_actual_mps` menjadi `/joint_states`, sehingga wheel/steering link selalu punya TF. Embedded RobotModel memakai QoS Transient Local agar tidak melewatkan `robot_description` saat renderer dibuka terlambat.
- A* belum muncul: kirim Goal Pose dan tunggu `planner_server` ACTIVE.
- Kandidat MPPI belum muncul: pastikan `FollowPath.visualize: true`, controller ACTIVE, dan goal sedang dijalankan.
- Renderer gagal: cek OpenGL (`glxinfo -B`) serta instalasi `ros-humble-rviz2`; subtab Ground Truth tetap dapat dipakai sebagai fallback.
