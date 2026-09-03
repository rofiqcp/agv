# Trajectory Safety Integration — Perception × Nav2 × Localization × ESC

## Prinsip arsitektur

Nav2 tetap menjadi pemilik **goal, trajectory, dan velocity utama**. Kamera tidak
menjadi sumber command final. Perception hanya menghasilkan fakta/constraint:

- candidate obstacle yang sudah lolos class + drivable + metric gate,
- lane safety state dan yaw advisory untuk recenter,
- camera connection/visibility health,
- near-field emergency stop.

`trajectory_safety_supervisor` di package `navigation` menggabungkan semuanya
dengan future path Nav2 dan pose localization sebelum command masuk
`NavigationCore` lalu ESC.

```text
Smac Hybrid / MPPI
   |-- /plan -----------------------------------------------+
   |-- /controller_server/transformed_global_plan ----------+-- future path
   |-- /cmd_vel_nav_smoothed -------------------------------+-- command utama
                                                            |
Perception                                                   |
   |-- /perception/object_points (candidate) ---------------+
   |-- /perception/lane_safety_state -----------------------+
   |-- /perception/lane_control_state ----------------------+
   |-- /cmd_vel/perception_advisory ------------------------+
   |-- /perception/camera_connected ------------------------+
   |-- /perception/camera_healthy --------------------------+
   `-- /perception/emergency_stop --------------------------+
                                                            v
                                            trajectory_safety_supervisor
                                                            |
                                       /cmd_vel/autonomy_integrated
                                                            |
                                                     NavigationCore
                                                            |
                                                        /cmd_vel
                                                            |
                                                           ESC
```

## Implementasi requirement 1–6

### 1. Obstacle jauh / dekat tetapi tidak relevan dengan belokan

Ada dua gate berurutan.

**Absolute metric candidate horizon** di perception:

```yaml
metric_maximum_forward_m: 4.0
```

Object di luar horizon tersebut tidak menjadi `object_points` safety, walaupun raw
YOLO detection masih boleh terlihat untuk debugging.

Setelah itu supervisor menguji posisi obstacle terhadap **future Nav2 path**, bukan
sekadar jarak lurus kamera:

```yaml
vehicle_width_m: 0.60
trajectory_lateral_margin_m: 0.25
path_horizon_m: 4.0
```

Collision corridor half-width = 0.60/2 + 0.25 = 0.55 m.

Parked car yang berjarak dekat secara Euclidean tetapi berada di seberang belokan
akan tetap `candidate`, namun jika tidak memotong corridor future path maka:

```text
path_relevant = false
=> tidak hard-stop
=> tidak slow
=> tidak masuk Collision Monitor collision cloud
```

Supervisor juga membuat planning envelope yang lebih lebar:

```yaml
planning_lateral_margin_m: 0.65
```

`/perception/planning_relevant_points` masuk local costmap/MPPI supaya obstacle
tetap terlihat selama planner melakukan manuver menghindar, tanpa kembali memasukkan
seluruh drivable area sebagai obstacle.

Command-swept corridor sengaja hanya pendek (`0.90 m`) dan hanya menjadi last-resort
hard stop bila obstacle sudah sangat dekat (`<=0.65 m`). Ia bukan pengganti future
Nav2 path sehingga tidak menghidupkan kembali kasus parked-car di seberang tikungan.

### 2. BBox yang tidak menyentuh drivable area diabaikan

Normal obstacle wajib lolos `require_drivable_contact`. GPU mengambil 15 sample
(5 kolom x 3 baris) pada **foot-contact band di tepi bawah bbox**. Hanya satu
baris support yang boleh berada 2 px di bawah bottom bbox untuk mengompensasi
noise mask; tidak ada pencarian drivable jauh di bawah object.

```yaml
require_drivable_contact: true
drivable_contact_min_samples: 3
drivable_contact_min_fraction: 0.20
```

Jadi bbox di trotoar/samping jalan tidak menjadi obstacle hanya karena pixel jalan
berada beberapa pixel di bawah bbox.

### 3. Normal obstacle hanya manusia, mobil, motor

```yaml
accept_all_detected_classes_as_obstacles: false
safety_obstacle_class_ids: [0, 2, 3]
```

Filter class normal diterapkan sebelum obstacle masuk tracking/costmap safety.
Namun semantic class ID pada TensorRT engine harus divalidasi pada kendaraan melalui:

```bash
ros2 topic echo /perception/raw_detections
```

Source tidak mengarang nama subtype dari angka logit. Startup juga mencetak warning
allow-list agar pengujian tidak lupa memverifikasi mapping engine aktual.

### 4. Object terlalu dekat / bbox terpotong

Sebelum koordinat bbox di-clamp ke image, perception menyimpan `clip_flags`.
Jika bbox:

- terpotong di bottom frame,
- confidence cukup,
- bbox cukup tinggi,
- overlap central ego corridor,

maka `/perception/emergency_stop` dilatch setelah confirmation frames. Emergency
near-field ini **tidak memakai allow-list class normal**.

Tambahan fail-safe untuk object yang terlalu dekat sampai detector tidak lagi
menghasilkan bbox: lower-center drivable ROI dipantau. Bila drivable mask di area
tepat depan kendaraan hilang secara persisten, state menjadi
`NEAR_FIELD_DRIVABLE_OCCLUDED` dan emergency stop aktif.

### 5. Lensa kamera tertutup

`camera_connected` tidak cukup karena device V4L2 masih connected saat lensa ditutup.
Perception sekarang mempunyai dua lapis visibility guard:

1. `camera_healthy` dari mean luminance, standard deviation, extreme dark/bright
   fraction, dan spatial gradient frame asli.
2. near-field drivable visibility guard dari segmentation di lower-center image.

Supervisor memakai:

```yaml
require_camera_connected: true
require_camera_health: true
```

Kamera disconnected, visibility tidak sehat, atau near-field view tertutup menjadi
veto STOP sebelum command mencapai actuator.

### 6. Safety line / danger-zone recenter

Lane threshold:

```yaml
edge_warning_clearance_m: 1.00
edge_critical_clearance_m: 0.40
```

State:

```text
NORMAL
RECENTER_LEFT
RECENTER_RIGHT
BLOCKED_STOP
LANE_LOST
```

Jika terlalu dekat garis kanan -> `RECENTER_LEFT`; terlalu dekat garis kiri ->
`RECENTER_RIGHT`. Perception menghitung advisory dari command Nav2 + center/heading
correction, bukan command independen.

Sebelum recenter, `obstacleBlocksRecenter()` mengecek swept lateral corridor. Jika
ruang geser berisi obstacle, state `BLOCKED_STOP` menjadi veto di supervisor.

Jika hanya satu boundary terlihat stabil, boundary lawan boleh diestimasi memakai
`nominal_road_width_m`; clearance terhadap boundary yang benar-benar terlihat tetap
bisa memicu danger-zone recenter. Confidence diturunkan untuk menandai estimasi.

## Dua PointCloud dengan tujuan berbeda

```text
/perception/object_points
    candidate setelah class+drivable+metric
            |
            +--> narrow future-path filter
            |       /perception/path_relevant_points
            |       -> Collision Monitor preview + hard stop/slow supervisor
            |
            `--> wider planning envelope
                    /perception/planning_relevant_points
                    -> local costmap / MPPI
```

Pemisahan ini mencegah dua masalah:

1. semua object di seluruh drivable area memblokir planner;
2. obstacle hilang dari costmap segera setelah MPPI mulai sedikit bergeser lalu
   planner berosilasi kembali ke trajectory lama.

## Decision priority supervisor

```text
1  Nav2 command stale                         -> STOP
2  perception emergency (near field)         -> STOP
3  camera disconnected                       -> STOP
4  camera unhealthy/covered                  -> STOP
5  moving tetapi future plan unavailable     -> STOP
6  obstacle stream stale                     -> STOP
7  lane recenter corridor blocked            -> STOP
8  immediate obstacle <= 0.65 m              -> STOP
9  obstacle on future path <= 0.90 m          -> STOP
10 obstacle on future path < 2.20 m           -> proportional SLOW
11 lane warning/critical                      -> RECENTER + speed cap
12 selain itu                                 -> Nav2 PASS
```

## Topic debugging utama

```bash
ros2 topic echo /navigation/trajectory_safety_state
ros2 topic echo /cmd_vel_nav_smoothed
ros2 topic echo /cmd_vel/perception_advisory
ros2 topic echo /cmd_vel/autonomy_integrated
ros2 topic echo /perception/raw_detections
ros2 topic echo /perception/camera_health_state
ros2 topic echo /perception/near_field_state
ros2 topic echo /perception/lane_safety_state
ros2 topic echo /perception/lane_control_state
ros2 topic hz /perception/object_points
ros2 topic hz /perception/path_relevant_points
ros2 topic hz /perception/planning_relevant_points
ros2 topic echo /controller_server/transformed_global_plan
```

Field penting `/navigation/trajectory_safety_state`:

- `decision`
- `path_source`
- `candidate_points`
- `path_relevant_points`
- `planning_relevant_points`
- `min_obstacle_path_m`
- `min_obstacle_command_m`
- `camera_connected`
- `camera_health_ok`
- `emergency`
- `lane_state`
- `lane_recenter_blocked`
- `speed_scale`
- `nav_v/nav_w`
- `out_v/out_w`

## Commissioning bypass

Untuk uji planner tanpa menerapkan safety command:

```bash
ros2 launch navigation autonomous.launch.py enable_trajectory_safety:=false
```

`NavigationCore` kembali memakai `/cmd_vel_nav_smoothed`. Supervisor tetap hidup
selama perception aktif supaya path-filtered cloud dan diagnostic dapat divalidasi
lebih dahulu. Pada operasi autonomous final gunakan default
`enable_trajectory_safety:=true`.

## V3 — Free-space-aware local avoidance

V3 mengubah supervisor dari obstacle gate menjadi arbitration yang mendukung local avoidance:

- `/perception/drivable_boundary_points` dibentuk langsung dari drivable head YOLOPv2 pada GPU.
- Lane geometry yang sudah valid/confident dapat mempersempit boundary tersebut.
- Local costmap memiliki `drivable_boundary_layer` terpisah dari `obstacle_layer`.
- `/perception/planning_relevant_points` berisi seluruh obstacle valid pada ruang manuver lokal, bukan hanya obstacle pada current path.
- `/perception/path_relevant_points` tetap narrow dan dipakai untuk safety collision diagnostics.
- MPPI memilih local detour. Supervisor tidak memerintah arah kiri/kanan secara langsung.
- `/navigation/avoidance_hint` hanya diagnostic tentang sisi free-space yang tersedia.
- Obstacle pada global path tidak otomatis stop. Bila current MPPI command sudah menjauh dan free corridor cukup, state berubah menjadi `AVOID_LEFT`/`AVOID_RIGHT`.
- Bila tidak ada free corridor, drivable envelope hilang pada jarak dekat, atau command masih menuju obstacle, supervisor fail-closed.

Default minimum free corridor adalah 1.10 m (vehicle 0.60 m + 0.25 m margin di tiap sisi).
