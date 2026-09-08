# Historical note — superseded

Dokumen re-audit 2026-08-25 ini telah digantikan oleh arsitektur final 2026-08-28.
Authority saat ini adalah:

- `CPU_GPU_BACKEND_GUIDE.md`
- `$AGV_ROOT/docs/DEPENDENCIES_CPU_GPU.md`
- `$AGV_ROOT/docs/README_MINIPC_FINAL.md`

Kontrak final:

- `off` = kamera aktif, inference tidak dijalankan.
- `cpu` = direct TorchScript `$AGV_ROOT/models/yolopv2.pt` via LibTorch; tidak ada ONNX runtime.
- `gpu` = CUDA/TensorRT `$AGV_ROOT/models/yolopv2.engine` pada Jetson target.

Detail ONNX/OpenCV-DNN CPU pada revisi lama tidak lagi berlaku untuk runtime robot.
