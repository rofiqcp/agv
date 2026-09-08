# Models

Folder ini menyimpan model runtime yang dibutuhkan package perception.

Binary model **tidak disimpan ke Git**. File `*.pt`, `*.pth`, `*.onnx`, `*.engine`, dan `*.plan` di-ignore agar repository tetap ringan.

## YOLOPv2

Model CPU yang digunakan:

```text
yolopv2.pt
```

Unduh model resmi CAIC-AD/YOLOPv2 dengan:

```bash
cd $AGV_ROOT/models
chmod +x model.sh
./model.sh
```

Untuk mengunduh ulang walaupun file sudah ada:

```bash
./model.sh --force
```

Sumber model: release `V0.0.1` repository resmi `CAIC-AD/YOLOPv2`.

## Semantic obstacle detector (COCO)

YOLOPv2 tetap menjadi sumber drivable/lane dan generic obstacle. Label semantic memakai TorchVision SSDLite320 MobileNetV3 yang dilatih pada COCO.

Mapping runtime proyek tetap:

```text
0 = person
2 = car
3 = motorcycle
```

Checkpoint disiapkan dengan:

```bash
cd $AGV_ROOT/models
chmod +x semantic_model.sh
./semantic_model.sh
```

Dataset preparation untuk fine-tuning/domain adaptation tersedia di `src/perception/tools/prepare_coco_obstacle_subset.py`.
