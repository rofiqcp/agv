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
cd /home/otomasi/ros/models
chmod +x model.sh
./model.sh
```

Untuk mengunduh ulang walaupun file sudah ada:

```bash
./model.sh --force
```

Sumber model: release `V0.0.1` repository resmi `CAIC-AD/YOLOPv2`.
