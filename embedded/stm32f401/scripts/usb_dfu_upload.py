Import("env")

import glob
import os
import time

DFU_VID = "0483"
DFU_PID = "df11"
MANUAL_DFU_WAIT_S = 60.0


def _usb_nodes(vid, pid):
    nodes = []
    for vendor_file in glob.glob("/sys/bus/usb/devices/*/idVendor"):
        devdir = os.path.dirname(vendor_file)
        try:
            with open(vendor_file, "r", encoding="ascii") as f:
                vendor = f.read().strip().lower()
            with open(os.path.join(devdir, "idProduct"), "r", encoding="ascii") as f:
                product = f.read().strip().lower()
            if vendor != vid.lower() or product != pid.lower():
                continue
            with open(os.path.join(devdir, "busnum"), "r", encoding="ascii") as f:
                busnum = int(f.read().strip())
            with open(os.path.join(devdir, "devnum"), "r", encoding="ascii") as f:
                devnum = int(f.read().strip())
            nodes.append(f"/dev/bus/usb/{busnum:03d}/{devnum:03d}")
        except (OSError, ValueError):
            continue
    return nodes


def _find_cdc_port():
    configured = env.GetProjectOption("custom_usb_cdc_port", "").strip()
    candidates = []
    if configured:
        candidates.append(configured)
    candidates.extend(sorted(glob.glob(
        "/dev/serial/by-id/usb-STMicroelectronics_BLACKPILL_F411CE_CDC_in_FS_Mode*-if00"
    )))
    candidates.extend(sorted(glob.glob("/dev/ttyACM*")))
    for path in candidates:
        if os.path.exists(path):
            return path
    return None


def _assert_dfu_permission(nodes):
    writable = [n for n in nodes if os.access(n, os.R_OK | os.W_OK)]
    if writable:
        return
    joined = ", ".join(nodes) if nodes else "unknown USB node"
    raise RuntimeError(
        "STM32 ROM DFU is active, but Linux cannot write " + joined + ". "
        "Run ./scripts/install_stm32_udev_rules.sh once with sudo, "
        "then reconnect/reset the board into DFU and rerun pio run -t upload."
    )


def _wait_for_dfu(seconds, manual_hint=False):
    if manual_hint:
        print("[USB-DFU] Firmware lama belum punya BOOT:DFU.")
        print("[USB-DFU] Masuk ROM DFU sekali ini via USB:")
        print("          1) tahan tombol BOOT0")
        print("          2) tekan-lepas NRST/RESET")
        print("          3) lepas BOOT0")
        print(f"[USB-DFU] Menunggu 0483:df11 sampai {int(seconds)} detik ...")

    deadline = time.time() + seconds
    last_tick = -1
    while time.time() < deadline:
        nodes = _usb_nodes(DFU_VID, DFU_PID)
        if nodes:
            print("[USB-DFU] STM32 ROM DFU ready:", ", ".join(nodes))
            _assert_dfu_permission(nodes)
            time.sleep(0.35)
            return True
        if manual_hint:
            remaining = int(deadline - time.time())
            tick = remaining // 10
            if tick != last_tick and remaining > 0:
                last_tick = tick
                print(f"[USB-DFU] waiting... {remaining}s")
        time.sleep(0.10)
    return False


def _request_software_dfu(port):
    print(f"[USB-DFU] Requesting ROM DFU through {port} ...")
    try:
        import serial
        with serial.Serial(port, 115200, timeout=0.20, write_timeout=1.0) as ser:
            time.sleep(0.15)
            ser.reset_input_buffer()
            ser.write(b"BOOT:DFU\n")
            ser.flush()
            time.sleep(0.20)
            try:
                reply = ser.read(256).decode(errors="replace").strip()
            except OSError:
                reply = ""
            if reply:
                print("[USB-DFU] CDC reply:", reply.replace("\r", " "))
            return "ERR:UNKNOWN_COMMAND:BOOT:DFU" not in reply
    except Exception as exc:
        print(f"[USB-DFU] CDC trigger detail: {exc}")
        return True


def _before_upload(source, target, env):
    nodes = _usb_nodes(DFU_VID, DFU_PID)
    if nodes:
        print("[USB-DFU] STM32 ROM DFU already active:", ", ".join(nodes))
        _assert_dfu_permission(nodes)
        return

    port = _find_cdc_port()
    if not port:
        if _wait_for_dfu(MANUAL_DFU_WAIT_S, manual_hint=True):
            return
        raise RuntimeError(
            "BLACKPILL CDC tidak ditemukan dan ROM DFU tidak muncul dalam waktu tunggu."
        )

    software_supported = _request_software_dfu(port)
    if software_supported:
        if _wait_for_dfu(12.0, manual_hint=False):
            return
        raise RuntimeError(
            "BOOT:DFU dikirim tetapi STM32 ROM DFU tidak terdeteksi. "
            "Coba satu kali bootstrap manual BOOT0 + RESET."
        )

    if _wait_for_dfu(MANUAL_DFU_WAIT_S, manual_hint=True):
        return

    raise RuntimeError(
        "Firmware di board masih versi lama. ROM DFU 0483:df11 tidak muncul. "
        "Tahan BOOT0, tekan-lepas RESET, lepas BOOT0 saat uploader sedang menunggu."
    )


env.AddPreAction("upload", _before_upload)
