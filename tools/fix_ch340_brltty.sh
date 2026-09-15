#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR=/home/sirobo/agv

echo '[1/7] Stop BRLTTY USB takeover'
systemctl stop brltty-udev.service brltty.service 2>/dev/null || true
pkill -x brltty 2>/dev/null || true
# Mask only the udev auto-start path; keep manual accessibility service package intact.
systemctl mask brltty-udev.service >/dev/null 2>&1 || true
# Override vendor udev rule that falsely classifies QinHeng 1a86:7523 as Braille.
ln -sfn /dev/null /etc/udev/rules.d/85-brltty.rules

echo '[2/7] Install AGV serial identity rules'
install -m 0644 "$ROOT_DIR/tools/99-agv-serial.rules" /etc/udev/rules.d/99-agv-serial.rules
udevadm control --reload-rules
modprobe ch341
modprobe cp210x

echo '[3/7] Detach any stale USB-serial bindings'
for ifc in /sys/bus/usb/devices/*:1.0; do
  [ -r "$ifc/../idVendor" ] || continue
  vid=$(cat "$ifc/../idVendor"); pid=$(cat "$ifc/../idProduct")
  case "$vid:$pid" in
    1a86:7523|10c4:ea60)
      if [ -L "$ifc/driver" ]; then
        drv=$(readlink -f "$ifc/driver")
        echo "$(basename "$ifc")" >"$drv/unbind" || true
      fi
      ;;
  esac
done
sleep 0.5

echo '[4/7] Bind CH340 first -> expected ttyUSB0'
for ifc in /sys/bus/usb/devices/*:1.0; do
  [ -r "$ifc/../idVendor" ] || continue
  [ "$(cat "$ifc/../idVendor"):$(cat "$ifc/../idProduct")" = '1a86:7523' ] || continue
  echo "$(basename "$ifc")" >/sys/bus/usb/drivers/ch341/bind || true
done
udevadm settle
sleep 0.5

echo '[5/7] Bind CP2102 after CH340 -> expected ttyUSB1'
for ifc in /sys/bus/usb/devices/*:1.0; do
  [ -r "$ifc/../idVendor" ] || continue
  [ "$(cat "$ifc/../idVendor"):$(cat "$ifc/../idProduct")" = '10c4:ea60' ] || continue
  echo "$(basename "$ifc")" >/sys/bus/usb/drivers/cp210x/bind || true
done
udevadm trigger --subsystem-match=tty --action=add
udevadm settle
sleep 0.5

echo '[6/7] Verify persistent aliases'
ls -l /dev/ttyUSB* /dev/agv_ch340 /dev/agv_gnss /dev/agv_imu 2>/dev/null || true
ls -l /dev/serial/by-id 2>/dev/null || true

echo '[7/7] Verify driver ownership'
for n in /dev/ttyUSB*; do
  [ -e "$n" ] || continue
  echo "--- $n ---"
  udevadm info -q property -n "$n" | grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL|ID_PATH)=' || true
done
