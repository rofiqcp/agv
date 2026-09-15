#!/usr/bin/env bash
# Stable serial endpoints for AGV. These paths survive tty index changes.
export AGV_F411_PORT="/dev/serial/by-id/usb-STMicroelectronics_BLACKPILL_F411CE_CDC_in_FS_Mode_338133833134-if00"
export AGV_IMU_PORT="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
export AGV_GNSS_PORT="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
export AGV_CH340_ALIAS="/dev/agv_ch340"
export AGV_IMU_ALIAS="/dev/agv_imu"
