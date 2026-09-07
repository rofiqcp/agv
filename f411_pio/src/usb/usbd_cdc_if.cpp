#include "usbd_cdc.h"
#include "UsbCdcPort.h"

static uint8_t UserRxBufferFS[CDC_DATA_FS_OUT_PACKET_SIZE];
static uint8_t UserTxBufferFS[CDC_DATA_FS_IN_PACKET_SIZE];

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Init_FS(void) {
  (void)USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0U);
  (void)USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (int8_t)USBD_OK;
}

static int8_t CDC_DeInit_FS(void) { return (int8_t)USBD_OK; }

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length) {
  (void)cmd;
  (void)pbuf;
  (void)length;
  return (int8_t)USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len) {
  gUsb.onReceive(pbuf, *Len);
  (void)USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (int8_t)USBD_OK;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum) {
  (void)pbuf;
  (void)Len;
  (void)epnum;
  gUsb.onTransmitComplete();
  return (int8_t)USBD_OK;
}
