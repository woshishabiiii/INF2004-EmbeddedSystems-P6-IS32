#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#define CFG_TUSB_MCU             OPT_MCU_RP2040
#define CFG_TUSB_OS              OPT_OS_NONE
#define CFG_TUSB_DEBUG           0
#define CFG_TUSB_RHPORT0_MODE    (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#define CFG_TUD_ENABLED          1
#define CFG_TUD_ENDPOINT0_SIZE   64
#define CFG_TUD_CDC              1
#define CFG_TUD_MSC              0
#define CFG_TUD_HID              0
#define CFG_TUD_MIDI             0
#define CFG_TUD_VENDOR           0
#define CFG_TUD_AUDIO            0
#define CFG_TUD_DFU              0
#define CFG_TUD_DFU_RUNTIME      0
#define CFG_TUD_ECM_RNDIS        0
#define CFG_TUD_NCM              0
#define CFG_TUD_USBTMC           0
#define CFG_TUD_VIDEO            0

#define CFG_TUD_CDC_RX_BUFSIZE   64
#define CFG_TUD_CDC_TX_BUFSIZE   256
#define CFG_TUD_CDC_EP_BUFSIZE   64

#endif
