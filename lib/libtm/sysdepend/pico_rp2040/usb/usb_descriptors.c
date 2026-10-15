/* Minimal USB CDC ACM descriptors for the Thread-Metric console. */

#include <string.h>
#include "tusb.h"

#define USB_VID 0x2e8a
#define USB_PID 0x000a
#define USB_BCD 0x0100

static tusb_desc_device_t const device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = USB_BCD,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&device_descriptor;
}

enum {
    ITF_CDC_CONTROL,
    ITF_CDC_DATA,
    ITF_COUNT,
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static uint8_t const configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_TOTAL_LEN, 0, 100),
    TUD_CDC_DESCRIPTOR(ITF_CDC_CONTROL, 4, 0x81, 8, 0x02, 0x82, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return configuration_descriptor;
}

static char const *const string_table[] = {
    (const char[]){ 0x09, 0x04 },
    "micro T-Kernel Forum",
    "MTK3 Thread-Metric RP2040",
    "MTK3PICO",
    "Thread-Metric console",
};
static uint16_t string_descriptor[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    size_t count;
    (void)langid;

    if (index >= sizeof(string_table) / sizeof(string_table[0])) return NULL;
    if (index == 0) {
        memcpy(&string_descriptor[1], string_table[0], 2);
        count = 1;
    } else {
        char const *text = string_table[index];
        for (count = 0; count < 31 && text[count] != '\0'; ++count) {
            string_descriptor[1 + count] = (uint8_t)text[count];
        }
    }
    string_descriptor[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * count + 2));
    return string_descriptor;
}
