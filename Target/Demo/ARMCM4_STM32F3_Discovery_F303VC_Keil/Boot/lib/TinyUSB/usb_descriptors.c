/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "tusb.h"
#include "stm32f3xx.h"

// These USB VID and PID values are free to use, as long as they meet the requirements
// as outlined here:
//   https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
#define USB_VID   0x16C0  // Van Ooijen Technische Informatica
#define USB_PID   0x05e1  // Free shared USB VID/PID pair for CDC devices
#define USB_BCD   0x0200

//--------------------------------------------------------------------+
// Unique ID string
//--------------------------------------------------------------------+
// Length of the unique ID in bytes. On the STM32F3, the unique
// identifier is 96 bits.
#define UID_STR_BYTE_LEN          (96U/8U)
#define UID_STR_WORD_LEN          (UID_STR_BYTE_LEN / sizeof(uint32_t))

// NUL terminated character array for storing the device's unique
// identifier. Callback function tud_descriptor_string_cb() overwrites
// its default value with the actual device's unique identifier.
static char uniqueIdStr[(UID_STR_BYTE_LEN * 2U) + 1U] = "0123456789ABCDEF";

// Initializes the string that holds the ASCII representation of the device's
// unique identifier.
static void UniqueIdStrInit(void)
{
  uint8_t        charIdx = 0U;
  static uint8_t initialized = 0U;
  struct
  {
    union
    {
      uint8_t b[UID_STR_BYTE_LEN];
      uint32_t w[UID_STR_WORD_LEN];
    };
  } uidData;

  // Only initialize once.
  if (initialized == 0U)
  {
    // Copy the unique device ID into the local structure.
    uidData.w[0] = *(uint32_t volatile *)(UID_BASE);
    uidData.w[1] = *(uint32_t volatile *)(UID_BASE + 0x04U);
    uidData.w[2] = *(uint32_t volatile *)(UID_BASE + 0x08U);
    // Update flag.
    initialized = 1U;
    // Loop through all unique identifier bytes.
    for (uint8_t idx = 0U; idx < UID_STR_BYTE_LEN; idx++)
    {
      // Convert byte value to nibbles.
      uint8_t nibbles[2];
      nibbles[0] = (uidData.b[idx] >> 4U) & 0x0F;
      nibbles[1] = (uidData.b[idx]) & 0x0F;
      // Loop through the nibbles.
      for (uint8_t n = 0U; n < sizeof(nibbles); n++)
      {
        // Convert the nibble to an ASCII character.
        char nibbleChr = (nibbles[n] < 10U) ? nibbles[n] + '0' : nibbles[n] - 10 + 'A';
        // Store the ASCII character in the string.
        uniqueIdStr[charIdx] = nibbleChr;
        // Update indexer for the next character.
        charIdx++;
      }
    }
    // Add NUL termination.
    uniqueIdStr[charIdx] = '\0';
  }
}

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
  .bLength            = sizeof(tusb_desc_device_t),
  .bDescriptorType    = TUSB_DESC_DEVICE,
  .bcdUSB             = USB_BCD,

  // Use Interface Association Descriptor (IAD) for CDC
  // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
  .bDeviceClass       = TUSB_CLASS_MISC,
  .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
  .bDeviceProtocol    = MISC_PROTOCOL_IAD,
  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

  .idVendor           = USB_VID,
  .idProduct          = USB_PID,
  .bcdDevice          = 0x0100,

  .iManufacturer      = 0x01,
  .iProduct           = 0x02,
  .iSerialNumber      = 0x03,

  .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
enum {
  ITF_NUM_CDC_0 = 0,
  ITF_NUM_CDC_0_DATA,
  ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + CFG_TUD_CDC * TUD_CDC_DESC_LEN)

#define EPNUM_CDC_0_NOTIF   0x81
#define EPNUM_CDC_0_OUT     0x02
#define EPNUM_CDC_0_IN      0x82


static uint8_t const desc_fs_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 150),

  // 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 16, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 64),

};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index; // for multiple configurations

  return desc_fs_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
static char const *string_desc_arr[] = {
  (const char[]) { 0x09, 0x04 },   // 0: is supported language is English (0x0409)
  "Feaser https://www.feaser.com", // 1: Manufacturer
  "USB Serial Device",             // 2: Product
  uniqueIdStr,                     // 3: Serials, use chip ID
  "Virtual COM Port",              // 4: CDC Interface
};

static uint16_t _desc_str[32 + 1];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;

  uint8_t chr_count;

  // Make sure the unique identifier string is initialized.
  UniqueIdStrInit();

  if ( index == 0)
  {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  }else
  {
    // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
    // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

    if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

    const char* str = string_desc_arr[index];

    // Cap at max char
    chr_count = (uint8_t) strlen(str);
    if ( chr_count > 31 ) chr_count = 31;

    // Convert ASCII string into UTF-16
    for(uint8_t i=0; i<chr_count; i++)
    {
      _desc_str[1+i] = str[i];
    }
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8 ) | (2*chr_count + 2));

  return _desc_str;
}
