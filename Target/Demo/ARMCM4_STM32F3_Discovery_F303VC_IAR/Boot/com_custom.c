/************************************************************************************//**
* \file         Demo/ARMCM4_STM32F3_Discovery_F303VC_IAR/Boot/com_custom.c
* \brief        Bootloader custom communication interface source file.
* \ingroup      Boot_ARMCM4_STM32F3_Discovery_F303VC_IAR
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2026  by Feaser    http://www.feaser.com    All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You have received a copy of the GNU General Public License along with OpenBLT. It
* should be located in ".\Doc\license.html". If not, contact Feaser to obtain a copy.
*
* \endinternal
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include "boot.h"                                /* bootloader generic header          */
#if (BOOT_COM_CUSTOM_ENABLE > 0)
#include "tusb.h"                                /* TinyUSB stack                      */
#include "stm32f3xx.h"                           /* STM32 CPU and HAL header           */


/****************************************************************************************
* Background information
****************************************************************************************/
/* This source file implements a custom XCP communication interface for the OpenBLT
 * bootloader. It builds on the TinyUSB communication stack to construct a CDC-ACM
 * device.
 *
 * This custom XCP communication interface makes it possible to select RS232 as the
 * communcation interface in the MicroBoot (or BootCommander) and perform updates via
 * USB as a virtual COM-port.
 *
 * It was purely added for demonstration purposes, allowing you to have a reference for
 * how to build your own custom XCP communication interface for the OpenBLT bootloader.
 *
 * Thanks to this feature to add a custom XCP communication interface, you can perform
 * firmware updates via any type of communication interface, as long as you can somehow
 * embed an XCP packet in its communication packets. For example RS485 packets with a
 * custom layout, or SPI, I2C, etc.
 *
 * To keep this example implementation simple, it skips the support of the checksum byte
 * that XCP on RS232 supports. You also don't need it, because USB packets already
 * incorporate a checksum. Refer to macro BOOT_COM_RS232_CS_TYPE in the OpenBLT
 * source code for more info regarding the checksum byte.
 *
 * To embed the XCP packet in USB CDC-ACM packets, an XCP packet length byte was added
 * at the start of the packet:
 *    ---------------------------------------------------------------------------
 *   | Len of XCP packet |       XCP packet data[0..(Len of XCP packet-1)]       |
 *    ---------------------------------------------------------------------------
 *
 * This is needed in this case, because in MicroBoot (or BootCommander) we plan to
 * configure firmware updates via RS232. Therefore, we need to use the same layout used
 * for XCP packets on RS232.
 *
 * Because of this extra length byte at the start of a USB CDC-ACM packet, the
 * configuration macros BOOT_COM_CUSTOM_TX_MAX_DATA and BOOT_COM_CUSTOM_RX_MAX_DATA were
 * both set to 63. 63 bytes for the XCP packet plus the extra 1 for the length, makes 64
 * bytes in total, which is the size of the USB endpoint.
 */


/****************************************************************************************
* Configuration check
****************************************************************************************/
/* The TinyUSB port was modified to support polling mode operation, needed by the
 * bootloader. Verify that polling mode was actually enabled in the configuration header.
 */
#if (CFG_TUSB_POLLING_ENABLED <= 0)
#error "CFG_TUSB_POLLING_ENABLED must be > 0"
#endif


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static blt_bool ComCustomReceiveByte(blt_int8u *data);


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief USB CDC handle. */
static PCD_HandleTypeDef usbCdcHandle;


/************************************************************************************//**
** \brief     Initializes the custom communication interface.
** \return    none.
**
****************************************************************************************/
void ComCustomInitHook(void)
{
  /* Make sure the USB power and clock are enabled and that the GPIO pins are configured.
   * note that this is only actually used for CubeMX generated projects. These projects
   * should be configured to not call MX_USB_PCD_Init() to do this. This is because
   * MX_USB_PCD_Init() relies on a running SysTick, which is not the case at the point
   * where the CubeMX generate code calls MX_USB_PCD_Init(). For non CubeMX generated
   * projects, the call to HAL_PCD_MspInit() simply goes to the empty "__weak" function.
   */
  usbCdcHandle.Instance = USB;
  HAL_PCD_MspInit(&usbCdcHandle);

  /* initialize the TinyUSB device stack on the configured roothub port */
  tud_init(BOARD_TUD_RHPORT);

  /* Extend the time that the backdoor is open in case the default timed backdoor
   * mechanism is used. For a USB-CDC communication interface, the backdoor needs to stay
   * open long enough for the USB device to enumerate on the host PC. Therefore the
   * default backdoor open time needs to be extended. Note that this might not be long
   * enough for a first time USB driver install on the host PC. In this case the
   * bootloader should be started with the backup backdoor that uses, for example, a
   * digital input to force the bootloader to stay active. This can be implemented in
   * CpuUserProgramStartHook(). Feel free to shorten/lengthen this time for finetuning.
   */
#if (BOOT_BACKDOOR_HOOKS_ENABLE == 0)
  if (BackDoorGetExtension() < 2000U)
  {
    BackDoorSetExtension(2000U);
  }
#endif /* BOOT_BACKDOOR_HOOKS_ENABLE == 0 */
} /*** end of ComCustomInitHook ***/


/************************************************************************************//**
** \brief     Releases the custom communication interface.
** \return    none.
**
****************************************************************************************/
void ComCustomFreeHook(void)
{
  /* Disconnect the TinyUSB device stack.*/
  tud_disconnect();

  /* disable the USB power and clock and undo the the GPIO pins are configuration. note
   * that this is only actually used for CubeMX generated projects. for non CubeMX
   * generated projects, the call to HAL_PCD_MspDeInit() simply goes to the empty
   * "__weak" function.
   */
  usbCdcHandle.Instance = USB;
  HAL_PCD_MspDeInit(&usbCdcHandle);
} /*** end of ComCustomFreeHook ***/


/************************************************************************************//**
** \brief     Transmits a packet formatted for the communication interface.
** \param     data Pointer to byte array with data that it to be transmitted.
** \param     len  Number of bytes that are to be transmitted.
** \return    none.
**
****************************************************************************************/
void ComCustomTransmitPacketHook(blt_int8u *data, blt_int8u len)
{
  /* Verify validity of the len-paramenter. */
  ASSERT_RT(len <= BOOT_COM_CUSTOM_TX_MAX_DATA);

  /* First transmit the length of the packet. */
  tud_cdc_n_write(0, &len, 1);
  /* Next transmit the actual packet bytes. */
  tud_cdc_n_write(0, data, len);
  /* Make sure the transmission starts, even if the endpoint buffer is not yet full. */
  tud_cdc_n_write_flush(0);
} /*** end of ComCustomTransmitPacketHook ***/


/************************************************************************************//**
** \brief     Receives a communication interface packet if one is present.
** \param     data Pointer to byte array where the data is to be stored.
** \param     len Pointer where the length of the packet is to be stored.
** \return    BLT_TRUE if a packet was received, BLT_FALSE otherwise.
**
****************************************************************************************/
blt_bool ComCustomReceivePacketHook(blt_int8u *data, blt_int8u *len)
{
  blt_bool result = BLT_FALSE;
  static blt_int8u xcpCtoReqPacket[BOOT_COM_CUSTOM_RX_MAX_DATA+1];  /* one extra for length */
  static blt_int8u xcpCtoRxLength;
  static blt_bool  xcpCtoRxInProgress = BLT_FALSE;

  /* Poll for USB interrupt flags to process USB releated event and run the USB device
   * stack task.
   */
  tud_int_handler(BOARD_TUD_RHPORT);
  tud_task();

  /* Start of cto packet received? */
  if (xcpCtoRxInProgress == BLT_FALSE)
  {
    /* Store the message length when received. */
    if (ComCustomReceiveByte(&xcpCtoReqPacket[0]) == BLT_TRUE)
    {
      if ( (xcpCtoReqPacket[0] > 0) &&
           (xcpCtoReqPacket[0] <= BOOT_COM_CUSTOM_RX_MAX_DATA) )
      {
        /* Indicate that a cto packet is being received. */
        xcpCtoRxInProgress = BLT_TRUE;
        /* reset packet data count */
        xcpCtoRxLength = 0;
      }
    }
  }
  else
  {
    /* Store the next packet byte. */
    if (ComCustomReceiveByte(&xcpCtoReqPacket[xcpCtoRxLength+1]) == BLT_TRUE)
    {
      /* Increment the packet data count. */
      xcpCtoRxLength++;

      /* Check to see if the entire packet was received. */
      if (xcpCtoRxLength == xcpCtoReqPacket[0])
      {
        /* Copy the packet data. */
        CpuMemCopy((blt_int32u)data, (blt_int32u)&xcpCtoReqPacket[1], xcpCtoRxLength);
        /* Done with cto packet reception. */
        xcpCtoRxInProgress = BLT_FALSE;
        /* Set the packet length. */
        *len = xcpCtoRxLength;
        /* Packet reception complete. Update the result. */
        result = BLT_TRUE;
      }
    }
  }

  /* Give the result back to the caller. */
  return result;
} /*** end of ComCustomReceivePacketHook ***/


/************************************************************************************//**
** \brief     Receives a communication interface byte if one is present.
** \param     data Pointer to byte where the data is to be stored.
** \return    BLT_TRUE if a byte was received, BLT_FALSE otherwise.
**
****************************************************************************************/
static blt_bool ComCustomReceiveByte(blt_int8u *data)
{
  blt_bool result = BLT_FALSE;
  blt_int32u count;

  /* USB received data available? */
  if (tud_cdc_n_available(0))
  {
    /* Read the next byte from the internal USB reception buffer. */
    count = tud_cdc_n_read(0, data, 1);
    /* Check read result. */
    if (count == 1)
    {
      result = BLT_TRUE;
    }
  }
  /* Give the result back to the caller. */
  return result;
} /*** end of ComCustomReceiveByte ***/
#endif /* BOOT_COM_CUSTOM_ENABLE > 0 */


/*********************************** end of com_custom.c *******************************/
