/****************************************************************************
 * boards/arm/stm32/photon/src/stm32_composite.c
 *
 *   Copyright (C) 2009, 2011, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Configure and register the STM32 MMC/SD SDIO block driver.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/adb.h>
#include <nuttx/usb/usbhid.h>
#include <nuttx/usb/composite.h>

#include "stm32.h"

// #if defined(CONFIG_BOARDCTL_USBDEVCTRL) && defined(CONFIG_USBDEV_COMPOSITE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_composite0_connect
 *
 * Description:
 *   Connect the USB composite device on the specified USB device port for
 *   configuration 0.
 *
 * Input Parameters:
 *   port     - The USB device port.
 *
 * Returned Value:
 *   A non-NULL handle value is returned on success.  NULL is returned on
 *   any failure.
 *
 ****************************************************************************/

static FAR void *board_composite0_connect(int port)
{
  /* Here we are composing the configuration of the usb composite device.
   *
   * The standard is to use one CDC/ACM and one USB mass storage device.
   */

  struct composite_devdesc_s dev[2];
  int ifnobase = 0;
  int strbase  = COMPOSITE_NSTRIDS-1;

  int dev_idx = 0;

#if 1
  /* Configure USB HID device device */
  /* Ask the USB HID driver to fill in the constants we didn't
   * know here.
   */

  usbhid_get_composite_devdesc(&dev[dev_idx]);

  /* Interfaces */

  dev[dev_idx].devinfo.ifnobase = ifnobase;               /* Offset to Interface-IDs */
  dev[dev_idx].minor = 0;                                 /* The minor interface number */

  /* Strings */

  dev[dev_idx].devinfo.strbase = strbase;   /* Offset to String Numbers */

  /* Endpoints */

  dev[dev_idx].devinfo.epno[USBHID_EP_INTIN_IDX]  = 3;

  /* Count up the base numbers */

  ifnobase += dev[dev_idx].devinfo.ninterfaces;
  strbase  += dev[dev_idx].devinfo.nstrings;

  dev_idx += 1;
#endif
#if 1
  /* Configure the ADB USB device */

  /* Ask the adb driver to fill in the constants we didn't
   * know here.
   */

  usbdev_adb_get_composite_devdesc(&dev[dev_idx]);

  /* Interfaces */

  dev[dev_idx].devinfo.ifnobase = ifnobase;             /* Offset to Interface-IDs */
  dev[dev_idx].minor = 0;                               /* The minor interface number */

  /* Strings */

  dev[dev_idx].devinfo.strbase = strbase;               /* Offset to String Numbers */

  /* Endpoints */

  dev[dev_idx].devinfo.epno[USBADB_EP_BULKIN_IDX]  = 1;
  dev[dev_idx].devinfo.epno[USBADB_EP_BULKOUT_IDX] = 2;

  /* Count up the base numbers */

  ifnobase += dev[dev_idx].devinfo.ninterfaces;
  strbase  += dev[dev_idx].devinfo.nstrings;

  dev_idx += 1;
#endif

  return composite_initialize(dev_idx, dev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_composite_initialize
 *
 * Description:
 *   Perform architecture specific initialization of a composite USB device.
 *
 ****************************************************************************/

int board_composite_initialize(int port)
{
   return OK;
}

/****************************************************************************
 * Name:  board_composite_connect
 *
 * Description:
 *   Connect the USB composite device on the specified USB device port using
 *   the specified configuration.  The interpretation of the configid is
 *   board specific.
 *
 * Input Parameters:
 *   port     - The USB device port.
 *   configid - The USB composite configuration
 *
 * Returned Value:
 *   A non-NULL handle value is returned on success.  NULL is returned on
 *   any failure.
 *
 ****************************************************************************/

FAR void *board_composite_connect(int port, int configid)
{
  if (configid == 0)
    {
      return board_composite0_connect(port);
    }

  return NULL;
}

// #endif /* CONFIG_BOARDCTL_USBDEVCTRL && CONFIG_USBDEV_COMPOSITE */
