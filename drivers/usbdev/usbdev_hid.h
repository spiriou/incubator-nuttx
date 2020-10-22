/****************************************************************************
 * drivers/usbdev/usbdev_hid.h
 *
 *   Copyright (C) 2011-2012, 2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __DRIVERS_USBDEV_USBHID_H
#define __DRIVERS_USBDEV_USBHID_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/hid.h>
#include <nuttx/usb/usbhid.h>
#include <nuttx/usb/usbdev_trace.h>

#define HID_RPT01_SIZE 52

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* If the serial device is configured as part of a composite device than both
 * CONFIG_USBDEV_COMPOSITE and CONFIG_USBHID_COMPOSITE must be defined.
 */

#ifndef CONFIG_USBDEV_COMPOSITE
#  undef CONFIG_USBHID_COMPOSITE
#endif

#if defined(CONFIG_USBHID_COMPOSITE) && !defined(CONFIG_USBHID_STRBASE)
#  define CONFIG_USBHID_STRBASE    (4)
#endif

#if defined(CONFIG_USBHID_COMPOSITE) && !defined(CONFIG_COMPOSITE_IAD)
#  warning "CONFIG_COMPOSITE_IAD may be needed"
#endif

/* Packet and request buffer sizes */

#ifndef CONFIG_USBHID_COMPOSITE
#  ifndef CONFIG_USBHID_EP0MAXPACKET
#    define CONFIG_USBHID_EP0MAXPACKET 8
#  endif
#endif

/* Descriptors **************************************************************/
/* These settings are not modifiable via the NuttX configuration */

#define CONFIG_USBHID_VERSIONNO    0x0001   /* HID version number 20.00 (BCD) */
#define USBHID_CONFIGIDNONE        (0)      /* Config ID means to return to address mode */

/* Interface IDs:
 *
 * USBHID_NINTERFACES              One interface
 * USBHID_NOTIFID                  ID of the notifier interface
 * USBHID_NOTALTIFID               No alternate for the notifier interface
 */

#define USBHID_NOTALTIFID          (0)

/* Buffer big enough for any of our descriptors (the config descriptor is the
 * biggest).
 */

#define USBHID_MXDESCLEN           (64)
#define USBHID_MAXSTRLEN           (USBHID_MXDESCLEN-2)

/* Descriptor strings.  If there serial device is part of a composite device
 * then the manufacturer, product, and serial number strings will be provided
 * by the composite logic.
 */

#ifndef CONFIG_USBHID_COMPOSITE
#  define USBHID_MANUFACTURERSTRID (1)
#  define USBHID_PRODUCTSTRID      (2)
#  define USBHID_SERIALSTRID       (0)
#  define USBHID_CONFIGSTRID       (0)

#  define USBHID_LASTBASESTRID     (2)
#  define USBHID_STRBASE           (0)
#else
#  define USBHID_STRBASE           CONFIG_USBHID_STRBASE
#  define USBHID_LASTBASESTRID     CONFIG_USBHID_STRBASE
#endif

/* These string IDs only exist if a user-defined string is provided */

#ifdef CONFIG_USBHID_NOTIFSTR
#  define USBHID_NOTIFSTRID        (USBHID_LASTBASESTRID+1)
#else
#  define USBHID_NOTIFSTRID        USBHID_LASTBASESTRID
#endif

#define USBHID_LASTSTRID           USBHID_NOTIFSTRID
#define USBHID_NSTRIDS             (USBHID_LASTSTRID - USBHID_STRBASE)

/* Endpoint configuration ****************************************************/

#define USBHID_MKEPINTIN(desc)     (USB_DIR_IN | (desc)->epno[USBHID_EP_INTIN_IDX])
#define USBHID_EPINTIN_ATTR        (USB_EP_ATTR_XFER_INT)

/* Device driver definitions ************************************************/
/* A HID device is specific by a minor number in the range of 0-255.
 * This maps to a character device at /dev/hidx, x=0..255.
 */

#define USBHID_DEVNAME_FORMAT      "/dev/hid%d"
#define USBHID_DEVNAME_SIZE        16

/* Misc Macros **************************************************************/
/* MIN/MAX macros */

#ifndef MIN
#  define MIN(a,b) ((a)<(b)?(a):(b))
#endif

#ifndef MAX
#  define MAX(a,b) ((a)>(b)?(a):(b))
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum usbhid_epdesc_e
{
  USBHID_EPINTIN = 0,  /* Interrupt IN endpoint descriptor */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: usbhid_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

int usbhid_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc);

/****************************************************************************
 * Name: usbhid_mkhiddesc
 *
 * Description:
 *   Construct a HID Class Specific descriptor
 *
 ****************************************************************************/

int usbhid_mkhiddesc(struct usbhid_descriptor_s *hiddesc);

/****************************************************************************
 * Name: usbhid_getdevdesc
 *
 * Description:
 *   Return a pointer to the raw device descriptor
 *
 ****************************************************************************/

#ifndef CONFIG_USBHID_COMPOSITE
FAR const struct usb_devdesc_s *usbhid_getdevdesc(void);
#endif

/****************************************************************************
 * Name: usbhid_copy_epdesc
 *
 * Description:
 *   Copies the requested Endpoint Description into the buffer given.
 *   Returns the number of Bytes filled in (sizeof(struct usb_epdesc_s)).
 *
 ****************************************************************************/

int usbhid_copy_epdesc(FAR struct usb_epdesc_s *epdesc,
                       FAR struct usbdev_devinfo_s *devinfo,
                       bool hispeed);

/****************************************************************************
 * Name: usbhid_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
int16_t usbhid_mkcfgdesc(FAR uint8_t *buf,
                         FAR struct usbdev_devinfo_s *devinfo,
                         uint8_t speed, uint8_t type);
#else
int16_t usbhid_mkcfgdesc(FAR uint8_t *buf,
                         FAR struct usbdev_devinfo_s *devinfo);
#endif

/****************************************************************************
 * Name: usbhid_getqualdesc
 *
 * Description:
 *   Return a pointer to the raw qual descriptor
 *
 ****************************************************************************/

#if !defined(CONFIG_USBHID_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
FAR const struct usb_qualdesc_s *usbhid_getqualdesc(void);
#endif

#endif /* __DRIVERS_USBDEV_USBHID_H */
