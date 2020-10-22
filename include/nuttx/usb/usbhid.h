/****************************************************************************
 * include/nuttx/usb/usbhid.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_USB_USBHID_H
#define __INCLUDE_NUTTX_USB_USBHID_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/usb/usb.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* CONFIG_USBHID
 *   Enable compilation of the USB serial driver
 * CONFIG_USBHID_EP0MAXPACKET
 *   Endpoint 0 max packet size. Default 64.
 * CONFIG_USBHID_EPINTIN
 *   The logical 7-bit address of a hardware endpoint that supports
 *   interrupt IN operation.  Default 1.
 * CONFIG_USBHID_EPINTIN_FSSIZE
 *   Max package size for the interrupt IN endpoint if full speed mode.
 *   Default 64.
 * CONFIG_USBHID_EPINTIN_HSSIZE
 *   Max package size for the interrupt IN endpoint if high speed mode.
 *   Default 64.
 * CONFIG_USBHID_VENDORID and CONFIG_USBHID_VENDORSTR
 *   The vendor ID code/string.  Default 0x0525 and "NuttX"
 *   0x0525 is the Netchip vendor and should not be used in any
 *   products.  This default VID was selected for compatibility with
 *   the Linux HID default VID.
 * CONFIG_USBHID_PRODUCTID and CONFIG_USBHID_PRODUCTSTR
 *   The product ID code/string.
 */

/* Informations needed in usbdev_devinfo_s */

#define USBHID_NUM_EPS             (1)

#define USBHID_EP_INTIN_IDX        (0)

#define USBHID_NCONFIGS            (1)      /* Number of configurations supported */

/* Configuration descriptor values */

#define USBHID_CONFIGID            (1)      /* The only supported configuration ID */

#define USBHID_NINTERFACES         (1)      /* Number of interfaces in the configuration */

/* EP0 max packet size */

#ifndef CONFIG_USBHID_EP0MAXPACKET
#  define CONFIG_USBHID_EP0MAXPACKET 8
#endif

/* Endpoint number and size (in bytes) of the HID device-to-host (IN)
 * notification interrupt endpoint.
 */

#ifndef CONFIG_USBHID_COMPOSITE
#  ifndef CONFIG_USBHID_EPINTIN
#    define CONFIG_USBHID_EPINTIN 1
#  endif
#endif

#ifndef CONFIG_USBHID_EPINTIN_FSSIZE
#  define CONFIG_USBHID_EPINTIN_FSSIZE 4
#endif

#ifndef CONFIG_USBHID_EPINTIN_HSSIZE
#  define CONFIG_USBHID_EPINTIN_HSSIZE 4
#endif

/* Vendor and product IDs and strings.
 */

#ifndef CONFIG_USBHID_VENDORID
#  define CONFIG_USBHID_VENDORID  0x0f62
#endif

#ifndef CONFIG_USBHID_PRODUCTID
#  define CONFIG_USBHID_PRODUCTID 0x1001
#endif

#ifndef CONFIG_USBHID_VENDORSTR
#  define CONFIG_USBHID_VENDORSTR  "MyNuttX"
#endif

#ifndef CONFIG_USBHID_PRODUCTSTR
#  define CONFIG_USBHID_PRODUCTSTR "USB PS/2 Mouse"
#endif

#undef CONFIG_USBHID_SERIALSTR
#define CONFIG_USBHID_SERIALSTR "0"

#undef CONFIG_USBHID_CONFIGSTR
#define CONFIG_USBHID_CONFIGSTR "Bulk"

/* USB Controller */

#ifdef CONFIG_USBDEV_SELFPOWERED
#  define USBHID_SELFPOWERED USB_CONFIG_ATTR_SELFPOWER
#else
#  define USBHID_SELFPOWERED (0)
#endif

#ifdef CONFIG_USBDEV_REMOTEWAKEUP
#  define USBHID_REMOTEWAKEUP USB_CONFIG_ATTR_WAKEUP
#else
#  define USBHID_REMOTEWAKEUP (0)
#endif

#ifndef CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C"
{
#else
# define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: usbhid_classobject
 *
 * Description:
 *   Register USB serial port (and USB serial console if so configured) and
 *   return the class object.
 *
 * Input Parameters:
 *   minor - Device minor number.  E.g., minor 0 would correspond to
 *     /dev/hid0.
 *   classdev - The location to return the HID class' device
 *     instance.
 *
 * Returned Value:
 *   A pointer to the allocated class object (NULL on failure).
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_USBHID_COMPOSITE)
struct usbdev_devinfo_s;
struct usbdevclass_driver_s;
int usbhid_classobject(int minor, FAR struct usbdev_devinfo_s *devinfo,
                       FAR struct usbdevclass_driver_s **classdev);
#endif

/****************************************************************************
 * Name: usbhid_initialize
 *
 * Description:
 *   Register USB serial port (and USB serial console if so configured).
 *
 * Input Parameters:
 *   minor - Device minor number.  E.g., minor 0 would correspond to
 *     /dev/hid0.
 *   handle - An optional opaque reference to the HID class object that
 *     may subsequently be used with usbhid_uninitialize().
 *
 * Returned Value:
 *   Zero (OK) means that the driver was successfully registered.  On any
 *   failure, a negated errno value is retured.
 *
 ****************************************************************************/

#if !defined(CONFIG_USBDEV_COMPOSITE) || !defined(CONFIG_USBHID_COMPOSITE)
int usbhid_initialize(int minor, FAR void **handle);
#endif

/****************************************************************************
 * Name: usbhid_uninitialize
 *
 * Description:
 *   Un-initialize the USB storage class driver.  This function is used
 *   internally by the USB composite driver to uninitialized the HID
 *   driver.  This same interface is available (with an untyped input
 *   parameter) when the HID driver is used standalone.
 *
 * Input Parameters:
 *   There is one parameter, it differs in typing depending upon whether the
 *   HID driver is an internal part of a composite device, or a
 *   standalone USB driver:
 *
 *     classdev - The class object returned by usbhid_classobject()
 *     handle - The opaque handle representing the class object returned by
 *       a previous call to usbhid_initialize().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_USBHID_COMPOSITE)
void usbhid_uninitialize(FAR struct usbdevclass_driver_s *classdev);
#else
void usbhid_uninitialize(FAR void *handle);
#endif

/****************************************************************************
 * Name: usbhid_get_composite_devdesc
 *
 * Description:
 *   Helper function to fill in some constants into the composite
 *   configuration struct.
 *
 * Input Parameters:
 *     dev - Pointer to the configuration struct we should fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_USBHID_COMPOSITE)
struct composite_devdesc_s;
void usbhid_get_composite_devdesc(struct composite_devdesc_s *dev);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_USBHID_H */
