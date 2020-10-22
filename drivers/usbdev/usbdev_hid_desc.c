/****************************************************************************
 * drivers/usbdev/cdcacm_desc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/hid.h>
#include <nuttx/usb/usbdev_trace.h>

#include "usbdev_hid.h"

// TODO move
#define CONFIG_USBHID_STR_LANGUAGE 0x0409 /* String language en-us */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB descriptor templates these will be copied and modified **************/
/* Device Descriptor.  If the USB serial device is configured as part of
 * composite device, then the device descriptor will be provided by the
 * composite device logic.
 */

#ifndef CONFIG_USBHID_COMPOSITE
static const struct usb_devdesc_s g_devdesc =
{
  USB_SIZEOF_DEVDESC,                           /* len */
  USB_DESC_TYPE_DEVICE,                         /* type */
  {                                             /* usb */
    LSBYTE(0x0110),
    MSBYTE(0x0110)
  },
  USB_CLASS_PER_INTERFACE,                      /* class */
  USBHID_SUBCLASS_NONE,                         /* subclass */
  USBHID_PROTOCOL_NONE,                         /* protocol */
  64,                   /* maxpacketsize */
  {
    LSBYTE(CONFIG_USBHID_VENDORID),             /* vendor */
    MSBYTE(CONFIG_USBHID_VENDORID)
  },
  {
    LSBYTE(CONFIG_USBHID_PRODUCTID),            /* product */
    MSBYTE(CONFIG_USBHID_PRODUCTID)
  },
  {
    LSBYTE(CONFIG_USBHID_VERSIONNO),            /* device */
    MSBYTE(CONFIG_USBHID_VERSIONNO)
  },
  USBHID_MANUFACTURERSTRID,                     /* imfgr */
  USBHID_PRODUCTSTRID,                          /* iproduct */
  USBHID_SERIALSTRID,                           /* serno */
  USBHID_NCONFIGS                               /* nconfigs */
};
#endif

#if !defined(CONFIG_USBHID_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
#error aaaaa
static const struct usb_qualdesc_s g_qualdesc =
{
  USB_SIZEOF_QUALDESC,                          /* len */
  USB_DESC_TYPE_DEVICEQUALIFIER,                /* type */
  {                                             /* usb */
     LSBYTE(0x0200),
     MSBYTE(0x0200)
  },
  USB_CLASS_VENDOR_SPEC,                        /* class */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_USBHID_EP0MAXPACKET,                   /* mxpacketsize */
  USBHID_NCONFIGS,                              /* nconfigs */
  0,                                            /* reserved */
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhid_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

int usbhid_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc)
{
  _err("entry %d\n", id);
#if !defined(CONFIG_USBHID_COMPOSITE) || defined(CONFIG_USBHID_NOTIFSTR)

  const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
#ifndef CONFIG_USBHID_COMPOSITE
    case 0:
      {
        /* Descriptor 0 is the language id */

        strdesc->len     = 4;
        strdesc->type    = USB_DESC_TYPE_STRING;
        strdesc->data[0] = LSBYTE(CONFIG_USBHID_STR_LANGUAGE);
        strdesc->data[1] = MSBYTE(CONFIG_USBHID_STR_LANGUAGE);
        return 4;
      }

    case USBHID_MANUFACTURERSTRID:
      str = CONFIG_USBHID_VENDORSTR;
      break;

    case USBHID_PRODUCTSTRID:
      str = CONFIG_USBHID_PRODUCTSTR;
      break;

#if 0
    case USBHID_SERIALSTRID:
      str = CONFIG_USBHID_SERIALSTR;
      break;

    case USBHID_CONFIGSTRID:
      str = CONFIG_USBHID_CONFIGSTR;
      break;
#endif
#endif

#ifdef CONFIG_USBHID_NOTIFSTR
    case USBHID_NOTIFSTRID:
      str = CONFIG_USBHID_NOTIFSTR;
      break;
#endif

    default:
      return -EINVAL;
    }

   /* The string is utf16-le.  The poor man's utf-8 to utf16-le
    * conversion below will only handle 7-bit en-us ascii
    */

   len = strlen(str);
   if (len > (USBHID_MAXSTRLEN / 2))
     {
       len = (USBHID_MAXSTRLEN / 2);
     }

   for (i = 0, ndata = 0; i < len; i++, ndata += 2)
     {
       strdesc->data[ndata]   = str[i];
       strdesc->data[ndata+1] = 0;
     }

   strdesc->len  = ndata+2;
   strdesc->type = USB_DESC_TYPE_STRING;
   return strdesc->len;
#else
   return -EINVAL;
#endif
}

/****************************************************************************
 * Name: usbhid_getdevdesc
 *
 * Description:
 *   Return a pointer to the raw device descriptor
 *
 ****************************************************************************/

#ifndef CONFIG_USBHID_COMPOSITE
FAR const struct usb_devdesc_s *usbhid_getdevdesc(void)
{
  return &g_devdesc;
}
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
                       bool hispeed)
{
#ifndef CONFIG_USBDEV_DUALSPEED
  UNUSED(hispeed);
#endif

  _err("use EP 0x%x\n", USBHID_MKEPINTIN(devinfo));

  epdesc->len  = USB_SIZEOF_EPDESC;            /* Descriptor length */
  epdesc->type = USB_DESC_TYPE_ENDPOINT;       /* Descriptor type */
  epdesc->addr = USBHID_MKEPINTIN(devinfo);    /* Endpoint address */
  epdesc->attr = USBHID_EPINTIN_ATTR;          /* Endpoint attributes */

#ifdef CONFIG_USBDEV_DUALSPEED
  if (hispeed)
    {
      /* Maximum packet size (high speed) */

      epdesc->mxpacketsize[0] = LSBYTE(CONFIG_USBHID_EPINTIN_HSSIZE);
      epdesc->mxpacketsize[1] = MSBYTE(CONFIG_USBHID_EPINTIN_HSSIZE);
    }
  else
#endif
    {
      /* Maximum packet size (full speed) */

      epdesc->mxpacketsize[0] = LSBYTE(CONFIG_USBHID_EPINTIN_FSSIZE);
      epdesc->mxpacketsize[1] = MSBYTE(CONFIG_USBHID_EPINTIN_FSSIZE);
    }

  epdesc->interval = 10;                       /* Interval */

  return sizeof(struct usb_epdesc_s);
}

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
                         uint8_t speed, uint8_t type)
#else
int16_t usbhid_mkcfgdesc(FAR uint8_t *buf,
                         FAR struct usbdev_devinfo_s *devinfo)
#endif
{
  int ret;
  int length = 0;
  bool hispeed = false;

#ifdef CONFIG_USBDEV_DUALSPEED
  hispeed = (speed == USB_SPEED_HIGH);

  /* Check for switches between high and full speed */

  if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG)
    {
      hispeed = !hispeed;
    }
#endif

  /* Fill in all descriptors directly to the buf */

  /* Configuration Descriptor.  If the serial device is used in as part
   * or a composite device, then the configuration descriptor is
   * provided by the composite device logic.
   */

#if !defined(CONFIG_USBHID_COMPOSITE)
  if (buf != NULL)
    {
      /* Configuration descriptor.  If the USB serial device is configured as part of
       * composite device, then the configuration descriptor will be provided by the
       * composite device logic.
       */

      FAR struct usb_cfgdesc_s *dest = (FAR struct usb_cfgdesc_s *)buf;

      /* Let's calculate the size... */

#ifdef CONFIG_USBDEV_DUALSPEED
      int16_t size = usbhid_mkcfgdesc(NULL, NULL, speed, type);
#else
      int16_t size = usbhid_mkcfgdesc(NULL, NULL);
#endif
      // uinfo("cfg size %d\n", size);

      dest->len         = USB_SIZEOF_CFGDESC;                /* Descriptor length */
      dest->type        = USB_DESC_TYPE_CONFIG;              /* Descriptor type */
      dest->totallen[0] = LSBYTE(size);                      /* LS Total length */
      dest->totallen[1] = MSBYTE(size);                      /* MS Total length */
      dest->ninterfaces = USBHID_NINTERFACES;                /* Number of interfaces */
      dest->cfgvalue    = USBHID_CONFIGID;                   /* Configuration value */
      dest->icfg        = USBHID_CONFIGSTRID;                /* Configuration */
      dest->attr        = USB_CONFIG_ATTR_ONE |              /* Attributes */
                          USBHID_SELFPOWERED |
                          USBHID_REMOTEWAKEUP;
      dest->mxpower     = (CONFIG_USBDEV_MAXPOWER + 1) / 2;  /* Max power (mA/2) */

      buf += sizeof(struct usb_cfgdesc_s);
    }

  length += sizeof(struct usb_cfgdesc_s);

  /* If the serial device is part of a composite device, then it should
   * begin with an interface association descriptor (IAD) because the
   * HID device consists of more than one interface. The IAD associates
   * the two HID interfaces with the same HID device.
   */

#elif defined(CONFIG_COMPOSITE_IAD)

#if 0
  /* Interface association descriptor */
  #error aaa
  if (buf != NULL)
    {
      FAR struct usb_iaddesc_s *dest = (FAR struct usb_iaddesc_s *)buf;

      dest->len       = USB_SIZEOF_IADDESC;                  /* Descriptor length */
      dest->type      = USB_DESC_TYPE_INTERFACEASSOCIATION;  /* Descriptor type */
      dest->firstif   = devinfo->ifnobase;                   /* Number of first interface of the function */
      dest->nifs      = devinfo->ninterfaces;                /* Number of interfaces associated with the function */
      dest->classid   = USB_CLASS_CDC;                       /* Class code */
      dest->subclass  = CDC_SUBCLASS_ACM;                    /* Sub-class code */
      dest->protocol  = CDC_PROTO_NONE;                      /* Protocol code */
      dest->ifunction = 0;                                   /* Index to string identifying the function */

      buf += sizeof(struct usb_iaddesc_s);
    }

  length += sizeof(struct usb_iaddesc_s);
#endif

#endif

  /* Notification interface */

  if (buf != NULL)
    {
      FAR struct usb_ifdesc_s *dest = (FAR struct usb_ifdesc_s *)buf;

      dest->len      = USB_SIZEOF_IFDESC;                    /* Descriptor length */
      dest->type     = USB_DESC_TYPE_INTERFACE;              /* Descriptor type */
      dest->ifno     = devinfo->ifnobase;                    /* Interface number */
      dest->alt      = USBHID_NOTALTIFID;                    /* Alternate setting */
      dest->neps     = 1;                                    /* Number of endpoints */
      dest->classid  = USB_CLASS_HID;                        /* Interface class */
      dest->subclass = USBHID_SUBCLASS_BOOTIF;               /* Interface sub-class */
      dest->protocol = USBHID_PROTOCOL_MOUSE;                /* Interface protocol */
#ifdef CONFIG_USBHID_NOTIFSTR
      dest->iif      = devinfo->strbase + USBHID_NOTIFSTRID; /* iInterface */
#else
      dest->iif      = 0;                                    /* iInterface */
#endif

      buf += sizeof(struct usb_ifdesc_s);
    }

  length += sizeof(struct usb_ifdesc_s);

  /* HID Class-Specific Descriptor */

  if (buf != NULL)
    {
      ret = usbhid_mkhiddesc((FAR struct usbhid_descriptor_s *)buf);
      buf += ret;
    }
  else
    {
      ret = 9;
    }

  length += ret; // sizeof(struct usbhid_descriptor_s);

#if 0
    0x09,//sizeof(USB_HID_DSC)+3,    // Size of this descriptor in bytes RRoj hack
    DSC_HID,                // HID descriptor type
    DESC_CONFIG_WORD(0x0111),                 // HID Spec Release Number in BCD format (1.11)
    0x00,                   // Country Code (0x00 for Not supported)
    HID_NUM_OF_DSC,         // Number of class descriptors, see usbcfg.h
    DSC_RPT,                // Report descriptor type
    DESC_CONFIG_WORD(50),   //sizeof(hid_rpt01),      // Size of the report descriptor
#endif


#if 0
  /* Header functional descriptor */

  if (buf != NULL)
    {
      FAR struct cdc_hdr_funcdesc_s *dest = (FAR struct cdc_hdr_funcdesc_s *)buf;

      dest->size    = SIZEOF_HDR_FUNCDESC;             /* Descriptor length */
      dest->type    = USB_DESC_TYPE_CSINTERFACE;       /* Descriptor type */
      dest->subtype = CDC_DSUBTYPE_HDR;                /* Descriptor sub-type */
      dest->cdc[0]  = LSBYTE(CONFIG_USBHID_VERSIONNO); /* CDC release number in BCD */
      dest->cdc[1]  = MSBYTE(CONFIG_USBHID_VERSIONNO);

      buf += sizeof(struct cdc_hdr_funcdesc_s);
    }

  length += sizeof(struct cdc_hdr_funcdesc_s);

  /* ACM functional descriptor */

  if (buf != NULL)
    {
      FAR struct cdc_acm_funcdesc_s *dest = (FAR struct cdc_acm_funcdesc_s *)buf;

      dest->size    = SIZEOF_ACM_FUNCDESC;                   /* Descriptor length */
      dest->type    = USB_DESC_TYPE_CSINTERFACE;             /* Descriptor type */
      dest->subtype = CDC_DSUBTYPE_ACM;                      /* Descriptor sub-type */
      dest->caps    = 0x06;                                  /* Bit encoded capabilities */

      buf += sizeof(struct cdc_acm_funcdesc_s);
    }

  length += sizeof(struct cdc_acm_funcdesc_s);

  /* Call Management functional descriptor */

  if (buf != NULL)
    {
      FAR struct cdc_callmgmt_funcdesc_s *dest = (FAR struct cdc_callmgmt_funcdesc_s *)buf;

      dest->size    = SIZEOF_CALLMGMT_FUNCDESC;               /* Descriptor length */
      dest->type    = USB_DESC_TYPE_CSINTERFACE;              /* Descriptor type */
      dest->subtype = CDC_DSUBTYPE_CALLMGMT;                  /* Descriptor sub-type */
      dest->caps    = 3;                                      /* Bit encoded capabilities */
      dest->ifno    = devinfo->ifnobase + 1;                  /* Interface number of Data Class interface */

      buf += sizeof(struct cdc_callmgmt_funcdesc_s);
    }

  length += sizeof(struct cdc_callmgmt_funcdesc_s);
#endif

  /* Interrupt IN endpoint descriptor */

  if (buf != NULL)
    {
      usbhid_copy_epdesc((struct usb_epdesc_s *)buf, devinfo, hispeed);

      buf += USB_SIZEOF_EPDESC;
    }

  length += USB_SIZEOF_EPDESC;

  return length;
}

/****************************************************************************
 * Name: usbhid_getqualdesc
 *
 * Description:
 *   Return a pointer to the raw qual descriptor
 *
 ****************************************************************************/

#if !defined(CONFIG_USBHID_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
FAR const struct usb_qualdesc_s *usbhid_getqualdesc(void)
{
  return &g_qualdesc;
}
#endif

int usbhid_mkhiddesc(struct usbhid_descriptor_s *hiddesc)
{
  /* HID Class-Specific Descriptor */

  hiddesc->len        = 9;        /* Size of the HID descriptor */
  hiddesc->type       = USBHID_DESCTYPE_HID;       /* HID descriptor type */
  hiddesc->hid[0]     = LSBYTE(0x0110);            /* HID class specification release */
  hiddesc->hid[1]     = MSBYTE(0x0110);
  hiddesc->country    = USBHID_COUNTRY_NONE;       /* Country code */
  hiddesc->ndesc      = 1;                         /* Number of descriptors (>=1) */
  hiddesc->classdesc  = USBHID_DESCTYPE_REPORT;    /* Class descriptor type (See 7.1) */
  hiddesc->desclen[0] = LSBYTE(HID_RPT01_SIZE);    /* Size of the report descriptor */
  hiddesc->desclen[1] = MSBYTE(HID_RPT01_SIZE);
      // dest->optdesc;    /* Type of optional descriptor */
      // dest->optlen[2];  /* Size of the optional descriptor */

  return 9; // sizeof(struct usbhid_descriptor_s);
}