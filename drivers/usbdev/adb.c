/****************************************************************************
 * drivers/usbdev/adb.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

// #include <sys/types.h>
// #include <stdint.h>
// #include <stdbool.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/usb/adb.h>

// char device
#include <nuttx/fs/fs.h>

#ifdef CONFIG_USBADB_COMPOSITE
#  include <nuttx/usb/composite.h>
#  include "composite.h"
#endif

#define CONFIG_USBADB_EPBULK_SIZE 64

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USB Controller */

#ifdef CONFIG_USBDEV_SELFPOWERED
#  define USBADB_SELFPOWERED USB_CONFIG_ATTR_SELFPOWER
#else
#  define USBADB_SELFPOWERED (0)
#endif

#ifdef CONFIG_USBDEV_REMOTEWAKEUP
#  define USBADB_REMOTEWAKEUP USB_CONFIG_ATTR_WAKEUP
#else
#  define USBADB_REMOTEWAKEUP (0)
#endif

/* Buffer big enough for any of our descriptors (the config descriptor is the
 * biggest).
 */

#define USBADB_MXDESCLEN           (64)
#define USBADB_MAXSTRLEN           (USBADB_MXDESCLEN-2)

/* Device descriptor values */

#define USBADB_VERSIONNO           (0x0101) /* Device version number 1.1 (BCD) */

/* String language */

#define USBADB_STR_LANGUAGE        (0x0409) /* en-us */

/* Indexes for devinfo.epno[] array. Used for composite device configuration. */

#define USBADB_EP_BULKIN_IDX     (0)
#define USBADB_EP_BULKOUT_IDX    (1)

/* Descriptor strings.  If there serial device is part of a composite device
 * then the manufacturer, product, and serial number strings will be provided
 * by the composite logic.
 */

#ifndef CONFIG_USBADB_COMPOSITE
#  define USBADB_MANUFACTURERSTRID (1)
#  define USBADB_PRODUCTSTRID      (2)
#  define USBADB_SERIALSTRID       (3)
#  define USBADB_CONFIGSTRID       (4)
#  define USBADB_INTERFACESTRID    (5)

#  define USBADB_STRBASE           (0)
#else
#  define USBADB_INTERFACESTRID    ((CONFIG_USBADB_STRBASE)+1)
#  define USBADB_STRBASE           (CONFIG_USBADB_STRBASE)
#endif

#  define USBADB_LASTBASESTRID     (USBADB_INTERFACESTRID)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the internal state of the driver */

struct usbdev_adb_s
{
  FAR struct usbdev_s *usbdev;      /* usbdev driver pointer */
#ifdef CONFIG_USBADB_COMPOSITE
  struct usbdev_devinfo_s devinfo;
#endif

  FAR struct usbdev_req_s *ctrlreq; /* Pointer to preallocated control request */
};

struct adb_driver_s
{
  struct usbdevclass_driver_s drvr;
  struct usbdev_adb_s dev;
};

struct adb_cfgdesc_s
{
#ifndef CONFIG_USBADB_COMPOSITE
  struct usb_cfgdesc_s cfgdesc;        /* Configuration descriptor */
#endif
  struct usb_ifdesc_s  ifdesc;         /* ADB interface descriptor */
  struct usb_epdesc_s  epbulkoutdesc;  /* Bulk out interface descriptor */
  struct usb_epdesc_s  epbulkindesc;   /* Bulk in interface descriptor */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR struct usbdev_req_s *usbclass_allocreq(FAR struct usbdev_ep_s *ep,
                                                  uint16_t len);

/* USB class device *********************************************************/

static int     usbclass_bind(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static void    usbclass_unbind(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static int     usbclass_setup(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev,
                 FAR const struct usb_ctrlreq_s *ctrl, FAR uint8_t *dataout,
                 size_t outlen);
static void    usbclass_disconnect(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
#if 1 // def CONFIG_SERIAL_REMOVABLE
static void    usbclass_suspend(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static void    usbclass_resume(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
#endif

/* Char device Operations ***************************************************/

static int adb_char_open(FAR struct file *filep);
static int adb_char_close(FAR struct file *filep);

static ssize_t adb_char_read(FAR struct file *filep, FAR char *buffer,
                               size_t len);
static ssize_t adb_char_write(FAR struct file *filep,
                                FAR const char *buffer, size_t len);
static int adb_char_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
#ifdef CONFIG_EVENT_FD_POLL
static int adb_char_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB class device *********************************************************/

static const struct usbdevclass_driverops_s g_adb_driverops =
{
  usbclass_bind,       /* bind       */
  usbclass_unbind,     /* unbind     */
  usbclass_setup,      /* setup      */
  usbclass_disconnect, /* disconnect */
  usbclass_suspend,    /* suspend    */
  usbclass_resume      /* resume     */
};

/* Char device **************************************************************/

static const struct file_operations g_adb_fops =
{
  adb_char_open,  /* open */
  adb_char_close, /* close */
  adb_char_read,  /* read */
  adb_char_write, /* write */
  0,              /* seek */
  adb_char_ioctl  /* ioctl */
#ifdef CONFIG_USBADB_POLL
  , adb_char_poll /* poll */
#endif
};

/* USB descriptor ***********************************************************/

#define USB_ADB_DESC_TOTALLEN 32

#ifndef CONFIG_USBADB_COMPOSITE
static const struct usb_devdesc_s g_adb_devdesc =
{
  .len = USB_SIZEOF_DEVDESC,         /* Descriptor length */
  .type = USB_DESC_TYPE_DEVICE,      /* Descriptor type */
  .usb =                             /* USB version */
  {
    LSBYTE(0x0200),
    MSBYTE(0x0200)
  },
  .classid = 0,                      /* Device class */
  .subclass = 0,                     /* Device sub-class */
  .protocol = 0,                     /* Device protocol */
  .mxpacketsize = CONFIG_USBADB_EP0MAXPACKET, /* Max packet size (ep0) */
  .vendor =                          /* Vendor ID */
  {
    LSBYTE(CONFIG_USBADB_VENDORID),
    MSBYTE(CONFIG_USBADB_VENDORID)
  },
  .product =                         /* Product ID */
  { LSBYTE(CONFIG_USBADB_PRODUCTID),
    MSBYTE(CONFIG_USBADB_PRODUCTID)
  },
  .device =                          /* Device ID */
  { LSBYTE(USBADB_VERSIONNO),
    MSBYTE(USBADB_VERSIONNO)
  },
  .imfgr = USBADB_MANUFACTURERSTRID, /* Manufacturer */
  .iproduct = USBADB_PRODUCTSTRID,   /* Product */
  .serno = USBADB_SERIALSTRID,       /* Serial number */
  .nconfigs = 1                      /* Number of configurations */
};
#endif

static const struct adb_cfgdesc_s g_adb_cfgdesc =
{
#ifndef CONFIG_USBADB_COMPOSITE
  {
    .len          = USB_SIZEOF_CFGDESC, /* Descriptor length    */
    .type         = USB_DESC_TYPE_CONFIG, /* Descriptor type      */
    .totallen     =
    {
      LSBYTE(USB_ADB_DESC_TOTALLEN), /* LS Total length */
      MSBYTE(USB_ADB_DESC_TOTALLEN)  /* MS Total length */
    },
    .ninterfaces  = 1, /* Number of interfaces */
    .cfgvalue     = 1, /* Configuration value  */
    .icfg         = USBADB_CONFIGSTRID, /* Configuration        */
    .attr         = USB_CONFIG_ATTR_ONE |
                    USBADB_SELFPOWERED |
                    USBADB_REMOTEWAKEUP, /* Attributes           */
    .mxpower      = (CONFIG_USBDEV_MAXPOWER + 1) / 2 /* Max power (mA/2)     */
  },
#endif
  {
    .len            = USB_SIZEOF_IFDESC,
    .type           = USB_DESC_TYPE_INTERFACE,
    .ifno           = 0,
    .alt            = 0,
    .neps           = 2,
    .classid        = USB_CLASS_VENDOR_SPEC,
    .subclass       = 0x42,
    .protocol       = 0x01,
    .iif            = USBADB_INTERFACESTRID     /* ADB Interface */
  },
  {
    .len          = USB_SIZEOF_EPDESC,
    .type         = USB_DESC_TYPE_ENDPOINT,
    .addr         = USB_EPIN(CONFIG_USBADB_EPBULKIN),
    .attr         = USB_EP_ATTR_XFER_BULK | /* Endpoint attributes */
                    USB_EP_ATTR_NO_SYNC |
                    USB_EP_ATTR_USAGE_DATA,
#ifdef CONFIG_USBDEV_DUALSPEED
    .mxpacketsize =
    {
      LSBYTE(512), MSBYTE(512)
    },
    .interval     = 0
#else
    .mxpacketsize =
    {
      LSBYTE(64), MSBYTE(64)
    },
    .interval     = 1
#endif
  },
  {
    .len          = USB_SIZEOF_EPDESC,
    .type         = USB_DESC_TYPE_ENDPOINT,
    .addr         = USB_EPOUT(CONFIG_USBADB_EPBULKOUT),
    .attr         = USB_EP_ATTR_XFER_BULK | /* Endpoint attributes */
                    USB_EP_ATTR_NO_SYNC |
                    USB_EP_ATTR_USAGE_DATA,
#ifdef CONFIG_USBDEV_DUALSPEED
    .mxpacketsize =
    {
      LSBYTE(512), MSBYTE(512)
    },
    .interval     = 0
#else
    .mxpacketsize =
    {
      LSBYTE(64), MSBYTE(64)
    },
    .interval     = 1
#endif
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR struct usbdev_req_s *usbclass_allocreq(FAR struct usbdev_ep_s *ep,
                                                  uint16_t len)
{
  FAR struct usbdev_req_s *req;

  req = EP_ALLOCREQ(ep);
  if (req != NULL)
    {
      req->len = len;
      req->buf = EP_ALLOCBUFFER(ep, len);

      if (req->buf == NULL)
        {
          EP_FREEREQ(ep, req);
          req = NULL;
        }
    }

  return req;
}













#if 0


/****************************************************************************
 * Name: cdcacm_resetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

static void cdcacm_resetconfig(FAR struct cdcacm_dev_s *priv)
{
  /* Are we configured? */

  if (priv->config != CDCACM_CONFIGIDNONE)
    {
      /* Yes.. but not anymore */

      priv->config = CDCACM_CONFIGIDNONE;

      /* Inform the "upper half" driver that there is no (functional) USB
       * connection.
       */

#ifdef CONFIG_SERIAL_REMOVABLE
      uart_connected(&priv->serdev, false);
#endif

      /* Disable endpoints.  This should force completion of all pending
       * transfers.
       */

      EP_DISABLE(priv->epintin);
      EP_DISABLE(priv->epbulkin);
      EP_DISABLE(priv->epbulkout);
    }
}

/****************************************************************************
 * Name: cdcacm_epconfigure
 *
 * Description:
 *   Configure one endpoint.
 *
 ****************************************************************************/

static int cdcacm_epconfigure(FAR struct usbdev_ep_s *ep,
                              enum cdcacm_epdesc_e epid, bool last,
                              FAR struct usbdev_devinfo_s *devinfo,
                              bool hispeed)
{
  struct usb_epdesc_s epdesc;
  cdcacm_copy_epdesc(epid, &epdesc, devinfo, hispeed);
  return EP_CONFIGURE(ep, &epdesc, last);
}

/****************************************************************************
 * Name: cdcacm_setconfig
 *
 * Description:
 *   Set the device configuration by allocating and configuring endpoints and
 *   by allocating and queue read and write requests.
 *
 ****************************************************************************/

static int cdcacm_setconfig(FAR struct cdcacm_dev_s *priv, uint8_t config)
{
  FAR struct usbdev_req_s *req;
  int i;
  int ret = 0;

#ifdef CONFIG_DEBUG_FEATURES
  if (priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EINVAL;
    }
#endif

  if (config == priv->config)
    {
      /* Already configured -- Do nothing */

      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALREADYCONFIGURED), 0);
      return 0;
    }

  /* Discard the previous configuration data */

  cdcacm_resetconfig(priv);

  /* Was this a request to simply discard the current configuration? */

  if (config == CDCACM_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGNONE), 0);
      return 0;
    }

  /* We only accept one configuration */

  if (config != CDCACM_CONFIGID)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGIDBAD), 0);
      return -EINVAL;
    }

  /* Configure the IN interrupt endpoint */

#ifdef CONFIG_USBDEV_DUALSPEED
  if (priv->usbdev->speed == USB_SPEED_HIGH)
    {
      ret = cdcacm_epconfigure(priv->epintin, CDCACM_EPINTIN, false,
                               &priv->devinfo, true);
    }
  else
#endif
    {
      ret = cdcacm_epconfigure(priv->epintin, CDCACM_EPINTIN, false,
                               &priv->devinfo, false);
    }

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epintin->priv = priv;

  /* Configure the IN bulk endpoint */

#ifdef CONFIG_USBDEV_DUALSPEED
  if (priv->usbdev->speed == USB_SPEED_HIGH)
    {
      ret = cdcacm_epconfigure(priv->epbulkin, CDCACM_EPBULKIN, false,
                               &priv->devinfo, true);
    }
  else
#endif
    {
      ret = cdcacm_epconfigure(priv->epbulkin, CDCACM_EPBULKIN, false,
                               &priv->devinfo, false);
    }

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Configure the OUT bulk endpoint */

#ifdef CONFIG_USBDEV_DUALSPEED
  if (priv->usbdev->speed == USB_SPEED_HIGH)
    {
      ret = cdcacm_epconfigure(priv->epbulkout, CDCACM_EPBULKOUT, true,
                               &priv->devinfo, true);
    }
  else
#endif
    {
      ret = cdcacm_epconfigure(priv->epbulkout, CDCACM_EPBULKOUT, true,
                               &priv->devinfo, false);
    }

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkout->priv = priv;

  /* Queue read requests in the bulk OUT endpoint */

  DEBUGASSERT(priv->nrdq == 0);
  for (i = 0; i < CONFIG_CDCACM_NRDREQS; i++)
    {
      req           = priv->rdreqs[i].req;
      req->callback = cdcacm_rdcomplete;
      ret           = EP_SUBMIT(priv->epbulkout, req);
      if (ret != OK)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT),
                   (uint16_t)-ret);
          goto errout;
        }

      priv->nrdq++;
    }

  /* We are successfully configured */

  priv->config = config;

  /* Inform the "upper half" driver that we are "open for business" */

#ifdef CONFIG_SERIAL_REMOVABLE
  uart_connected(&priv->serdev, true);
#endif

  return OK;

errout:
  cdcacm_resetconfig(priv);
  return ret;
}




#endif



























static void usbclass_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                 FAR struct usbdev_req_s *req)
{
  _err("entry\n");
}

static int16_t usbclass_mkcfgdesc(FAR uint8_t *buf,
                                  FAR struct usbdev_devinfo_s *devinfo)
{
  // dest->ifdesc.ifno += devinfo->ifnobase;
  // dest->ifdesc.iif = devinfo->strbase;

  FAR struct adb_cfgdesc_s *dest = (FAR struct adb_cfgdesc_s *)buf;
  memcpy(dest, &g_adb_cfgdesc, sizeof(g_adb_cfgdesc));

#ifdef CONFIG_USBADB_COMPOSITE
  /* For composite device, apply possible offset to the interface numbers */

  dest->ifdesc.ifno = devinfo->ifnobase;
  dest->ifdesc.iif  = devinfo->strbase;
  dest->epbulkindesc.addr  = USB_EPIN(devinfo->epno[USBADB_EP_BULKIN_IDX]);
  dest->epbulkoutdesc.addr = USB_EPOUT(devinfo->epno[USBADB_EP_BULKOUT_IDX]);
#endif

  return sizeof(g_adb_cfgdesc);
}

static int usbclass_mkstrdesc(uint8_t id, FAR struct usb_strdesc_s *strdesc)
{
  FAR const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
#ifndef CONFIG_USBADB_COMPOSITE
    case 0:
      {
        /* Descriptor 0 is the language id */

        strdesc->len     = 4;
        strdesc->type    = USB_DESC_TYPE_STRING;
        strdesc->data[0] = LSBYTE(USBADB_STR_LANGUAGE);
        strdesc->data[1] = MSBYTE(USBADB_STR_LANGUAGE);
        return 4;
      }

    case USBADB_MANUFACTURERSTRID:
      str = CONFIG_USBADB_VENDORSTR;
      break;

    case USBADB_PRODUCTSTRID:
      str = CONFIG_USBADB_PRODUCTSTR;
      break;

    case USBADB_SERIALSTRID:
      str = CONFIG_USBADB_SERIALSTR;
      break;

    case USBADB_CONFIGSTRID:
      str = CONFIG_USBADB_CONFIGSTR;
      break;
#endif

    case USBADB_INTERFACESTRID:
      str = CONFIG_USBADB_INTERFACESTR;
      break;

    default:
      return -EINVAL;
    }

  /* The string is utf16-le.  The poor man's utf-8 to utf16-le
   * conversion below will only handle 7-bit en-us ascii
   */

  len = strlen(str);
  if (len > (USBADB_MAXSTRLEN / 2))
    {
      len = (USBADB_MAXSTRLEN / 2);
    }

  for (i = 0, ndata = 0; i < len; i++, ndata += 2)
    {
      strdesc->data[ndata]   = str[i];
      strdesc->data[ndata + 1] = 0;
    }

  strdesc->len  = ndata + 2;
  strdesc->type = USB_DESC_TYPE_STRING;
  return strdesc->len;
}

/****************************************************************************
 * USB Class Driver Methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbclass_bind
 *
 * Description:
 *   Invoked when the driver is bound to a USB device driver
 *
 ****************************************************************************/

static int usbclass_bind(FAR struct usbdevclass_driver_s *driver,
                         FAR struct usbdev_s *dev)
{
  FAR struct usbdev_adb_s *priv = &((FAR struct adb_driver_s *)driver)->dev;

  usbtrace(TRACE_CLASSBIND, 0);

  _err("CHECK ADDR %p %p\n", dev, driver);

  priv->ctrlreq = usbclass_allocreq(dev->ep0, USBADB_MXDESCLEN);
  if (priv->ctrlreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCCTRLREQ), 0);
      return -ENOMEM;
    }

  priv->ctrlreq->callback = usbclass_ep0incomplete;

  /* Report if we are selfpowered (unless we are part of a
   * composite device)
   */

#ifndef CONFIG_USBADB_COMPOSITE
#ifdef CONFIG_USBDEV_SELFPOWERED
  DEV_SETSELFPOWERED(dev);
#endif

  /* And pull-up the data line for the soft connect function (unless we are
   * part of a composite device)
   */

  DEV_CONNECT(dev);
#endif
  return OK;
}

/****************************************************************************
 * Name: usbclass_unbind
 *
 * Description:
 *    Invoked when the driver is unbound from a USB device driver
 *
 ****************************************************************************/

static void usbclass_unbind(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev)
{
  usbtrace(TRACE_CLASSUNBIND, 0);
}

/****************************************************************************
 * Name: usbclass_setup
 *
 * Description:
 *   Invoked for ep0 control requests.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static int usbclass_setup(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev,
                          FAR const struct usb_ctrlreq_s *ctrl,
                          FAR uint8_t *dataout, size_t outlen)
{
  uint16_t value;
  uint16_t len;
  int ret = -EOPNOTSUPP;

  FAR struct usbdev_adb_s *priv;
  FAR struct usbdev_req_s *ctrlreq;

  /* Extract reference to private data */

  usbtrace(TRACE_CLASSSETUP, ctrl->req);
  priv = &((FAR struct adb_driver_s *)driver)->dev;
  ctrlreq = priv->ctrlreq;

  /* Extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl->value);
  len   = GETUINT16(ctrl->len);

  uinfo("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, index, len);

  if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
      switch (ctrl->req)
        {
        case USB_REQ_GETDESCRIPTOR:
            /* The value field specifies the descriptor type in the MS byte
             * and the descriptor index in the LS byte (order is little
             * endian)
             */

            switch (ctrl->value[1])
              {
              /* If the device is used in as part of a composite
               * device, then the device descriptor is provided by logic in
               * the composite device implementation.
               */

#ifndef CONFIG_USBADB_COMPOSITE
              case USB_DESC_TYPE_DEVICE:
                {
                  ret = USB_SIZEOF_DEVDESC;
                  memcpy(ctrlreq->buf, &g_adb_devdesc, ret);
                }
                break;

              case USB_DESC_TYPE_DEVICEQUALIFIER:
                break;
              case USB_DESC_TYPE_OTHERSPEEDCONFIG:
                break;

              /* If the serial device is used in as part of a composite
               * device, then the configuration descriptor is provided by
               * logic in the composite device implementation.
               */

              case USB_DESC_TYPE_CONFIG:
                {
                  ret = usbclass_mkcfgdesc(ctrlreq->buf, NULL);
                }
                break;

              /* If the serial device is used in as part of a composite
               * device, then the language string descriptor is provided by
               * logic in the composite device implementation.
               */

              case USB_DESC_TYPE_STRING:
                {
                  /* index == language code. */

                  ret =
                  usbclass_mkstrdesc(ctrl->value[0],
                                    (FAR struct usb_strdesc_s *)
                                      ctrlreq->buf);
                }
                break;
#endif

              default:
                {
                  usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_GETUNKNOWNDESC),
                           value);
                }
                break;
              }            

        case USB_REQ_SETCONFIGURATION:
          {
            if (ctrl->type == 0)
              {
                ret = 0; // usbclass_setconfig(priv, value);
              }

            // return 0; /* Composite driver will send the reply */
            // if (ctrl->type == 0)
            //   {
            //     ret = usbclass_setconfig(priv, value);
            //   }
          }
          break;

        /* If the serial device is used in as part of a composite device,
         * then the overall composite class configuration is managed by logic
         * in the composite device implementation.
         */

#ifndef CONFIG_USBADB_COMPOSITE
        case USB_REQ_GETCONFIGURATION:
          {
            if (ctrl->type == USB_DIR_IN)
              {
                *(FAR uint8_t *)ctrlreq->buf = 1; // 0; // priv->config;
                ret = 1;
              }
          }
          break;
#endif

        case USB_REQ_SETINTERFACE:
          {
            /* Only one alternate setting (0) is supported */

            if (value == 0)
              {
                ret = 0;
              }
#if 0
            if (ctrl->type == USB_REQ_RECIPIENT_INTERFACE &&
                priv->config == CDCACM_CONFIGID)
              {
                  if ((index == priv->devinfo.ifnobase &&
                       value == CDCACM_NOTALTIFID) ||
                      (index == (priv->devinfo.ifnobase + 1) &&
                       value == CDCACM_DATAALTIFID))
                  {
                    cdcacm_resetconfig(priv);
                    cdcacm_setconfig(priv, priv->config);
                    ret = 0;
                  }
              }
#endif
          }
          break;

        case USB_REQ_GETINTERFACE:
          {
            *(FAR uint8_t *) ctrlreq->buf = 0;
            ret = 1;
#if 0
            if (ctrl->type == (USB_DIR_IN | USB_REQ_RECIPIENT_INTERFACE) &&
                priv->config == CDCACM_CONFIGIDNONE)
              {
                  if ((index == priv->devinfo.ifnobase &&
                       value == CDCACM_NOTALTIFID) ||
                      (index == (priv->devinfo.ifnobase + 1) &&
                       value == CDCACM_DATAALTIFID))
                   {
                    *(FAR uint8_t *) ctrlreq->buf = value;
                    ret = 1;
                  }
                else
                  {
                    ret = -EDOM;
                  }
              }
#endif
           }
           break;

        default:
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDSTDREQ),
                   ctrl->req);
          break;
        }
    }

  else if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS)
    {
      /**********************************************************************
       * ADB-Specific Requests
       **********************************************************************/

      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ),
               ctrl->req);
    }
  else
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDTYPE), ctrl->type);
    }

  /* Respond to the setup command if data was returned.  On an error return
   * value (ret < 0), the USB driver will stall.
   */

  if (ret >= 0)
    {
      ctrlreq->len   = (len < ret) ? len : ret;
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;

      /* Send the response -- either directly to the USB controller or
       * indirectly in the case where this class is a member of a composite
       * device.
       */

#ifndef CONFIG_USBADB_COMPOSITE
      ret = EP_SUBMIT(dev->ep0, ctrlreq);
#else
      ret = composite_ep0submit(driver, dev, ctrlreq);
#endif



      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPRESPQ), (uint16_t)-ret);
          ctrlreq->result = OK;
          usbclass_ep0incomplete(dev->ep0, ctrlreq);
        }
    }

  /* Returning a negative value will cause a STALL */

  return ret;
}

/****************************************************************************
 * Name: usbclass_disconnect
 *
 * Description:
 *   Invoked after all transfers have been stopped, when the host is
 *   disconnected.  This function is probably called from the context of an
 *   interrupt handler.
 *
 ****************************************************************************/

static void usbclass_disconnect(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev)
{
  usbtrace(TRACE_CLASSDISCONNECT, 0);
}

/****************************************************************************
 * Name: usbclass_suspend
 *
 * Description:
 *   Handle the USB suspend event.
 *
 ****************************************************************************/

static void usbclass_suspend(FAR struct usbdevclass_driver_s *driver,
                             FAR struct usbdev_s *dev)
{
  usbtrace(TRACE_CLASSSUSPEND, 0);
}

/****************************************************************************
 * Name: usbclass_resume
 *
 * Description:
 *   Handle the USB resume event.
 *
 ****************************************************************************/

static void usbclass_resume(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev)
{
  usbtrace(TRACE_CLASSRESUME, 0);
}

/****************************************************************************
 * Name: usbclass_classobject
 *
 * Description:
 *   Register USB driver and return the class object.
 *
 * Returned Value:
 *   0 on success, negative error code on failure.
 *
 ****************************************************************************/

static int usbclass_classobject(int minor,
                                FAR struct usbdev_devinfo_s *devinfo,
                                FAR struct usbdevclass_driver_s **classdev)
{
  int ret;
  FAR struct adb_driver_s *alloc;

  alloc = (FAR struct adb_driver_s *)
    kmm_malloc(sizeof(struct adb_driver_s));

  if (!alloc)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCDEVSTRUCT), 0);
      return -ENOMEM;
    }

  /* Initialize the USB class driver structure */

  alloc->drvr.speed = USB_SPEED_FULL;
  alloc->drvr.ops   = &g_adb_driverops;

#ifdef CONFIG_USBADB_COMPOSITE
  /* Save the caller provided device description (composite only) */

  memcpy(&priv->devinfo, devinfo,
         sizeof(struct usbdev_devinfo_s));
#endif

  /* TODO register char device driver */

  ret = register_driver("/dev/adb0", &g_adb_fops, 0666, alloc);
  if (ret < 0)
    {
      uerr("Failed to register char device");
      goto exit_free_driver;
    }

  *classdev = &alloc->drvr;
  return OK;

exit_free_driver:
  kmm_free(alloc);
  return ret;
}

/****************************************************************************
 * Name: usbclass_uninitialize
 *
 * Description:
 *   Free allocated memory
 *
 * Returned Value:
 *   0 on success, negative error code on failure.
 *
 ****************************************************************************/

static void usbclass_uninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  kmm_free(classdev);
}

/****************************************************************************
 * Char Device Driver Methods
 ****************************************************************************/

static int adb_char_open(FAR struct file *filep)
{
  _err("entry\n");
  return -EINVAL;
}

static int adb_char_close(FAR struct file *filep)
{
  _err("entry\n");
  return -EINVAL;
}

static ssize_t adb_char_read(FAR struct file *filep, FAR char *buffer,
                               size_t len)
{
  _err("entry\n");
  return -EINVAL;
}

static ssize_t adb_char_write(FAR struct file *filep,
                                FAR const char *buffer, size_t len)
{
  _err("entry\n");
  return -EINVAL;
}

static int adb_char_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  _err("entry\n");
  return -EINVAL;
}

#ifdef CONFIG_USBADB_POLL
static int adb_char_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
  _err("entry\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_adb_initialize
 *
 * Description:
 *   Initialize the Android Debug Bridge USB device driver.
 *
 * Returned Value:
 *   0 on success, -errno on failure
 *
 ****************************************************************************/

#ifndef CONFIG_USBADB_COMPOSITE
int usbdev_adb_initialize(void)
{
  int ret;
  FAR struct usbdevclass_driver_s *classdev;
  FAR struct adb_driver_s *drvr;

  ret = usbclass_classobject(0, NULL, &classdev);
  if (ret)
    {
      nerr("usbclass_classobject failed: %d\n", ret);
      return ret;
    }

  drvr = (FAR struct adb_driver_s *)classdev;

  ret = usbdev_register(&drvr->drvr);
  if (ret)
    {
      nerr("usbdev_register failed: %d\n", ret);
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_DEVREGISTER), (uint16_t)-ret);
      usbclass_uninitialize(classdev);
      return ret;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: usbdev_adb_get_composite_devdesc
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

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_USBADB_COMPOSITE)
void usbdev_adb_get_composite_devdesc(struct composite_devdesc_s *dev)
{
  memset(dev, 0, sizeof(struct composite_devdesc_s));

  dev->mkconfdesc          = usbclass_mkcfgdesc;
  dev->mkstrdesc           = usbclass_mkstrdesc;
  dev->classobject         = usbclass_classobject;
  dev->uninitialize        = usbclass_uninitialize;
  dev->nconfigs            = 1;
  dev->configid            = 0; // 1 CDC
  dev->cfgdescsize         = sizeof(usb_ifdesc_s); // cdcacm_mkcfgdesc(NULL, NULL);
  dev->devinfo.ninterfaces = 1; // 2 CDC
  dev->devinfo.nstrings    = 1; // CDCACM_NSTRIDS             (CDCACM_LASTSTRID - CDCACM_STRBASE)
  dev->devinfo.nendpoints  = 0; // 3 cdc
}
#endif
