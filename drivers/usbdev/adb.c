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

#define CONFIG_USBADB_EPBULKIN_HSSIZE 512
#define CONFIG_USBADB_EPBULKIN_FSSIZE 64
#define CONFIG_USBADB_EPBULKOUT_HSSIZE 512
#define CONFIG_USBADB_EPBULKOUT_FSSIZE 64

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

#define USBADB_NCONFIGS          (1)
#define USBADB_CONFIGID          (1)
#define USBADB_CONFIGIDNONE      (0)

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

  FAR struct usbdev_ep_s  *epbulkin;  /* Bulk IN endpoint structure */
  FAR struct usbdev_ep_s  *epbulkout; /* Bulk OUT endpoint structure */

  FAR struct usbdev_req_s *ctrlreq;   /* Preallocated control request */
  FAR struct usbdev_req_s *rdreq;     /* Preallocated read request */

  uint8_t config;                     /* USB Configuration number */
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
  // struct usb_epdesc_s  epbulkoutdesc;  /* Bulk out interface descriptor */
  // struct usb_epdesc_s  epbulkindesc;   /* Bulk in interface descriptor */
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
  }
#if 0
  ,{
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
#endif
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

/****************************************************************************
 * Name: usbclass_copy_epdesc
 *
 * Description:
 *   Copies the requested Endpoint Description into the buffer given.
 *   Returns the number of Bytes filled in ( sizeof(struct usb_epdesc_s) ).
 *
 ****************************************************************************/

static int usbclass_copy_epdesc(int epid, FAR struct usb_epdesc_s *epdesc,
                                FAR struct usbdev_devinfo_s *devinfo,
                                bool hispeed)
{
#ifndef CONFIG_USBDEV_DUALSPEED
  UNUSED(hispeed);
#endif

  epdesc->len  = USB_SIZEOF_EPDESC;           /* Descriptor length */
  epdesc->type = USB_DESC_TYPE_ENDPOINT;      /* Descriptor type */
  epdesc->attr = USB_EP_ATTR_XFER_BULK |      /* Endpoint attributes */
                 USB_EP_ATTR_NO_SYNC |
                 USB_EP_ATTR_USAGE_DATA;
  epdesc->interval = 0;                       /* Interval */

  if (epid == USBADB_EP_BULKIN_IDX) /* Bulk IN endpoint */
    {
      /* Endpoint address */

#ifdef CONFIG_USBADB_COMPOSITE
      epdesc->addr = USB_EPIN(devinfo->epno[USBADB_EP_BULKIN_IDX]);
#else
      epdesc->addr = USB_EPIN(CONFIG_USBADB_EPBULKIN);
#endif

#ifdef CONFIG_USBDEV_DUALSPEED
      if (hispeed)
        {
          /* Maximum packet size (high speed) */

          epdesc->mxpacketsize[0] = LSBYTE(CONFIG_USBADB_EPBULKIN_HSSIZE);
          epdesc->mxpacketsize[1] = MSBYTE(CONFIG_USBADB_EPBULKIN_HSSIZE);
        }
      else
#endif
        {
          /* Maximum packet size (full speed) */

          epdesc->mxpacketsize[0] = LSBYTE(CONFIG_USBADB_EPBULKIN_FSSIZE);
          epdesc->mxpacketsize[1] = MSBYTE(CONFIG_USBADB_EPBULKIN_FSSIZE);
        }
    }
  else /* USBADB_EP_BULKOUT_IDX: Bulk OUT endpoint */
    {
      /* Endpoint address */

#ifdef CONFIG_USBADB_COMPOSITE
      epdesc->addr = USB_EPOUT(devinfo->epno[USBADB_EP_BULKOUT_IDX]);
#else
      epdesc->addr = USB_EPOUT(CONFIG_USBADB_EPBULKOUT);
#endif

#ifdef CONFIG_USBDEV_DUALSPEED
      if (hispeed)
        {
          /* Maximum packet size (high speed) */

          epdesc->mxpacketsize[0] = LSBYTE(CONFIG_USBADB_EPBULKOUT_HSSIZE);
          epdesc->mxpacketsize[1] = MSBYTE(CONFIG_USBADB_EPBULKOUT_HSSIZE);
        }
      else
#endif
        {
          /* Maximum packet size (full speed) */

          epdesc->mxpacketsize[0] = LSBYTE(CONFIG_USBADB_EPBULKOUT_FSSIZE);
          epdesc->mxpacketsize[1] = MSBYTE(CONFIG_USBADB_EPBULKOUT_FSSIZE);
        }
    }

  return sizeof(struct usb_epdesc_s);
}






/****************************************************************************
 * Name: usb_adb_submit_rdreq
 *
 * Description:
 *   Submits the bulk OUT read request. Takes care not to submit the request
 *   when the RX packet buffer is already in use.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Returned Value:
 *   The return value of the EP_SUBMIT operation
 *
 ****************************************************************************/

static int usb_adb_submit_rdreq(FAR struct usbdev_adb_s *priv)
{
  irqstate_t flags = enter_critical_section();
  int ret = OK;

  // if (!priv->rdreq_submitted && !priv->rx_blocked)
  //   {
      priv->rdreq->len = priv->epbulkout->maxpacket;
      ret = EP_SUBMIT(priv->epbulkout, priv->rdreq);
      if (ret != OK)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT),
                   (uint16_t)-priv->rdreq->result);
        }
      // else
      //   {
      //     priv->rdreq_submitted = true;
      //   }
    // }

  leave_critical_section(flags);
  return ret;
}


/****************************************************************************
 * Name: usb_adb_rdcomplete
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.
 *
 ****************************************************************************/

static void usb_adb_rdcomplete(FAR struct usbdev_ep_s *ep,
                               FAR struct usbdev_req_s *req)
{
  FAR struct usbdev_adb_s *priv;
  irqstate_t flags;
  int ret;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !ep->priv || !req)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract references to private data */

  priv = (FAR struct usbdev_adb_s *)ep->priv;

  /* Process the received data unless this is some unusual condition */

  ret = OK;

  _err("GOT USB FRAME %d %d\n", req->result, req->xfrd);
  DumpHex(req->buf, req->len);

  flags = enter_critical_section();
  // priv->rdreq_submitted = false;

#if 0
  switch (req->result)
    {
    case 0: /* Normal completion */
      ret = rndis_recvpacket(priv, req->buf, req->xfrd);
      DEBUGASSERT(ret != -ENOMEM);
      break;

    case -ESHUTDOWN: /* Disconnection */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSHUTDOWN), 0);
      leave_critical_section(flags);
      return;

    default: /* Some other error occurred */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDUNEXPECTED),
               (uint16_t)-req->result);
      break;
    };
#endif

  if (ret == OK)
    {
      usb_adb_submit_rdreq(priv);
    }

  leave_critical_section(flags);
}









/****************************************************************************
 * Name: usbclass_resetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

static void usbclass_resetconfig(FAR struct usbdev_adb_s *priv)
{
  /* Are we configured? */

  if (priv->config != USBADB_CONFIGIDNONE)
    {
      /* Yes.. but not anymore */

      priv->config = USBADB_CONFIGIDNONE;

      // TODO unlock all char device pollers
      #warning TODO

      /* Disable endpoints.  This should force completion of all pending
       * transfers.
       */

      EP_DISABLE(priv->epbulkin);
      EP_DISABLE(priv->epbulkout);
    }
}

/****************************************************************************
 * Name: usbclass_setconfig
 *
 * Description:
 *   Set the device configuration by allocating and configuring endpoints and
 *   by allocating and queue read and write requests.
 *
 ****************************************************************************/

static int usbclass_setconfig(FAR struct usbdev_adb_s *priv, uint8_t config)
{
  struct usb_epdesc_s epdesc;
// #ifdef CONFIG_USBDEV_DUALSPEED
  bool hispeed = false;
// #endif
  int ret = 0;

#ifdef CONFIG_DEBUG_FEATURES
  if (priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EIO;
    }
#endif

#ifdef CONFIG_USBDEV_DUALSPEED
  hispeed = (priv->usbdev->speed == USB_SPEED_HIGH);
#endif

  if (config == priv->config)
    {
      /* Already configured -- Do nothing */

      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALREADYCONFIGURED), 0);
      return 0;
    }

  /* Discard the previous configuration data */

  usbclass_resetconfig(priv);

  /* Was this a request to simply discard the current configuration? */

  if (config == USBADB_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGNONE), 0);
      return 0;
    }

  /* We only accept one configuration */

  if (config != USBADB_CONFIGID)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGIDBAD), 0);
      return -EINVAL;
    }

  /* Configure the IN bulk endpoint */

#ifdef CONFIG_USBADB_COMPOSITE
  usbclass_copy_epdesc(USBADB_EP_BULKIN_IDX, &epdesc,
                       &priv->devinfo, hispeed);
#else
  usbclass_copy_epdesc(USBADB_EP_BULKIN_IDX, &epdesc, NULL, hispeed);
#endif

  ret = EP_CONFIGURE(priv->epbulkin, &epdesc, false);

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Configure the OUT bulk endpoint */

#ifdef CONFIG_USBADB_COMPOSITE
  usbclass_copy_epdesc(USBADB_EP_BULKOUT_IDX, &epdesc,
                       &priv->devinfo, hispeed);
#else
  usbclass_copy_epdesc(USBADB_EP_BULKOUT_IDX, &epdesc, NULL, hispeed);
#endif
  ret = EP_CONFIGURE(priv->epbulkout, &epdesc, true);

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkout->priv = priv;





  /* Queue read requests in the bulk OUT endpoint */

  // priv->rdreq->callback = usb_adb_rdcomplete;

  ret = usb_adb_submit_rdreq(priv);
  if (ret != OK)
    {
      goto errout;
    }

#if 0
  priv->rdreq->callback = rndis_rdcomplete;
  ret = rndis_submit_rdreq(priv);
  if (ret != OK)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT), (uint16_t)-ret);
      goto errout;
    }
#endif
  /* We are successfully configured */

  priv->config = config;

  #warning TODO mark char device as active (write/read ok)

  return OK;

errout:
  usbclass_resetconfig(priv);
  return ret;
}

static void usbclass_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                 FAR struct usbdev_req_s *req)
{
  _err("entry\n");
}

#ifdef CONFIG_USBDEV_DUALSPEED
static int16_t usbclass_mkcfgdesc(FAR uint8_t *buf,
                                  FAR struct usbdev_devinfo_s *devinfo,
                                  uint8_t speed, uint8_t type);
#else
static int16_t usbclass_mkcfgdesc(FAR uint8_t *buf,
                                  FAR struct usbdev_devinfo_s *devinfo)
#endif
{
  // dest->ifdesc.ifno += devinfo->ifnobase;
  // dest->ifdesc.iif = devinfo->strbase;

  bool hispeed = false;
  FAR struct usb_epdesc_s *epdesc;
  FAR struct adb_cfgdesc_s *dest;

#ifdef CONFIG_USBDEV_DUALSPEED
  hispeed = (speed == USB_SPEED_HIGH);

  /* Check for switches between high and full speed */

  if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG)
    {
      hispeed = !hispeed;
    }
#endif

  dest = (FAR struct adb_cfgdesc_s *)buf;
  epdesc = (FAR struct usb_epdesc_s *)(buf+sizeof(g_adb_cfgdesc));

  memcpy(dest, &g_adb_cfgdesc, sizeof(g_adb_cfgdesc));

#ifdef CONFIG_USBADB_COMPOSITE
  usbclass_copy_epdesc(USBADB_EP_BULKIN_IDX, &epdesc[0], devinfo, hispeed);
  usbclass_copy_epdesc(USBADB_EP_BULKOUT_IDX, &epdesc[1], devinfo, hispeed);
#else
  usbclass_copy_epdesc(USBADB_EP_BULKIN_IDX, &epdesc[0], NULL, hispeed);
  usbclass_copy_epdesc(USBADB_EP_BULKOUT_IDX, &epdesc[1], NULL, hispeed);
#endif

#ifdef CONFIG_USBADB_COMPOSITE
  /* For composite device, apply possible offset to the interface numbers */

  dest->ifdesc.ifno = devinfo->ifnobase;
  dest->ifdesc.iif  = devinfo->strbase;
#endif

  return sizeof(g_adb_cfgdesc)+2*USB_SIZEOF_EPDESC;
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
  int ret;
  uint16_t reqlen;
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

  /* Pre-allocate all endpoints... the endpoints will not be functional
   * until the SET CONFIGURATION request is processed in usbclass_setconfig.
   * This is done here because there may be calls to kmm_malloc and the SET
   * CONFIGURATION processing probably occurs within interrupt handling
   * logic where kmm_malloc calls will fail.
   */

  /* Pre-allocate the IN bulk endpoint */

  priv->epbulkin = DEV_ALLOCEP(dev,
#ifdef CONFIG_USBADB_COMPOSITE
      USB_EPIN(priv->devinfo.epno[USBADB_EP_BULKIN_IDX]),
#else
      USB_EPIN(CONFIG_USBADB_EPBULKIN),
#endif
      true,
      USB_EP_ATTR_XFER_BULK);

  if (!priv->epbulkin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Pre-allocate the OUT bulk endpoint */

  priv->epbulkout = DEV_ALLOCEP(dev,
#ifdef CONFIG_USBADB_COMPOSITE
      USB_EPOUT(priv->devinfo.epno[USBADB_EP_BULKOUT_IDX]),
#else
      USB_EPOUT(CONFIG_USBADB_EPBULKOUT),
#endif
      false,
      USB_EP_ATTR_XFER_BULK);

  if (!priv->epbulkout)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epbulkout->priv = priv;

  /* Pre-allocate read requests.  The buffer size is one full packet. */

#ifdef CONFIG_USBDEV_DUALSPEED
  reqlen = CONFIG_USBADB_EPBULKOUT_HSSIZE;
#else
  reqlen = CONFIG_USBADB_EPBULKOUT_FSSIZE;
#endif

  priv->rdreq = usbclass_allocreq(priv->epbulkout, reqlen);
  if (priv->rdreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDALLOCREQ), -ENOMEM);
      ret = -ENOMEM;
      goto errout;
    }

  priv->rdreq->callback = usb_adb_rdcomplete;

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

errout:
  usbclass_unbind(driver, dev);
  return ret;
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

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !dev->ep0 || !ctrl)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EIO;
    }
#endif

  /* Extract reference to private data */

  usbtrace(TRACE_CLASSSETUP, ctrl->req);
  priv = &((FAR struct adb_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv || !priv->ctrlreq)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return -ENODEV;
    }
#endif

  ctrlreq = priv->ctrlreq;

  /* Extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl->value);
  len   = GETUINT16(ctrl->len);

  uinfo("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, index, len);

  switch (ctrl->type & USB_REQ_TYPE_MASK)
    {
      case USB_REQ_TYPE_STANDARD:
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
                    ret = usbclass_setconfig(priv, value);
                  }
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
                    *(FAR uint8_t *)ctrlreq->buf = priv->config;
                    ret = 1;
                  }
              }
              break;
#endif

            case USB_REQ_SETINTERFACE:
            case USB_REQ_GETINTERFACE:
            default:
              usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDSTDREQ),
                       ctrl->req);
              break;
            }
        }

      case USB_REQ_TYPE_CLASS:
        {
          /* ADB-Specific Requests */

          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ),
                  ctrl->req);
        }

      default:
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDTYPE), ctrl->type);
        }
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
    kmm_zalloc(sizeof(struct adb_driver_s));

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
  // TODO unregister char driver
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
  dev->configid            = 1; // 1 CDC
  dev->cfgdescsize         = sizeof(usb_ifdesc_s); // cdcacm_mkcfgdesc(NULL, NULL);
  dev->devinfo.ninterfaces = 1; // 2 CDC
  dev->devinfo.nstrings    = 1; // CDCACM_NSTRIDS             (CDCACM_LASTSTRID - CDCACM_STRBASE)
  dev->devinfo.nendpoints  = 2; // 3 cdc
}
#endif
