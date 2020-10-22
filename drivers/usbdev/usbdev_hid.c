/****************************************************************************
 * drivers/usbdev/usbdev_hid.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/wdog.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/cdc.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhid.h>
#include <nuttx/usb/usbdev_trace.h>

#include "usbdev_hid.h"
#include <nuttx/usb/hid.h>

#ifdef CONFIG_USBHID_COMPOSITE
#  include <nuttx/usb/composite.h>
#  include "composite.h"
#endif

//Class specific descriptor - HID mouse
const uint8_t hid_rpt01[HID_RPT01_SIZE] = {
    0x05, 0x01, /* Usage Page (Generic Desktop)             */
    0x09, 0x02, /* Usage (Mouse)                            */
    0xA1, 0x01, /* Collection (Application)                 */
    0x09, 0x01, /*  Usage (Pointer)                         */
    0xA1, 0x00, /*  Collection (Physical)                   */
    0x05, 0x09, /*      Usage Page (Buttons)                */
    0x19, 0x01, /*      Usage Minimum (01)                  */
    0x29, 0x05, /*      Usage Maximum (05)                  */
    0x15, 0x00, /*      Logical Minimum (0)                 */
    0x25, 0x01, /*      Logical Maximum (1)                 */
    0x95, 0x05, /*      Report Count (5)                    */
    0x75, 0x01, /*      Report Size (1)                     */
    0x81, 0x02, /*      Input (Data, Variable, Absolute)    */
    0x95, 0x01, /*      Report Count (1)                    */
    0x75, 0x03, /*      Report Size (3)                     */
    0x81, 0x01, /*      Input (Constant)    ;5 bit padding  */
    0x05, 0x01, /*      Usage Page (Generic Desktop)        */
    0x09, 0x30, /*      Usage (X)                           */
    0x09, 0x31, /*      Usage (Y)                           */
    0x09, 0x38, /*      Usage (Wheel)                       */
    0x15, 0x81, /*      Logical Minimum (-127)              */
    0x25, 0x7F, /*      Logical Maximum (127)               */
    0x75, 0x08, /*      Report Size (8)                     */
    0x95, 0x03, /*      Report Count (3)                    */
    0x81, 0x06, /*      Input (Data, Variable, Relative)    */
    0xC0, 0xC0};/* End Collection,End Collection            */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the internal state of the driver */

struct cdcacm_dev_s
{
  FAR struct usbdev_s *usbdev;         /* usbdev driver pointer */

  uint8_t config;                      /* Configuration number */
  uint8_t minor;                       /* The device minor number */

  FAR struct usbdev_ep_s *epintin;     /* Interrupt IN endpoint structure */
  FAR struct usbdev_req_s *ctrlreq;    /* Allocated control request */
  FAR struct usbdev_req_s *req_test;
#ifdef CONFIG_USBHID_COMPOSITE
  struct usbdev_devinfo_s devinfo;
#endif
  /* HID specific */

  uint8_t idle;
  struct wdog_s test_move_timer;
};

/* The internal version of the class driver */

struct cdcacm_driver_s
{
  struct usbdevclass_driver_s drvr;
  FAR struct cdcacm_dev_s     *dev;
};

/* This is what is allocated */

struct cdcacm_alloc_s
{
  struct cdcacm_dev_s    dev;
  struct cdcacm_driver_s drvr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void cdcacm_wrcomplete(FAR struct usbdev_ep_s *ep,
                              FAR struct usbdev_req_s *req)
{
  // _info("%d\n", req->result);
}


/* Request helpers *********************************************************/

static struct usbdev_req_s *cdcacm_allocreq(FAR struct usbdev_ep_s *ep,
                 uint16_t len);
static void    cdcacm_freereq(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);

/* Configuration ***********************************************************/

static void    cdcacm_resetconfig(FAR struct cdcacm_dev_s *priv);
static int     cdcacm_epconfigure(FAR struct usbdev_ep_s *ep,
                 bool last,
                 FAR struct usbdev_devinfo_s *devinfo,
                 bool hispeed);
static int     cdcacm_setconfig(FAR struct cdcacm_dev_s *priv,
                 uint8_t config);

/* Completion event handlers ***********************************************/

static void    cdcacm_ep0incomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);

/* USB class device ********************************************************/

static int     cdcacm_bind(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static void    cdcacm_unbind(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static int     cdcacm_setup(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev,
                 FAR const struct usb_ctrlreq_s *ctrl, FAR uint8_t *dataout,
                 size_t outlen);
static void    cdcacm_disconnect(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static void    cdcacm_suspend(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static void    cdcacm_resume(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB class device *********************************************************/

static const struct usbdevclass_driverops_s g_driverops =
{
  cdcacm_bind,           /* bind */
  cdcacm_unbind,         /* unbind */
  cdcacm_setup,          /* setup */
  cdcacm_disconnect,     /* disconnect */
  cdcacm_suspend,        /* suspend */
  cdcacm_resume,         /* resume */
};

struct usbhid_mousereport2_s
{
  uint8_t buttons;   /* See USBHID_MOUSEIN_* definitions */
  uint8_t xdisp;     /* X displacement */
  uint8_t ydisp;     /* y displacement */
                     /* Device specific additional bytes may follow */
  uint8_t wdisp;     /* Wheel displacement */
};

static void cdcacm_rxtimeout(wdparm_t arg)
{
  FAR struct cdcacm_dev_s *priv = (FAR struct cdcacm_dev_s *)arg;
  // _info("timeout\n");
  int ret;
  memset(priv->req_test->buf, 0, 4);

  struct usbhid_mousereport2_s *aa = (struct usbhid_mousereport2_s*)priv->req_test->buf;
  aa->xdisp = 4;

  /* Then submit the request to the endpoint */
  priv->req_test->len             = 4;
  priv->req_test->priv            = priv;
  priv->req_test->flags           = USBDEV_REQFLAGS_NULLPKT;
  ret                  = EP_SUBMIT(priv->epintin, priv->req_test);

  if (ret < 0)
    {
      _err("Failed %d\n", ret);
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL), (uint16_t)-ret);
    }

  // _err("RUN WDOG2\n");
  (void)wd_start(&priv->test_move_timer, 10, cdcacm_rxtimeout, arg);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cdcacm_allocreq
 *
 * Description:
 *   Allocate a request instance along with its buffer
 *
 ****************************************************************************/

static struct usbdev_req_s *cdcacm_allocreq(FAR struct usbdev_ep_s *ep,
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
 * Name: cdcacm_freereq
 *
 * Description:
 *   Free a request instance along with its buffer
 *
 ****************************************************************************/

static void cdcacm_freereq(FAR struct usbdev_ep_s *ep,
                           FAR struct usbdev_req_s *req)
{
  if (ep != NULL && req != NULL)
    {
      if (req->buf != NULL)
        {
          EP_FREEBUFFER(ep, req->buf);
        }

      EP_FREEREQ(ep, req);
    }
}

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

  if (priv->config != USBHID_CONFIGIDNONE)
    {
      /* Yes.. but not anymore */

      priv->config = USBHID_CONFIGIDNONE;

      /* Disable endpoints.  This should force completion of all pending
       * transfers.
       */

      EP_DISABLE(priv->epintin);
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
                              bool last,
                              FAR struct usbdev_devinfo_s *devinfo,
                              bool hispeed)
{
  struct usb_epdesc_s epdesc;
  usbhid_copy_epdesc(&epdesc, devinfo, hispeed);
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

  if (config == USBHID_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGNONE), 0);
      return 0;
    }

  /* We only accept one configuration */

  if (config != USBHID_CONFIGID)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGIDBAD), 0);
      return -EINVAL;
    }

  /* Configure the IN interrupt endpoint */

#ifdef CONFIG_USBDEV_DUALSPEED
  if (priv->usbdev->speed == USB_SPEED_HIGH)
    {
      ret = cdcacm_epconfigure(priv->epintin, false,
                               &priv->devinfo, true);
    }
  else
#endif
    {
        ret = cdcacm_epconfigure(priv->epintin, false,
                                 &priv->devinfo, false);
    }

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINCONFIGFAIL), 0);
      goto errout;
    }

  _err("OK\n");

  priv->epintin->priv = priv;

  /* We are successfully configured */

  priv->config = config;

  return OK;

errout:
  _err("failed\n");
  cdcacm_resetconfig(priv);
  return ret;
}

/****************************************************************************
 * Name: cdcacm_ep0incomplete
 *
 * Description:
 *   Handle completion of EP0 control operations
 *
 ****************************************************************************/

static void cdcacm_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                 FAR struct usbdev_req_s *req)
{
  if (req->result || req->xfrd != req->len)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_REQRESULT), (uint16_t)-req->result);
    }
}

/****************************************************************************
 * USB Class Driver Methods
 ****************************************************************************/

/****************************************************************************
 * Name: cdcacm_bind
 *
 * Description:
 *   Invoked when the driver is bound to a USB device driver
 *
 ****************************************************************************/

static int cdcacm_bind(FAR struct usbdevclass_driver_s *driver,
                       FAR struct usbdev_s *dev)
{
  FAR struct cdcacm_dev_s *priv = ((FAR struct cdcacm_driver_s *)driver)->dev;
  int ret;

  usbtrace(TRACE_CLASSBIND, 0);

  /* Bind the structures */

  priv->usbdev   = dev;

  /* Save the reference to our private data structure in EP0 so that it
   * can be recovered in ep0 completion events (Unless we are part of
   * a composite device and, in that case, the composite device owns
   * EP0).
   */

#ifndef CONFIG_USBHID_COMPOSITE
  dev->ep0->priv = priv;
#endif

  /* Preallocate control request */

  priv->ctrlreq = cdcacm_allocreq(dev->ep0, USBHID_MXDESCLEN);
  if (priv->ctrlreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCCTRLREQ), 0);
      ret = -ENOMEM;
      goto errout;
    }

  priv->ctrlreq->callback = cdcacm_ep0incomplete;

  /* Pre-allocate all endpoints... the endpoints will not be functional
   * until the SET CONFIGURATION request is processed in cdcacm_setconfig.
   * This is done here because there may be calls to kmm_malloc and the SET
   * CONFIGURATION processing probably occurrs within interrupt handling
   * logic where kmm_malloc calls will fail.
   */

  /* Pre-allocate the IN interrupt endpoint */

  _err("use EP %d\n", USBHID_MKEPINTIN(&priv->devinfo));

  priv->epintin = DEV_ALLOCEP(dev, USBHID_MKEPINTIN(&priv->devinfo),
                              true, USB_EP_ATTR_XFER_INT);
  if (!priv->epintin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epintin->priv = priv;


  priv->req_test = cdcacm_allocreq(priv->epintin, 4);
  if (priv->req_test == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRALLOCREQ), -ENOMEM);
      ret = -ENOMEM;
      goto errout;
    }

  priv->req_test->priv     = priv;
  priv->req_test->callback = cdcacm_wrcomplete;

  /* Report if we are selfpowered (unless we are part of a composite device) */

#ifndef CONFIG_USBHID_COMPOSITE
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
  cdcacm_unbind(driver, dev);
  return ret;
}

/****************************************************************************
 * Name: cdcacm_unbind
 *
 * Description:
 *    Invoked when the driver is unbound from a USB device driver
 *
 ****************************************************************************/

static void cdcacm_unbind(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev)
{
  FAR struct cdcacm_dev_s *priv;

  usbtrace(TRACE_CLASSUNBIND, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
     }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct cdcacm_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Make sure that we are not already unbound */

  if (priv == NULL)
    {
      return;
    }

  /* Make sure that the endpoints have been unconfigured.  If
   * we were terminated gracefully, then the configuration should
   * already have been reset.  If not, then calling cdcacm_resetconfig
   * should cause the endpoints to immediately terminate all
   * transfers and return the requests to us (with result == -ESHUTDOWN)
   */

  cdcacm_resetconfig(priv);
  up_mdelay(50);

  /* Free the interrupt IN endpoint */

  if (priv->epintin)
    {
      DEV_FREEEP(dev, priv->epintin);
      priv->epintin = NULL;
    }

  /* Free the pre-allocated control request */

  if (priv->ctrlreq != NULL)
    {
      cdcacm_freereq(dev->ep0, priv->ctrlreq);
      priv->ctrlreq = NULL;
    }
}

/****************************************************************************
 * Name: cdcacm_setup
 *
 * Description:
 *   Invoked for ep0 control requests.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static int cdcacm_setup(FAR struct usbdevclass_driver_s *driver,
                        FAR struct usbdev_s *dev,
                        FAR const struct usb_ctrlreq_s *ctrl,
                        FAR uint8_t *dataout, size_t outlen)
{
  FAR struct cdcacm_dev_s *priv;
  FAR struct usbdev_req_s *ctrlreq;
  uint16_t value;
  uint16_t index;
  uint16_t len;
  int ret = -EOPNOTSUPP;
  int send_frame = 0;

  /* Extract reference to private data */

  usbtrace(TRACE_CLASSSETUP, ctrl->req);
  priv = ((FAR struct cdcacm_driver_s *)driver)->dev;

  ctrlreq = priv->ctrlreq;

  /* Extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl->value);
  index = GETUINT16(ctrl->index);
  len   = GETUINT16(ctrl->len);

  _info("t=%02x req=%02x v=%04x idx=%04x l=%04x\n",
        ctrl->type, ctrl->req, value, index, len);

  if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
      /**********************************************************************
       * Standard Requests
       **********************************************************************/

      switch (ctrl->req)
        {
        case USB_REQ_GETDESCRIPTOR:
          {
            // _info("a %d\n", ctrl->value[1]);
            /* The value field specifies the descriptor type in the MS byte
             * and the descriptor index in the LS byte (order is little
             * endian)
             */

            switch (ctrl->value[1])
              {
              /* If the serial device is used in as part of a composite
               * device, then the device descriptor is provided by logic in
               * the composite device implementation.
               */

#ifndef CONFIG_USBHID_COMPOSITE
              case USB_DESC_TYPE_DEVICE:
                {
                  // _info("b\n");
                  ret = USB_SIZEOF_DEVDESC;
                  memcpy(ctrlreq->buf, usbhid_getdevdesc(), ret);
                  // _info("got %d %02x\n", ret, ctrlreq->buf[8]);
                }
                break;
#endif

              /* If the serial device is used in as part of a composite
               * device, then the device qualifier descriptor is provided by
               * logic in the composite device implementation.
               */

#if !defined(CONFIG_USBHID_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
              case USB_DESC_TYPE_DEVICEQUALIFIER:
                {
                  ret = USB_SIZEOF_QUALDESC;
                  memcpy(ctrlreq->buf, usbhid_getqualdesc(), ret);
                }
                break;

              case USB_DESC_TYPE_OTHERSPEEDCONFIG:
#endif

              /* If the serial device is used in as part of a composite
               * device, then the configuration descriptor is provided by
               * logic in the composite device implementation.
               */

#ifndef CONFIG_USBHID_COMPOSITE
              case USB_DESC_TYPE_CONFIG:
                {
#ifdef CONFIG_USBDEV_DUALSPEED
                  ret = usbhid_mkcfgdesc(ctrlreq->buf, &priv->devinfo,
                                         dev->speed, ctrl->req);
#else
                  ret = usbhid_mkcfgdesc(ctrlreq->buf, &priv->devinfo);
#endif
                }
                break;
#endif

              /* If the serial device is used in as part of a composite
               * device, then the language string descriptor is provided by
               * logic in the composite device implementation.
               */

#ifndef CONFIG_USBHID_COMPOSITE
              case USB_DESC_TYPE_STRING:
                {
                  /* index == language code. */

                  ret =
                  usbhid_mkstrdesc(ctrl->value[0],
                                  (FAR struct usb_strdesc_s *)
                                    ctrlreq->buf);
                }
                break;
#endif

              case USBHID_DESCTYPE_HID:
                _err("BUILD HID descr\n");
                ret = usbhid_mkhiddesc((FAR struct usbhid_descriptor_s*)ctrlreq->buf);
                _err("ret %d\n", ret);
                send_frame = 1;
                break;

              case USBHID_DESCTYPE_REPORT:
                _err("USBHID_DESCTYPE_REPORT\n");
                memcpy(ctrlreq->buf, hid_rpt01, HID_RPT01_SIZE);
                ret = HID_RPT01_SIZE;
                send_frame = 1;
                break;

              case USBHID_DESCTYPE_PHYSICAL:
                ret = 0;
                send_frame = 1;
                break;

              default:
                {
                  usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_GETUNKNOWNDESC), value);
                }
                break;
              }
          }
          break;

        case USB_REQ_SETCONFIGURATION:
          {
            if (ctrl->type == 0)
              {
                ret = cdcacm_setconfig(priv, value);
              }
          }
          break;

        /* If the serial device is used in as part of a composite device,
         * then the overall composite class configuration is managed by logic
         * in the composite device implementation.
         */

#ifndef CONFIG_USBHID_COMPOSITE
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
          {
            if (ctrl->type == USB_REQ_RECIPIENT_INTERFACE &&
                priv->config == USBHID_CONFIGID)
              {
                  if ((index == priv->devinfo.ifnobase &&
                       value == USBHID_NOTALTIFID))
                  {
                    cdcacm_resetconfig(priv);
                    cdcacm_setconfig(priv, priv->config);
                    ret = 0;
                  }
              }
          }
          break;

        case USB_REQ_GETINTERFACE:
          {
            if (ctrl->type == (USB_DIR_IN | USB_REQ_RECIPIENT_INTERFACE) &&
                priv->config == USBHID_CONFIGIDNONE)
              {
                  if ((index == priv->devinfo.ifnobase &&
                       value == USBHID_NOTALTIFID))
                   {
                    *(FAR uint8_t *) ctrlreq->buf = value;
                    ret = 1;
                  }
                else
                  {
                    ret = -EDOM;
                  }
              }
           }
           break;

        default:
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDSTDREQ), ctrl->req);
          break;
        }
    }

  else if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS)
    {
      /**********************************************************************
       * HID Specific Requests
       **********************************************************************/

      _err("HID REQ\n");
      switch (ctrl->req)
        {

/* Class-specific requests (HID 7.2)
 *
 *   bmRequestType (                 USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE) -or-
 *                 (USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE)
 *   bRequest      Class-specific request
 *   wValue        Varies according to request
 *   wIndex        Varies according to request
 *   wLength       Number of bytes to transfer in the data phase
 *   Data          Data
 */
        case USBHID_REQUEST_GETREPORT:
          _info("get report\n");
          break;

        case USBHID_REQUEST_GETIDLE:
          ctrlreq->buf[0] = priv->idle;
          ret = 1;
          send_frame = 1;
          break;

        case USBHID_REQUEST_GETPROTOCOL:
          ctrlreq->buf[0] = 0;
          ret = 1;
          send_frame = 1;
          break;

        case USBHID_REQUEST_SETREPORT:
          /* Respond with a zero length packet */
          ret = 0;
          send_frame = 1;
          break;

        case USBHID_REQUEST_SETIDLE:
          _err("set idle %d\n", ctrlreq->buf[0]);
          priv->idle = ctrlreq->buf[0];
          /* Respond with a zero length packet */
          ret = 0;
          _err("RUN WDOG1\n");
          (void)wd_start(&priv->test_move_timer, 20, cdcacm_rxtimeout, (wdparm_t)priv);
          send_frame = 1;
          break;

        case USBHID_REQUEST_SETPROTOCOL:
          /* Respond with a zero length packet */
          ret = 0;
          send_frame = 1;
          break;

        default:
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ), ctrl->req);
          break;
        }
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
      /* Configure the response */

      ctrlreq->len   = MIN(len, ret);
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;

      /* Send the response -- either directly to the USB controller or
       * indirectly in the case where this class is a member of a composite
       * device.
       */

#ifndef CONFIG_USBHID_COMPOSITE
      ret = EP_SUBMIT(dev->ep0, ctrlreq);
      #error abcd
#else
      if (send_frame) {
        ret = composite_ep0submit(driver, dev, ctrlreq);
      }
      else {
        ret = 0; // composite_ep0submit(driver, dev, ctrlreq);
      }
#endif
      _err("SUBMIT %d %d\n", ret, ctrlreq->len);
      if (ret < 0)
        {
          _err("ERROR %d\n", ret);
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPRESPQ), (uint16_t)-ret);
          ctrlreq->result = OK;
          cdcacm_ep0incomplete(dev->ep0, ctrlreq);
        }
      // _info("Done %d\n", ret);
    }

  /* Returning a negative value will cause a STALL */
  _err("ret %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: cdcacm_disconnect
 *
 * Description:
 *   Invoked after all transfers have been stopped, when the host is
 *   disconnected.  This function is probably called from the context of an
 *   interrupt handler.
 *
 ****************************************************************************/

static void cdcacm_disconnect(FAR struct usbdevclass_driver_s *driver,
                              FAR struct usbdev_s *dev)
{
  FAR struct cdcacm_dev_s *priv;
  irqstate_t flags;

  usbtrace(TRACE_CLASSDISCONNECT, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !dev->ep0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
     }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct cdcacm_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Inform the "upper half serial driver that we have lost the USB serial
   * connection.
   */

  flags = enter_critical_section();

  /* Reset the configuration */

  cdcacm_resetconfig(priv);

  leave_critical_section(flags);

  /* Perform the soft connect function so that we will we can be
   * re-enumerated (unless we are part of a composite device)
   */

#ifndef CONFIG_USBHID_COMPOSITE
  DEV_CONNECT(dev);
#endif
}

/****************************************************************************
 * Name: cdcacm_suspend
 *
 * Description:
 *   Handle the USB suspend event.
 *
 ****************************************************************************/

static void cdcacm_suspend(FAR struct usbdevclass_driver_s *driver,
                           FAR struct usbdev_s *dev)
{
  usbtrace(TRACE_CLASSSUSPEND, 0);
}

/****************************************************************************
 * Name: cdcacm_resume
 *
 * Description:
 *   Handle the USB resume event.
 *
 ****************************************************************************/

static void cdcacm_resume(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev)
{
  usbtrace(TRACE_CLASSRESUME, 0);
}

/****************************************************************************
 * Public Functions
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
 *     /dev/ttyACM0.
 *   classdev - The location to return the CDC serial class' device
 *     instance.
 *
 * Returned Value:
 *   A pointer to the allocated class object (NULL on failure).
 *
 ****************************************************************************/

#ifndef CONFIG_USBHID_COMPOSITE
static
#endif
int usbhid_classobject(int minor, FAR struct usbdev_devinfo_s *devinfo,
                       FAR struct usbdevclass_driver_s **classdev)
{
  FAR struct cdcacm_alloc_s *alloc;
  FAR struct cdcacm_dev_s *priv;
  FAR struct cdcacm_driver_s *drvr;
  // char devname[USBHID_DEVNAME_SIZE];
  int ret;

  _err("entry\n");
  /* Allocate the structures needed */

  alloc = (FAR struct cdcacm_alloc_s *)
    kmm_malloc(sizeof(struct cdcacm_alloc_s));

  if (!alloc)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCDEVSTRUCT), 0);
      return -ENOMEM;
    }

  /* Convenience pointers into the allocated blob */

  priv = &alloc->dev;
  drvr = &alloc->drvr;

  /* Initialize the USB serial driver structure */

  memset(priv, 0, sizeof(struct cdcacm_dev_s));

  priv->minor               = minor;

  // priv->test_move_timer     = wd_create();

#ifdef CONFIG_USBHID_COMPOSITE
  /* Save the caller provided device description (composite only) */

  memcpy(&priv->devinfo, devinfo,
         sizeof(struct usbdev_devinfo_s));
#endif

  /* Initialize the USB class driver structure */

#ifdef CONFIG_USBDEV_DUALSPEED
  drvr->drvr.speed          = USB_SPEED_HIGH;
#else
  drvr->drvr.speed          = USB_SPEED_FULL;
#endif
  drvr->drvr.ops            = &g_driverops;
  drvr->dev                 = priv;

  *classdev = &drvr->drvr;
  return OK;

  kmm_free(alloc);
  return ret;
}

/****************************************************************************
 * Name: usbhid_initialize
 *
 * Description:
 *   Register USB serial port (and USB serial console if so configured).
 *
 * Input Parameters:
 *   minor - Device minor number.  E.g., minor 0 would correspond to
 *     /dev/ttyACM0.
 *   handle - An optional opaque reference to the CDC/ACM class object that
 *     may subsequently be used with usbhid_uninitialize().
 *
 * Returned Value:
 *   Zero (OK) means that the driver was successfully registered.  On any
 *   failure, a negated errno value is returned.
 *
 ****************************************************************************/

#ifndef CONFIG_USBHID_COMPOSITE
int usbhid_initialize(int minor, FAR void **handle)
{
  FAR struct usbdevclass_driver_s *drvr = NULL;
  struct usbdev_devinfo_s devinfo;
  int ret;

  memset(&devinfo, 0, sizeof(struct usbdev_devinfo_s));

  /* Interfaces.
   *
   * ifnobase must be provided by board-specific logic
   */

  devinfo.ninterfaces = USBHID_NINTERFACES; /* Number of interfaces in the configuration */

  /* Strings.
   *
   * strbase must be provided by board-specific logic
   */

  devinfo.nstrings    = USBHID_NSTRIDS;     /* Number of Strings */

  /* Endpoints.
   *
   * Endpoint numbers must be provided by board-specific logic when
   * CDC/ACM is used in a composite device.
   */

  devinfo.nendpoints  = USBHID_NUM_EPS;
#ifndef CONFIG_USBHID_COMPOSITE
  devinfo.epno[USBHID_EP_INTIN_IDX]   = CONFIG_USBHID_EPINTIN;
#endif

  /* Get an instance of the serial driver class object */

  ret = usbhid_classobject(minor, &devinfo, &drvr);
  if (ret == OK)
    {
      /* Register the USB serial class driver */

      ret = usbdev_register(drvr);
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_DEVREGISTER),
                   (uint16_t)-ret);
        }
    }

  /* Return the driver instance (if any) if the caller has requested it
   * by provided a pointer to the location to return it.
   */

  if (handle)
    {
      *handle = (FAR void *)drvr;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: usbhid_uninitialize
 *
 * Description:
 *   Un-initialize the USB storage class driver.  This function is used
 *   internally by the USB composite driver to uninitialize the CDC/ACM
 *   driver.  This same interface is available (with an untyped input
 *   parameter) when the CDC/ACM driver is used standalone.
 *
 * Input Parameters:
 *   There is one parameter, it differs in typing depending upon whether the
 *   CDC/ACM driver is an internal part of a composite device, or a standalone
 *   USB driver:
 *
 *     classdev - The class object returned by usbhid_classobject()
 *     handle - The opaque handle representing the class object returned by
 *       a previous call to usbhid_initialize().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_USBHID_COMPOSITE
void usbhid_uninitialize(FAR struct usbdevclass_driver_s *classdev)
#else
void usbhid_uninitialize(FAR void *handle)
#endif
{
#ifdef CONFIG_USBHID_COMPOSITE
  FAR struct cdcacm_driver_s *drvr = (FAR struct cdcacm_driver_s *)classdev;
#else
  FAR struct cdcacm_driver_s *drvr = (FAR struct cdcacm_driver_s *)handle;
#endif
  FAR struct cdcacm_dev_s    *priv = drvr->dev;

#ifdef CONFIG_USBHID_COMPOSITE
  /* Check for pass 2 uninitialization.  We did most of the work on the
   * first pass uninitialization.
   */

  if (priv->minor == (uint8_t)-1)
    {
      /* In this second and final pass, all that remains to be done is to
       * free the memory resources.
       */

      kmm_free(priv);
      return;
    }
#endif

  /* Unregister the driver (unless we are a part of a composite device).  The
   * device unregister logic will (1) return all of the requests to us then
   * (2) all the unbind method.
   *
   * The same thing will happen in the composite case except that: (1) the
   * composite driver will call usbdev_unregister() which will (2) return the
   * requests for all members of the composite, and (3) call the unbind
   * method in the composite device which will (4) call the unbind method
   * for this device.
   */

#ifndef CONFIG_USBHID_COMPOSITE
  usbdev_unregister(&drvr->drvr);

  /* And free the driver structure */

  kmm_free(priv);

#else
  /* For the case of the composite driver, there is a two pass
   * uninitialization sequence.  We cannot yet free the driver structure.
   * We will do that on the second pass.  We mark the fact that we have
   * already uninitialized by setting the minor number to -1.  If/when we
   * are called again, then we will free the memory resources.
   */

  priv->minor = (uint8_t)-1;
#endif
}

/****************************************************************************
 * Name: cdcacm_get_composite_devdesc
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
void usbhid_get_composite_devdesc(struct composite_devdesc_s *dev)
{
  memset(dev, 0, sizeof(struct composite_devdesc_s));

  /* The callback functions for the CDC/ACM class.
   *
   * classobject() and uninitialize() must be provided by board-specific
   * logic
   */

  dev->mkconfdesc   = usbhid_mkcfgdesc;
  dev->mkstrdesc    = usbhid_mkstrdesc;

  dev->classobject  = usbhid_classobject;
  dev->uninitialize = usbhid_uninitialize;

  dev->nconfigs     = USBHID_NCONFIGS;           /* Number of configurations supported */
  dev->configid     = USBHID_CONFIGID;           /* The only supported configuration ID */

  /* Let the construction function calculate the size of the config descriptor */

#ifdef CONFIG_USBDEV_DUALSPEED
  dev->cfgdescsize  = usbhid_mkcfgdesc(NULL, NULL, USB_SPEED_UNKNOWN, 0);
#else
  dev->cfgdescsize  = usbhid_mkcfgdesc(NULL, NULL);
#endif

  /* Board-specific logic must provide the device minor */

  /* Interfaces.
   *
   * ifnobase must be provided by board-specific logic
   */

  dev->devinfo.ninterfaces = USBHID_NINTERFACES; /* Number of interfaces in the configuration */

  /* Strings.
   *
   * strbase must be provided by board-specific logic
   */

  dev->devinfo.nstrings    = USBHID_NSTRIDS;     /* Number of Strings */

  /* Endpoints.
   *
   * Endpoint numbers must be provided by board-specific logic.
   */

  dev->devinfo.nendpoints  = USBHID_NUM_EPS;
}
#endif
