/****************************************************************************
 * arch/sim/src/sim/up_libusb.c
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
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_devaddr.h>
#include <nuttx/usb/usbhost_trace.h>

#include <libusb-1.0/libusb.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_USB_PORTS 10
#define CONFIG_SIM_USBHOST_DESCSIZE (64 * 1024)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_usbhost_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to struct sim_usbhost_s.
   */

  struct usbhost_driver_s drvr;
};

struct sim_libusb_device_s
{
  FAR struct usbhost_hubport_s hport;
  libusb_device *dev;
  libusb_device_handle *handle;
};

struct sim_ep_s
{
  libusb_device_handle  *handle;  /* libusb device handle */
  uint8_t                epno;    /* Device endpoint number (0-127) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sim_wait(FAR struct usbhost_connection_s *conn,
                    FAR struct usbhost_hubport_s **hport);
static int sim_enumerate(FAR struct usbhost_connection_s *conn,
                         FAR struct usbhost_hubport_s *hport);

static int sim_ep0configure(FAR struct usbhost_driver_s *drvr,
                              usbhost_ep_t ep0, uint8_t funcaddr,
                              uint8_t speed, uint16_t maxpacketsize);
static int sim_epalloc(FAR struct usbhost_driver_s *drvr,
                       FAR const FAR struct usbhost_epdesc_s *epdesc,
                       FAR usbhost_ep_t *ep);
static int sim_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static int sim_alloc(FAR struct usbhost_driver_s *drvr,
                     FAR uint8_t **buffer, FAR size_t *maxlen);
static int sim_free(FAR struct usbhost_driver_s *drvr,
                    FAR uint8_t *buffer);
static int sim_ioalloc(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t **buffer, size_t buflen);
static int sim_iofree(FAR struct usbhost_driver_s *drvr,
                      FAR uint8_t *buffer);
static int sim_ctrlin(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                      const struct usb_ctrlreq_s *req,
                      FAR uint8_t *buffer);
static int sim_ctrlout(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                       FAR const struct usb_ctrlreq_s *req,
                       FAR const uint8_t *buffer);
static ssize_t sim_transfer(FAR struct usbhost_driver_s *drvr,
                            usbhost_ep_t ep, FAR uint8_t *buffer,
                            size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static int sim_asynch(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                      FAR uint8_t *buffer, size_t buflen,
                      usbhost_asynch_t callback, FAR void *arg);
#endif
static int sim_cancel(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);
#ifdef CONFIG_USBHOST_HUB
static int sim_connect(FAR struct usbhost_driver_s *drvr,
                       FAR struct usbhost_hubport_s *hport,
                       bool connected);
#endif
static void sim_disconnect(FAR struct usbhost_driver_s *drvr,
                           FAR struct usbhost_hubport_s *hport);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* In this driver implementation, support is provided for only a single a
 * single USB device.  All status information can be simply retained in a
 * single global instance.
 */

static struct sim_usbhost_s g_usbhost;

/* This is the connection/enumeration interface */

static struct usbhost_connection_s g_usbconn =
{
  .wait             = sim_wait,
  .enumerate        = sim_enumerate,
};

static struct sim_libusb_device_s g_libusb_device;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 ****************************************************************************/

static inline uint16_t usbhost_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: usbhost_putle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 ****************************************************************************/

static void usbhost_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/****************************************************************************
 * Name: sim_wait
 *
 * Description:
 *   Wait for a device to be connected or disconnected to/from a hub port.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from
 *     the call to the USB driver initialization logic.
 *   hport - The location to return the hub port descriptor that detected
 *     the connection related event.
 *
 * Returned Value:
 *   Zero (OK) is returned on success when a device is connected or
 *   disconnected. This function will not return until either (1) a device
 *   is connected or disconnect to/from any hub port or until (2) some
 *   failure occurs.  On a failure, a negated errno value is returned
 *   indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int sim_wait(FAR struct usbhost_connection_s *conn,
                      FAR struct usbhost_hubport_s **hport)
{
  libusb_device **list;
  libusb_device *dev;
  int i;
  int ret = 0;
  struct sim_ep_s *ep0;

  while(1)
  {
    ssize_t cnt = libusb_get_device_list(NULL, &list);
    if (cnt < 0)
    {
      uerr("Failed to get the device list: %d.\n", cnt);
      nxsig_sleep(1);
      continue;
    }

    i = 0;
    while ((dev = list[i++]) != NULL)
    {
      struct libusb_device_descriptor desc;
      int r = libusb_get_device_descriptor(dev, &desc);
      if (r < 0)
      {
        uinfo("failed to get device descriptor\n");
        continue;
      }
#if 0
      uinfo("%04x:%04x (bus %d, device  %d)\n",
        desc.idVendor, desc.idProduct,
        libusb_get_bus_number(dev), libusb_get_device_address(dev));
#endif      
      if (desc.idVendor == 0x046d && desc.idProduct == 0xc52b)
      { 
        if (g_libusb_device.dev == NULL)
        {
          uinfo("Found magic device\n");
          uinfo("%04x:%04x (bus %d, device %d)\n",
            desc.idVendor, desc.idProduct,
            libusb_get_bus_number(dev), libusb_get_device_address(dev));
          ret = libusb_open(dev, &g_libusb_device.handle);
          if (ret < 0)
          {
            continue;
          }
          uinfo("Magic device connection status changed\n");
          g_libusb_device.dev = libusb_ref_device(dev);
          g_libusb_device.hport.connected = true;
          goto ret_free;
        }
      }
    }
    libusb_free_device_list(list, true);
    nxsig_sleep(1);
  }

ret_free:
  libusb_free_device_list(list, true);
  g_libusb_device.hport.drvr = (struct usbhost_driver_s *)&g_usbhost;

  ep0 = kmm_malloc(sizeof(struct sim_ep_s));
  if (ep0 == NULL)
  {
    return -1;
  }
  ep0->handle = g_libusb_device.handle;
  ret = libusb_set_auto_detach_kernel_driver(ep0->handle, 1);
  if (ret != 0)
    {
      uerr("Failed to auto-detach from kernel: %d\n", ret);
    }
  g_libusb_device.hport.ep0 = (usbhost_ep_t)ep0;

  *hport = &g_libusb_device.hport;
  return OK;
}

/****************************************************************************
 * Name: sim_enumerate
 *
 * Description:
 *   Enumerate the connected device.  As part of this enumeration process,
 *   the driver will (1) get the device's configuration descriptor, (2)
 *   extract the class ID info from the configuration descriptor, (3) call
 *   usbhost_findclass() to find the class that supports this device, (4)
 *   call the create() method on the struct usbhost_registry_s interface
 *   to get a class instance, and finally (5) call the connect() method
 *   of the struct usbhost_class_s interface.  After that, the class is in
 *   charge of the sequence of operations.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from
 *      the call to the USB driver initialization logic.
 *   hport - The descriptor of the hub port that has the newly connected
 *      device.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sim_enumerate(FAR struct usbhost_connection_s *conn,
                         FAR struct usbhost_hubport_s *hport)
{
  FAR struct sim_usbhost_s *priv = &g_usbhost;
  libusb_device_handle* dev_handle = ((struct sim_ep_s *)hport->ep0)->handle;
  int ret;

  DEBUGASSERT(hport);

  /* Then let the common usbhost_enumerate do the real enumeration. */

  uinfo("Enumerate the device\n");
  //priv->smstate = SMSTATE_ENUM;

  ret = usbhost_enumerate(hport, &hport->devclass);

  /* The enumeration may fail either because of some HCD interfaces failure
   * or because the device class is not supported.  In either case, we just
   * need to perform the disconnection operation and make ready for a new
   * enumeration.
   */

  if (ret < 0)
    {
      /* Return to the disconnected state */

      uerr("ERROR: Enumeration failed: %d\n", ret);
      //sim_gint_disconnected(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: sim_ep0configure
 *
 * Description:
 *   Configure endpoint 0.  This method is normally used internally by the
 *   enumerate() method but is made available at the interface to support
 *   an external implementation of the enumeration logic.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   funcaddr - The USB address of the function containing the endpoint that
 *     EP0 controls.  A funcaddr of zero will be received if no address is
 *     yet assigned to the device.
 *   speed - The speed of the port USB_SPEED_LOW, _FULL, or _HIGH
 *   maxpacketsize - The maximum number of bytes that can be sent to or
 *    received from the endpoint in a single data packet
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sim_ep0configure(FAR struct usbhost_driver_s *drvr,
                            usbhost_ep_t ep0, uint8_t funcaddr,
                            uint8_t speed, uint16_t maxpacketsize)
{
  return OK;
}

/****************************************************************************
 * Name: sim_epalloc
 *
 * Description:
 *   Allocate and configure one endpoint.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sim_epalloc(FAR struct usbhost_driver_s *drvr,
                       FAR const struct usbhost_epdesc_s *epdesc,
                       FAR usbhost_ep_t *ep)
{
  struct libusb_config_descriptor *config;
  struct libusb_device_handle *handle = \
    ((struct sim_ep_s *)epdesc->hport->ep0)->handle;
  
  struct sim_ep_s *sim_ep;
  int ifno;                 /* Interface Number */
  bool found_if = false;    /* Found the endpoint in the current config */
  int ret;

  /* The interface number is not availible through the endpoint descriptor
   * struct so we need to work our way though the config descriptor to
   * find it. This should be cached by the os, so it should be cheap. */

  if(libusb_get_active_config_descriptor(
      libusb_get_device(handle),
      &config) != 0)
    {
      return -1;
    }

  for (ifno = 0; ifno < config->bNumInterfaces; ifno++)
    {
      uint8_t ep_idx;
      const struct libusb_interface_descriptor* intf = \
        config->interface[ifno].altsetting;

      for(ep_idx = 0; ep_idx < intf->bNumEndpoints; ep_idx++)
        {
          uint8_t ep_addr = epdesc->in ? epdesc->addr | 0x80 : epdesc->addr;
          if(intf->endpoint[ep_idx].bEndpointAddress == ep_addr)
            {
              found_if = true;
              ret = libusb_claim_interface(handle, ifno);
              if (ret != 0)
                {
                  uerr("Failed to claim interface: %d\n", ret);
                  return -1;
                }
              break;
            }
        }

      if (found_if)
        {
          break;
        }
    }

  if (!found_if)
    {
      uerr("Could not find expected endpoint in active config.\n");
      return -1;
    }

  sim_ep = kmm_malloc(sizeof(struct sim_ep_s));
  if(sim_ep == NULL)
    {
      return -1;
    }

  *ep = (usbhost_ep_t)sim_ep;

  sim_ep->handle = handle;
  sim_ep->epno = epdesc->addr;
  if (epdesc->in)
    {
      sim_ep->epno |= USB_EP_ADDR_DIR_MASK;
    }

  return OK;
}

/****************************************************************************
 * Name: sim_epfree
 *
 * Description:
 *   Free and endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   ep - The endpoint to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sim_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  kmm_free(ep);
  return OK;
}

/****************************************************************************
 * Name: sim_alloc
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.  This method provides a
 *   mechanism to allocate the request/descriptor memory.  If the underlying
 *   hardware does not support such "special" memory, this functions may
 *   simply map to kmm_malloc.
 *
 *   This interface was optimized under a particular assumption.  It was
 *   assumed that the driver maintains a pool of small, pre-allocated
 *   buffers for descriptor traffic.  NOTE that size is not an input, but
 *   an output:  The size of the pre-allocated buffer is returned.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   buffer - The address of a memory location provided by the caller in
 *     which to return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by the caller in
 *     which to return the maximum size of the allocated buffer memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int sim_alloc(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t **buffer, FAR size_t *maxlen)
{
  FAR uint8_t *alloc;

  DEBUGASSERT(drvr && buffer && maxlen);

  /* There is no special memory requirement for the EFM32. */

  alloc = (FAR uint8_t *)kmm_malloc(CONFIG_SIM_USBHOST_DESCSIZE);
  if (!alloc)
    {
      return -ENOMEM;
    }

  /* Return the allocated address and size of the descriptor buffer */

  *buffer = alloc;
  *maxlen = CONFIG_SIM_USBHOST_DESCSIZE;
  return OK;
}

/****************************************************************************
 * Name: sim_free
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.  This method provides a
 *   mechanism to free that request/descriptor memory.  If the underlying
 *   hardware does not support such "special" memory, this functions may
 *   simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int sim_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  /* There is no special memory requirement */

  DEBUGASSERT(drvr && buffer);
  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: sim_ioalloc
 *
 * Description:
 *   Some hardware supports special memory in which larger IO buffers can
 *   be accessed more efficiently.  This method provides a mechanism to
 *   allocate the request/descriptor memory.  If the underlying hardware
 *   does not support such "special" memory, this functions may simply map
 *   to kmm_malloc.
 *
 *   This interface differs from DRVR_ALLOC in that the buffers are
 *   variable-sized.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   buffer - The address of a memory location provided by the caller in
 *     which to return the allocated buffer memory address.
 *   buflen - The size of the buffer required.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sim_ioalloc(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t **buffer, size_t buflen)
{
  FAR uint8_t *alloc;

  DEBUGASSERT(drvr && buffer && buflen > 0);

  /* There is no special memory requirement */

  alloc = (FAR uint8_t *)kmm_malloc(buflen);
  if (!alloc)
    {
      return -ENOMEM;
    }

  /* Return the allocated buffer */

  *buffer = alloc;
  return OK;
}

/****************************************************************************
 * Name: sim_iofree
 *
 * Description:
 *   Some hardware supports special memory in which IO data can  be accessed
 *   more efficiently.  This method provides a mechanism to free that IO
 *   buffer memory.  If the underlying hardware does not support such
 *   "special" memory, this functions may simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sim_iofree(FAR struct usbhost_driver_s *drvr,
                      FAR uint8_t *buffer)
{
  /* There is no special memory requirement */

  DEBUGASSERT(drvr && buffer);
  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: sim_ctrlin and sim_ctrlout
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  These methods
 *   will enqueue the request and wait for it to complete.  Only one
 *   transfer may be queued; Neither these methods nor the transfer()
 *   method can be called again until the control transfer functions
 *   returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   ep0 - The control endpoint to send/receive the control request.
 *   req - Describes the request to be sent.  This request must lie in memory
 *      created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *     responses.  This buffer must be large enough to hold the length value
 *     in the request description. buffer must have been allocated using
 *     DRVR_ALLOC.
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same
 *   allocated memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int sim_ctrlin(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                        FAR const struct usb_ctrlreq_s *req,
                        FAR uint8_t *buffer)
{
  libusb_device_handle *handle = ((struct sim_ep_s *)ep0)->handle;
  unsigned int timeout = 0;
  libusb_control_transfer(
    handle, 
		req->type,
		req->req,
		usbhost_getle16(req->value),
		usbhost_getle16(req->index),
		buffer,
		usbhost_getle16(req->len),
		timeout 
	);

  return OK;
}

static int sim_ctrlout(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                         FAR const struct usb_ctrlreq_s *req,
                         FAR const uint8_t *buffer)
{
  return OK;
}

/****************************************************************************
 * Name: sim_transfer
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request, blocking until the transfer completes.
 *   Only one transfer may be  queued; Neither this method nor the ctrlin or
 *   ctrlout methods can be called again until this function returns.
 *
 *   This is a blocking method; this functions will not return until the
 *   transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on
 *     which to perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or
 *     received (IN endpoint).  buffer must have been allocated using
 *     DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure:
 *
 *     EAGAIN - If devices NAKs the transfer (or NYET or other error where
 *              it may be appropriate to restart the entire transaction).
 *     EPERM  - If the endpoint stalls
 *     EIO    - On a TX or data toggle error
 *     EPIPE  - Overrun errors
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static ssize_t sim_transfer(FAR struct usbhost_driver_s *drvr,
                            usbhost_ep_t ep, FAR uint8_t *buffer,
                            size_t buflen)
{
  struct sim_ep_s *sim_ep = (struct sim_ep_s *)ep;
  int actual_length;
  int ret;
  unsigned int timeout = 100;
  ret = libusb_bulk_transfer(
    sim_ep->handle,
    sim_ep->epno,
    buffer,
    buflen,
    &actual_length,
    timeout
    );

  if (ret < 0 && ret != LIBUSB_ERROR_TIMEOUT)
    {
      return -EIO;
    }
  
  if (actual_length > 0)
    {
      uinfo("Transfered: %d\n", actual_length);
    }

  return actual_length;
}

/****************************************************************************
 * Name: sim_cancel
 *
 * Description:
 *   Cancel a pending transfer on an endpoint.  Cancelled synchronous or
 *   asynchronous transfer will complete normally with the error -ESHUTDOWN.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which
 *     an asynchronous transfer should be transferred.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 ****************************************************************************/

static int sim_cancel(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  /* Not Implemented */

  return OK;
}

/****************************************************************************
 * Name: sim_disconnect
 *
 * Description:
 *   Called by the class when an error occurs and driver has been
 *   disconnected.  The USB host driver should discard the handle to the
 *   class instance (it is stale) and not attempt any further interaction
 *   with the class driver instance (until a new instance is received from
 *   the create() method).  The driver should not called the class'
 *   disconnected() method.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   hport - The port from which the device is being disconnected.  Might be
 *      a port on a hub.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static void sim_disconnect(FAR struct usbhost_driver_s *drvr,
                           FAR struct usbhost_hubport_s *hport)
{
  DEBUGASSERT(hport != NULL);
  hport->devclass = NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_usbhost_initialize
 *
 * Description:
 *   Initialize USB host device controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
 *     this identifies which controller is being initialized.  Normally, this
 *     is just zero.
 *
 * Returned Value:
 *   And instance of the USB host interface.  The controlling task should
 *   use this interface to (1) call the wait() method to wait for a device
 *   to be connected, and (2) call the enumerate() method to bind the device
 *   to a class driver.
 *
 * Assumptions:
 * - This function should called in the initialization sequence in order
 *   to initialize the USB device functionality.
 * - Class drivers should be initialized prior to calling this function.
 *   Otherwise, there is a race condition if the device is already connected.
 *
 ****************************************************************************/

FAR struct usbhost_connection_s *sim_usbhost_initialize(int controller)
{
  FAR struct sim_usbhost_s *priv = &g_usbhost;
  FAR struct usbhost_driver_s *drvr;

  /* Initialize the device operations */
  drvr                 = &priv->drvr;
  drvr->ep0configure   = sim_ep0configure;
  drvr->epalloc        = sim_epalloc;
  drvr->epfree         = sim_epfree;
  drvr->alloc          = sim_alloc;
  drvr->free           = sim_free;
  drvr->ioalloc        = sim_ioalloc;
  drvr->iofree         = sim_iofree;
  drvr->ctrlin         = sim_ctrlin;
  drvr->ctrlout        = sim_ctrlout;
  drvr->transfer       = sim_transfer;
#ifdef CONFIG_USBHOST_ASYNCH
  drvr->asynch         = sim_asynch;
#endif
  drvr->cancel         = sim_cancel;
#ifdef CONFIG_USBHOST_HUB
  drvr->connect        = sim_connect;
#endif
  drvr->disconnect     = sim_disconnect;

  libusb_init(NULL);

  /* Initialize the device operations */
  return &g_usbconn;
}
