/****************************************************************************
 * boards/sim/sim/sim/src/sim_usb.c
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
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kthread.h>
#include <nuttx/usb/usbhost.h>

#include "sim.h"
#include "up_usb.h"

#ifdef CONFIG_SIM_USBHOST

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_USBHOST_DEFPRIO
#  define CONFIG_USBHOST_DEFPRIO 100
#endif

#ifndef CONFIG_USBHOST_STACKSIZE
#  define CONFIG_USBHOST_STACKSIZE (64 * 1024)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct usbhost_connection_s *g_usbconn;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_waiter
 *
 * Description:
 *   Wait for USB devices to be connected.
 *
 ****************************************************************************/

static int usbhost_waiter(int argc, char *argv[])
{
  struct usbhost_hubport_s *hport;

  uinfo("Running\n");
  for (; ; )
    {
      /* Wait for the device to change state */

      DEBUGVERIFY(CONN_WAIT(g_usbconn, &hport));
      uinfo("%s\n", hport->connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (hport->connected)
        {
          /* Yes.. enumerate the newly connected device */

          CONN_ENUMERATE(g_usbconn, hport);
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ****************************************************************************/

int sim_usb_initialize(void)
{
  int pid;
#if defined(CONFIG_USBHOST_HUB)    || defined(CONFIG_USBHOST_MSC) || \
    defined(CONFIG_USBHOST_HIDKBD) || defined(CONFIG_USBHOST_HIDMOUSE)
  int ret;
#endif

  /* First, register all of the class drivers needed to support the drivers
   * that we care about:
   */

  uinfo("Register class drivers\n");

#ifdef CONFIG_USBHOST_HUB
  /* Initialize USB hub class support */

  ret = usbhost_hub_initialize();
  if (ret < 0)
    {
      uerr("ERROR: usbhost_hub_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_MSC
  /* Register the USB mass storage class class */

  ret = usbhost_msc_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the mass storage class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_CDCACM
  /* Register the CDC/ACM serial class */

  ret = usbhost_cdcacm_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the CDC/ACM serial class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_HIDKBD
  /* Initialize the HID keyboard class */

  ret = usbhost_kbdinit();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the HID keyboard class\n");
    }
#endif

#ifdef CONFIG_USBHOST_HIDMOUSE
  /* Initialize the HID mouse class */

  ret = usbhost_mouse_init();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the HID mouse class\n");
    }
#endif

  /* Then get an instance of the USB host interface */

  uinfo("Initialize USB host\n");
  g_usbconn = sim_usbhost_initialize(0);
  if (g_usbconn)
    {
      /* Start a thread to handle device connection. */

      uinfo("Start usbhost_waiter\n");

      pid = kthread_create("usbhost", CONFIG_USBHOST_DEFPRIO,
                           CONFIG_USBHOST_STACKSIZE,
                           (main_t)usbhost_waiter, (FAR char * const *)NULL);
      return pid < 0 ? -ENOEXEC : OK;
    }

  return -ENODEV;
}

#endif /* CONFIG_SIM_USBHOST */
