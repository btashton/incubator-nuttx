/****************************************************************************
 * arch/risc-v/src/servant/servant_lowputc.c
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

#include <stdint.h>

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "riscv_arch.h"

#include "hardware/servant_memorymap.h"
#include "servant.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void up_lowputc(char ch)
{
//#ifdef HAVE_SERIAL_CONSOLE
  int cout = (ch | 0x100) << 1;

  irqstate_t flags = enter_critical_section();
  do
    {
      (*(volatile uint32_t *)SERVANT_UART_BASE) = cout;
      cout >>= 1;
      asm volatile ("nop");
      asm volatile ("nop");
    } while (cout);
  leave_critical_section(flags);

//#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Name: servant_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void servant_lowsetup(void)
{
  (*(volatile uint32_t *)SERVANT_UART_BASE) = 1;
}
