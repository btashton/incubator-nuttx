/****************************************************************************
 * arch/risc-v/src/litex/hardware/litex_plic.h
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

#ifndef __ARCH_RISCV_SRC_LITEX_HARDWARE_LITEX_PLIC_H
#define __ARCH_RISCV_SRC_LITEX_HARDWARE_LITEX_PLIC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* litex vexRiscv does not follow RISC-V privileged specification and
 * uses two additional CSRs: mask and pending.
 */
#define LITEX_MMASK_CSR     0xBC0     /* From LiteX CSR_IRQ_MASK */ 
#define LITEX_MPENDING_CSR     0xFC0  /* from LiteX CSR_IRQ_PENDING */

#endif /* __ARCH_RISCV_SRC_LITEX_HARDWARE_LITEX_PLIC_H */
