/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 1995 - 2024 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       SEGGER SystemView * Real-time application analysis           *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* SEGGER strongly recommends to not make any changes                 *
* to or modify the source code of this software in order to stay     *
* compatible with the SystemView and RTT protocol, and J-Link.       *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* condition is met:                                                  *
*                                                                    *
* o Redistributions of source code must retain the above copyright   *
*   notice, this condition and the following disclaimer.             *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************
-------------------------- END-OF-HEADER -----------------------------

File    : SEGGER_SYSVIEW_Conf.h
Purpose : SEGGER SystemView configuration file.
          Set defines which deviate from the defaults (see SEGGER_SYSVIEW_ConfDefaults.h) here.
Revision: $Rev: 21292 $

Additional information:
  Required defines which must be set are:
    SEGGER_SYSVIEW_GET_TIMESTAMP
    SEGGER_SYSVIEW_GET_INTERRUPT_ID
  For known compilers and cores, these might be set to good defaults
  in SEGGER_SYSVIEW_ConfDefaults.h.

  SystemView needs a (nestable) locking mechanism.
  If not defined, the RTT locking mechanism is used,
  which then needs to be properly configured.
*/

#ifndef SEGGER_SYSVIEW_CONF_H
#define SEGGER_SYSVIEW_CONF_H

/*********************************************************************
 *
 *       Defines, configurable
 *
 **********************************************************************
 */

/*********************************************************************
 *
 *       Define: SEGGER_SYSVIEW_SECTION
 *
 *  Description
 *    Section to place the SystemView RTT Buffer into.
 *    MUST be non-cacheable SRAM for J-Link SWD coherence:
 *    - DTCM:     safe (default .bss) but precious, 0-wait
 *    - RAM_D2:   safe on STM32H7, AHB bus, uncached, 32KB available
 *    - .bss:     use for targets without a linker-defined RAM_D2 section
 *    - RAM_D1:   DANGER – cached via L1 D$, J-Link sees stale data
 */
#if defined(STM32H723xx)
#define SEGGER_SYSVIEW_SECTION ".RAM_D2"
#else
#define SEGGER_SYSVIEW_SECTION ".bss"
#endif

/*********************************************************************
 * TODO: Add your defines here.                                       *
 **********************************************************************
 */

/*********************************************************************
 *
 *       SystemView description strings
 *
 *  Notes
 *    These override the defaults in SEGGER_SYSVIEW_ConfDefaults.h.
 *    The device name is used in the target description sent to the
 *    host. Change per board if needed (e.g. "Cortex-M4" / "Cortex-M7").
 */
#ifndef SEGGER_SYSVIEW_APP_NAME
#define SEGGER_SYSVIEW_APP_NAME "ThreadX Application"
#endif

#ifndef SEGGER_SYSVIEW_DEVICE_NAME
#define SEGGER_SYSVIEW_DEVICE_NAME "Cortex-M"
#endif

/*********************************************************************
 *
 *       Event filter mask
 *
 *  No SEGGER_SYSVIEW_DISABLE_EVENT_MASK is defined here, so all
 *  SystemView event categories remain enabled.
 */
/* RAM base where ThreadX objects (TCBs, semaphores, etc.) live.
 * On STM32H7, .data/.bss default to DTCMRAM (0x20000000).
 * Must match the lowest RAM addr used by traced objects so
 * SystemView can correctly compress and match IDs. */
#ifndef SEGGER_SYSVIEW_ID_BASE
#define SEGGER_SYSVIEW_ID_BASE 0x20000000
#endif

/*********************************************************************
 *
 *       RTT channel configuration for SystemView
 *
 *  Notes
 *    Channel 0 is used by default. If another module (e.g. ulog)
 *    already uses channel 0, set a different channel here.
 */
#ifndef SEGGER_SYSVIEW_RTT_CHANNEL
#define SEGGER_SYSVIEW_RTT_CHANNEL 1
#endif

/*********************************************************************
 *
 *       SystemView buffer size
 *
 *  Notes
 *    Buffer size for SystemView Up/Down RTT buffers.
 *    Increase if overflow events appear in SystemView.
 *    Typical: 1024 (debug), 4096-8192 (production trace).
 *    Each event is ~1-10 bytes encoded; at 1 kHz task switch
 *    rate, 8 KB holds ~1 second of trace.
 */
#ifndef SEGGER_SYSVIEW_RTT_BUFFER_SIZE
#define SEGGER_SYSVIEW_RTT_BUFFER_SIZE 8192
#endif

#endif // SEGGER_SYSVIEW_CONF_H

/*************************** End of file ****************************/
