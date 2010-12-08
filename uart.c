/*
 * Copyright (c) 2010
 * Telecooperation Office (TecO), Universitaet Karlsruhe (TH), Germany.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. Neither the name of the Universitaet Karlsruhe (TH) nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author(s): Philipp Scholl <scholl@teco.edu>
 */

#include "contiki.h"
#include "contiki-net.h"
#include "gdb2.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <AppHardwareApi.h>

#define GREETING "This is uart0, please select the operation mode like this:\n"\
                 "115200 8N1 rtscts\n"

PROCESS(uart_process, "Uart Process");
AUTOSTART_PROCESSES(&uart_process);

static flowcontrol = false;

/* NOTE: also change rts/cts/irq macro when changing uart */
#define UART E_AHI_UART_0

#if UART == E_AHI_UART_1
#define RTS  E_AHI_DIO18_INT
#define CTS  E_AHI_DIO17_INT
#else
#define RTS  E_AHI_DIO5_INT
#define CTS  E_AHI_DIO4_INT
#endif

#define rts(on) do { if(flowcontrol) vAHI_DioSetOutput(on?RTS:0, on?0:RTS); } while(0);
#define cts()   (flowcontrol ? (u32AHI_DioReadInput()&CTS) : 1)

unsigned int
atoi(const char *str, const char **retstr)
{
  int i;
  unsigned int num = 0;
  const char *strptr = str;

  if(str == NULL) {
    return 0;
  }

  while(isblank(*strptr)) {
    ++strptr;
  }

  for(i = 0; i < 10 && isdigit(strptr[i]); ++i) {
    num = num * 10 + strptr[i] - '0';
  }
  if(retstr != NULL) {
    if(i == 0) {
      *retstr = str;
    } else {
      *retstr = strptr + i;
    }
  }

  return num;
}

#define UART_LCR_OFFSET   0x0C
#define UART_DLM_OFFSET   0x04
#define UART_EER_OFFSET   0x20

void
vAHI_UartSetBaudrate(uint32_t handle, uint32_t u32BaudRate)
{
    uint8 *pu8Reg;
    uint8  u8TempLcr;
    uint16 u16Divisor;
    uint32 u32Remainder;
    uint32 UART_START_ADR;

    if(handle==E_AHI_UART_0) UART_START_ADR=0x30000000UL;
    if(handle==E_AHI_UART_1) UART_START_ADR=0x40000000UL;

    /* Put UART into clock divisor setting mode */
    pu8Reg    = (uint8 *)(UART_START_ADR + UART_LCR_OFFSET);
    u8TempLcr = *pu8Reg;
    *pu8Reg   = u8TempLcr | 0x80;

    /* Write to divisor registers:
       Divisor register = 16MHz / (16 x baud rate) */
    u16Divisor = (uint16)(16000000UL / (16UL * u32BaudRate));

    /* Correct for rounding errors */
    u32Remainder = (uint32)(16000000UL % (16UL * u32BaudRate));

    if (u32Remainder >= ((16UL * u32BaudRate) / 2))
    {
        u16Divisor += 1;
    }

    pu8Reg  = (uint8 *)UART_START_ADR;
    *pu8Reg = (uint8)(u16Divisor & 0xFF);
    pu8Reg  = (uint8 *)(UART_START_ADR + UART_DLM_OFFSET);
    *pu8Reg = (uint8)(u16Divisor >> 8);

    /* Put back into normal mode */
    pu8Reg    = (uint8 *)(UART_START_ADR + UART_LCR_OFFSET);
    u8TempLcr = *pu8Reg;
    *pu8Reg   = u8TempLcr & 0x7F;
}

void
uart_init(long baudrate, long databits, char parity, long stopbits, char rtscts)
{
  bool evenparity, enableparity;

  switch(parity) {
  case 'E':
  case 'e':
    evenparity = enableparity = true;
    break;
  case 'O':
  case 'o':
    evenparity   = false;
    enableparity = true;
    break;
  case 'N':
  case 'n':
  default:
    evenparity = enableparity = false;
    break;
  }

  switch(databits)
  {
    case '5':
      databits = E_AHI_UART_WORD_LEN_5;
      break;
    case '6':
      databits = E_AHI_UART_WORD_LEN_6;
      break;
    case '7':
      databits = E_AHI_UART_WORD_LEN_7;
      break;
    case '8':
    default:
      databits = E_AHI_UART_WORD_LEN_8;
      break;
  }

  if (stopbits==2)
    stopbits = E_AHI_UART_2_STOP_BITS;
  else
    stopbits = E_AHI_UART_1_STOP_BIT;


  /* start the uart */
  vAHI_UartEnable(UART);
  vAHI_UartReset(UART, true, true);

  /* configure to 5mHz baudrate in 8-N-1 mode */
  vAHI_UartSetBaudrate(UART, baudrate);
  vAHI_UartSetControl(UART, evenparity, enableparity, databits, stopbits,
      E_AHI_UART_RTS_HIGH);
  //vAHI_UartSetBaudrate(UART, 115200);
  //vAHI_UartSetControl(UART, 1, 0, E_AHI_UART_WORD_LEN_8, E_AHI_UART_1_STOP_BIT, E_AHI_UART_RTS_HIGH);

  /* disable rts/cts line usage by uart, we control them by ourself
   * as there is no automatic flow ctrl and manual pin ctrl through
   * uart module does not work properly (w.r.t. the cts line) */
  vAHI_UartSetRTSCTS(UART, false);

  flowcontrol = rtscts;

  if (flowcontrol)
    vAHI_DioSetDirection(CTS, RTS);

  /* come out of reset */
  vAHI_UartReset(UART, false, false);
  rts(1);
}

size_t /* frame end-condition is SLIP_END*/
uart_read(u8_t *buf, size_t n)
{
  size_t i=0;

  while(u8AHI_UartReadLineStatus(UART)&E_AHI_UART_LS_DR)
    buf[i++] = u8AHI_UartReadData(UART);

  return i;
}

size_t
uart_write(u8_t *buf, size_t n)
{
  size_t i=0;

  if (n==0 || buf == NULL)
    return 0;

  while (i<n)
  {
    /* wait until CTS is asserted */
    while (!cts())
      ;

    /* wait until buffer space available */
    while ( !(u8AHI_UartReadLineStatus(UART) & E_AHI_UART_LS_THRE) )
      ;

    vAHI_UartWriteData(UART, buf[i++]);
  }

  return i;
}


PROCESS_THREAD(uart_process, ev, data)
{
  /* make sure the modeline fits! */
  static char sbuf[20], sbuf_len = 0;
  static bool configured = false;

  if (ev == tcpip_event && (uip_aborted() || uip_timedout() || uip_closed()))
  {
    /* this restarts this process */
    process_exit(&uart_process);
    process_start(&uart_process, NULL);
    sbuf_len = configured = 0;
  }

  PROCESS_BEGIN();
  tcp_listen(UIP_HTONS(4404));

  while (1)
  {
    PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event && uip_connected());

    /* send the greeting */
    do {
      uip_send(GREETING, sizeof(GREETING));
      PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
    } while (!uip_acked());

    /* wait for the configuration line and store it in the sbuf. */
    do {
      PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event && uip_newdata());

      /* either copy in the whole uip_appdata, move already received bytes to make
       * space or just add to the end */
      if (uip_datalen() > sizeof(sbuf))
      {
        /* fill whole buffer with the end of the rxd buffer */
        memcpy(sbuf, uip_appdata+(uip_datalen()%sizeof(sbuf)), sizeof(sbuf));
        sbuf_len = sizeof(sbuf);
      }
      else if (uip_datalen()+sbuf_len <= sizeof(sbuf))
      {
        /* fill up the buffer with available data */
        memcpy(sbuf+sbuf_len, uip_appdata, uip_datalen());
        sbuf_len += uip_datalen();
      }
      else if (uip_datalen() < sizeof(sbuf))
        /* more available than can fit but not enough to fill the whole
         * buffer */
      {
        size_t remaining = sizeof(sbuf) - uip_datalen();
        memmove(sbuf, sbuf+remaining, sbuf_len-remaining);
        memcpy(sbuf+(sbuf_len-remaining), uip_appdata, uip_datalen());
        sbuf_len = sizeof(sbuf);
      }

      /* check for newline in sbuf and parse */
      if (memchr(sbuf, '\n', sbuf_len))
      {
        static char *next, parity, rtscts=0;
        static unsigned int  baudrate, databits, stopbits;

        baudrate = atoi(sbuf, &next);
        databits = atoi(next, &next);
        while (isblank(*next)) next++;
        parity   = *next;
        stopbits = atoi(next+1, &next);
        while (next < (sbuf+sizeof(sbuf)) && isblank(*next)) next++;
        rtscts = (*next == 'r');

        sbuf_len = snprintf(sbuf, sizeof(sbuf), "%d %d%c%d %s\n",
          baudrate, databits, parity, stopbits, rtscts?"rtscts":"");

        uart_init(baudrate, databits, parity, stopbits, rtscts);
        configured = 1;

        do {
          uip_send(sbuf, sbuf_len);
          PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
        } while (!uip_acked());
      }
    } while (!configured);

    sbuf_len = 0;

    /* start i/o */
    while (1)
    {
      PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);

      /* new data received, write it to uart */
      if (uip_newdata())
        uart_write(uip_appdata, uip_datalen());

      /* get new data from the uart when the current sbuf has been sent */
      if (uip_poll())
      {
        sbuf_len = uart_read(sbuf, sizeof(sbuf));
        //sbuf_len = snprintf(sbuf, sizeof(sbuf), "polled %d bytes\n", sbuf_len);

        if (sbuf_len > 0)
        {
          do {
            uip_send(sbuf, sbuf_len);
            PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event && (uip_rexmit() || uip_acked()));
          } while (!uip_acked());
        }
      }
    };
  }

  PROCESS_END();
}
