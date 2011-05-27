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
#include "leds.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <AppHardwareApi.h>

#define GREETING "remote uart port, please select the operation mode like this:\n"\
                 "UART0 115200 8N1\n"

PROCESS_NAME(uart_process);
PROCESS(uart_process, "Uart Process");
AUTOSTART_PROCESSES(&uart_process);

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

static int8_t
uart_parse_and_init(char *s, size_t *n, bool *framing_mode, uint32_t *port)
  /* parse the modeline:
   *
   * (UART) [FRAMING](BAUDRATE) (DATABITS)(PARITY)(STOPBITS) [FLOWCONTROL]
   * where anything in [] is optional, and anything in () is mandatory:
   * - UART, either UART0 or UART1, selects the serial port to use for that
   *         connection.
   * - FRAMING, either R for raw mode or F for framing serial protocol, defaults to
   *            raw mode.
   * - BAUDRATE, baudrate of the serial port, one of 9600, 19200, 38400, 57600,
   *             115200, 230400, 460800, 921600, 1843200
   * - DATABITS, either 7 or 8
   * - PARITY, N for no-parity, E for even parity, O for odd parity
   * - STOPBITS, 1 for 1 stop bit, 2 for 2 stop bits (or 1.5 bits if 7 databits are
   *             selected)
   * - FLOWCONTROL, RTSCTS if rts/cts flow control is to be enabled. DTR/DTS and
   *                XON/XOFF flowcontrol is not supported.
   */

{
  char    *next, *buf=s;
  uint32_t baudrate;
   uint8_t databits, parity, stopbits, flowcontrol;

  /* parse (UART) */
  if (strncasecmp("UART", s, 4) == 0)
    *port = atoi(s+4, &next);
  else if (!isdigit(*s))
  {
    strcpy(buf, "ERR: no uart port given\n");
    return -1;
  }
  else
    *port = atoi(s, &next);

  printf(s);
  s = next;

  if (*port != 1 && *port != 0)
  {
    strcpy(buf, "ERR: wrong uart port\n");
    return -1;
  }

  while (isspace(*s))
    s++; /* ignore whitespace */

  printf(s);

  /* parse [FRAMING] */
  if (*s == 'F' || *s == 'f')
  {
    *framing_mode = true;
    s++;
  }
  else if (*s == 'R' || *s == 'r')
  {
    *framing_mode = false;
    s++;
  }
  else if (!isdigit(*s))
  {
    strcpy(buf, "ERR: framing or baudrate expected\n");
    return -2;
  }

  /* parse (BAUDRATE) */
  baudrate = atoi(s, &next);
  if (s==next) return false;
  else         s=next;

  while (isspace(*s))
    s++; /* ignore whitespace */

  /* parse (DATABITS) */
  switch(*s)
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
      databits = E_AHI_UART_WORD_LEN_8;
      break;
    default:
      strcpy(buf, "ERR: databits must be in range of 5-8\n");
      return -3;
  }
  s++;

  /* parse (PARITY) */
  switch(*s) {
  case 'E':
  case 'e':
    parity = E_AHI_UART_EVEN_PARITY;
    break;
  case 'O':
  case 'o':
    parity = E_AHI_UART_ODD_PARITY;
    break;
  case 'N':
  case 'n':
    parity = E_AHI_UART_NO_PARITY;
    break;
  default:
    strcpy(buf, "ERR: parity must be either E,O or N\n");
    return -4;
  }
  s++;

  /* parse (STOPBITS) */
  switch(*s) {
  case '2':
    stopbits = E_AHI_UART_2_STOP_BITS;
    break;
  case '1':
    stopbits = E_AHI_UART_1_STOP_BIT;
    break;
  default:
    strcpy(buf, "ERR: stopbits must be 2 or 1\n");
    return -5;
  }
  s++;

  /* parse [FLOWCONTROL] */
  while (isspace(*s))
    s++; /* ignore whitespace */

  if (strncasecmp("RTSCTS", s, 6) == 0)
    flowcontrol = E_AHI_UART_RTSCTS_FLOWCTRL;
  else
    flowcontrol = E_AHI_UART_NO_FLOWCTRL;

  /* dest output, copy back to string */
  {
    int c = 'N';

    switch (parity) {
      case E_AHI_UART_NO_PARITY:
        c = 'N';
        break;
      case E_AHI_UART_EVEN_PARITY:
        c = 'E';
        break;
      case E_AHI_UART_ODD_PARITY:
        c = 'O';
        break;
      default:
        c = 'W';
        break;
    }

    *n = snprintf(buf, *n, "UART%d %c%d %d%c%d %s",
                       *port, *framing_mode? 'F':'R', baudrate,
                       databits+5, c, stopbits,
                       flowcontrol==E_AHI_UART_RTSCTS_FLOWCTRL?"RTSCTS":"abc\n");
  }

  uart_init(*port, baudrate, databits, parity, stopbits, flowcontrol);
  return 0;
}

PROCESS_THREAD(uart_process, ev, data)
{
  /* make sure the modeline fits! */
  static char buf[800];
  static uint16_t buf_len = 0;
  static bool configured = false, framing_mode = false;
  static uint32_t port = E_AHI_UART_0;

  /* whenever these events happen, the process is to be restarted */
  if (ev == tcpip_event && (uip_aborted() || uip_timedout() || uip_closed()))
  {
    process_exit(&uart_process);
    process_start(&uart_process, NULL);
    buf_len = configured = 0;
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

    /* wait for the configuration line and store it in the buf. */
    do {
      PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event && uip_newdata());

      /* either copy in the whole uip_appdata, move already received bytes to make
       * space or just add to the end */
      if (uip_datalen() > sizeof(buf))
      {
        /* fill whole buffer with the end of the rxd buffer */
        memcpy(buf, uip_appdata+(uip_datalen()%sizeof(buf)), sizeof(buf));
        buf_len = sizeof(buf);
      }
      else if (uip_datalen()+buf_len <= sizeof(buf))
      {
        /* fill up the buffer with available data */
        memcpy(buf+buf_len, uip_appdata, uip_datalen());
        buf_len += uip_datalen();
      }
      else if (uip_datalen() < sizeof(buf))
        /* more available than can fit but not enough to fill the whole
         * buffer */
      {
        size_t remaining = sizeof(buf) - uip_datalen();
        memmove(buf, buf+remaining, buf_len-remaining);
        memcpy(buf+(buf_len-remaining), uip_appdata, uip_datalen());
        buf_len = sizeof(buf);
      }

      /* check for newline in buf and parse */
      if (memchr(buf, '\n', buf_len))
      {
        static size_t m;
        m = buf_len;
        configured = 0==uart_parse_and_init(buf, &m, &framing_mode, &port);

        do { /* send the init message */
          m = strlen(buf);
          uip_send(buf, m);
          PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
        } while (!uip_acked());

        /* clear the buffer */
        memset(buf, 0x00, sizeof(buf));
        buf_len = 0;
      }
    } while (!configured);

    buf_len = 0;

    /* start i/o */
    while (1)
    {
      /* on a event-based uart implementation, we would have a uart_event
       * and check for new data there instead of using the uip poll event. */
      PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);

      /* new data received, write it to uart */
      if (uip_newdata())
      {
        uart_write(port, uip_appdata, uip_datalen());
        printf("%d bytes\n", uip_datalen());
      }

      if (uip_rexmit())
        uip_send(buf, buf_len);

      if (uip_acked())
        buf_len = 0;

      if ((uip_acked() || uip_poll()) && buf_len==0)
      {
        buf_len = uart_read(port, buf, sizeof(buf));
        printf("%d bytes polled\n", buf_len);
        uip_send(buf, buf_len);
      }
    };
  }

  PROCESS_END();
}
