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
#include "dev/uart0.h"
#include "dev/uart1.h"
#include "lib/ringbuf.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define GREETING "remote uart port, please select baudrate and uart to use like this:\n"\
                 "(uart is always operating at 8N1 with no flow control)\n"\
                 "UART0 115200\n"

PROCESS(stun_process, "uart process");

#ifdef STUN_CONF_BUFFER_SIZE
#define BUFSIZE STUN_CONF_BUFFER_SIZE
#else
#define BUFSIZE 1024
#endif

#if (BUFSIZE & (BUFSIZE - 1)) != 0
#error STUN_CONF_BUFFER_SIZE must be a power of two (i.e., 1, 2, 4, 8, 16, 32, 64, ...).
#endif

static struct  ringbuf uartbuf;
static uint8_t uartbuf_data[BUFSIZE];

static unsigned int
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


static uart_cb_handler(unsigned char c)
{
  ringbuf_put(&uartbuf, c);
  process_poll(&stun_process);
}

static void (*uart_writeb)(unsigned char c) = NULL;

static int8_t
uart_parse_and_init(char *s, size_t *n)
  /* parse the modeline:
   *
   * (UART) (BAUDRATE)
   * - UART, either UART0 or UART1, selects the serial port to use for that
   *         connection.
   * - BAUDRATE, baudrate of the serial port, one of 9600, 19200, 38400, 57600,
   *             115200, 230400, 460800, 921600, 1843200
   */

{
  char    *next, *buf=s;
  uint32_t baudrate, port;

  /* parse (UART) */
  if (strncasecmp("UART", s, 4) == 0)
    port = atoi(s+4, &next);
  else if (!isdigit(*s))
  {
    strcpy(buf, "ERR: no uart port given\n");
    return -1;
  }
  else
    port = atoi(s, &next);

  printf(s);
  s = next;

  if (port != 1 && port != 0)
  {
    strcpy(buf, "ERR: wrong uart port\n");
    return -1;
  }

  while (isspace(*s))
    s++; /* ignore whitespace */

  printf(s);

  /* parse (BAUDRATE) */
  baudrate = atoi(s, &next);
  if (s==next) return false;
  else         s=next;

  while (isspace(*s))
    s++; /* ignore whitespace */

  /* initialize uart port */
  ringbuf_init(&uartbuf, uartbuf_data, sizeof(uartbuf_data));

  if (port == 0)
  {
    uart0_init(baudrate);
    uart0_set_input(uart_cb_handler);
    uart_writeb = uart0_writeb;
  }
  else
  {
    uart1_init(baudrate);
    uart1_set_input(uart_cb_handler);
    uart_writeb = uart1_writeb;
  }

  return 0;
}

PROCESS_THREAD(stun_process, ev, data)
{
  /* make sure the modeline fits! */
  static bool configured = false;
  static char buf[sizeof(uartbuf_data)+200];
  static volatile uint16_t buf_len = 0;

  /* whenever these events happen, the process is to be restarted */
  if (ev == tcpip_event && (uip_aborted() || uip_timedout() || uip_closed()))
  {
    process_exit(&stun_process);
    process_start(&stun_process, NULL);
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

        configured = 0==uart_parse_and_init(buf, &m);

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
      PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event ||
                               ev == PROCESS_EVENT_POLL);

      /* XXX: possible problem here: a rexmit might get executed with packetbuf
       * of different size, is this a problem? */
      if (ev == PROCESS_EVENT_POLL)
      {
        int c;

        while ((c=ringbuf_get(&uartbuf)) != -1 && buf_len < sizeof(buf))
          buf[buf_len++] = (uint8_t) c;

        /* uip_len == 0 is there to avoid overwriting any data in uip_buf that a
         * process is waiting to get delivered. */
        if (uip_len==0)
          uip_send(buf, buf_len);
      }
      else if (ev == tcpip_event)
      {
        /* new data received, write it to uart */
        if (uip_newdata())
        {
          size_t i;

          for (i=0; i<uip_datalen(); i++)
            uart_writeb(((char*) uip_appdata)[i]);

          uip_len = 0;
        }

        if (uip_rexmit() || uip_poll())
          uip_send(buf, buf_len);

        if (uip_acked())
          buf_len = 0;
      }
    }
  }

  PROCESS_END();
}
