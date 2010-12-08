CONTIKI_PROJECT = main
all: $(CONTIKI_PROJECT)

UIP_CONF_IPV6=true

CONTIKI_SOURCEFILES = uart.c
CONTIKI = ../contiki
include $(CONTIKI)/Makefile.include
