CONTIKI_PROJECT = main
all: $(CONTIKI_PROJECT)

UIP_CONF_IPV6=true

CONTIKI = ../contiki
CONTIKI_SOURCEFILES = stun.c
include $(CONTIKI)/Makefile.include
