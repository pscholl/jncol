CONTIKI_PROJECT = stun
all: $(CONTIKI_PROJECT)

UIP_CONF_IPV6=true

CONTIKI = ../contiki
include $(CONTIKI)/Makefile.include
