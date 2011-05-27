#!/usr/bin/env python2
# -*- coding: utf8 -*-
#
# loopback serial line test for the wireless serial tunnel on ipv6 enabled
# jennic devices. Creates a serial connection to a local serial port and uses
# the ipv6 tcp serial tunnel to loop data through.

from serial   import Serial
from sys      import argv,stderr,exit
from random   import choice
from socket   import *
from time     import sleep
from sys      import stdout
import string

if len(argv) != 3:
    stderr.write("Usage: python2 %s <ipv6 address or hostname> <serial port>\n"%argv[0])
    exit(-1)

try:
    sp = Serial(argv[2], 115200, timeout=1, rtscts=1)
    af,typ,proto,name,sa = getaddrinfo(argv[1],4404,AF_UNSPEC,SOCK_STREAM)[0]
    so = socket(af,typ,proto)
    so.connect(sa)
except error, msg:
    stderr.write("ERROR: %s\n"%msg[1])
    exit(-1)

d = ""
while len(d) < 80:
    d += so.recv(1024)

# Select remote uart mode, we use a really slow baudrate to have a stable uart
# connection. And wait for answer


so.send("UART1 115200 8N1 RTSCTS\r\n")
#so.send("UART0 115200 8N1 \r\n")

d = " "
while d[-1] != '\n':
    d += so.recv(1024)

if d != " UART1 R115200 8N1 RTSCTS\n":
    print d
    exit(-1)


print("uart openend succesfully")
sleep(.5)

def test_tcp2ser(num):
    # generate random string
    chars = string.letters + string.digits
    s = ""

    for i in range(0,num):
        s += choice(chars)

    stdout.write("sending %d bytes"%num)
    so.send(s)

    d = ""
    while len(d) < len(s):
        d += sp.read(1024)

    #stdout.write(", read %d bytes: '%s'\n"%(len(d), d))
    stdout.write(", read %d bytes\n"%(len(d)))
    assert d==s, "fail"

test1(12400)
