#!/usr/bin/python

import time
import serial
import sys
import termios
#Do check the Python version as some constants have moved.
if (sys.hexversion < 0x020100f0):
	import TERMIOS
else:
	TERMIOS = termios

if (sys.hexversion < 0x020200f0):
	import FCNTL
else:
	import fcntl
	FCNTL = fcntl

port_names = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3',]

ports = {}

print (' '*5) + 'Open'
for port in port_names:
	ports[port] = serial.Serial(
		baudrate=115200,
		port=port,
		bytesize=serial.EIGHTBITS,
		parity=serial.PARITY_NONE,
		stopbits=serial.STOPBITS_ONE,
		xonxoff = False,
		rtscts = False,
		timeout=None,           # read timeout:wait forever if None
		writeTimeout=1.0,       # write timeout:will rise a
		                        # SerialTimeoutException if the data not
		                        # sent in seconds
		)

count = 1
print '/dev/ttyACM0      /dev/ttyACM1      /dev/ttyACM2      /dev/ttyACM3'
try:
	while True:
		print_buffer = ''
		for port in port_names:
			print_buffer +=  '{0:14}    '.format(ports[port].read(16).rstrip())
		if (count % 10) is 0:
			print_buffer += '\n\n/dev/ttyACM0      /dev/ttyACM1      /dev/ttyACM2      /dev/ttyACM3'
			count = 0
		print print_buffer
		count += 1
finally:
	print (' '*5) + 'finally: Close'
	for port in ports:
		if ports[port].isOpen():
			ports[port].close()
