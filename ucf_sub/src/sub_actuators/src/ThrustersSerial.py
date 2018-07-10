#run "pip install pyserial"

import serial
from sub_trajectory import ThrusterCmd

ser = serial.Serial(
    port='/dev/ucfsub/motors',
    baudrate=1000000,
    bytesize=serial.EIGHTBITS
)

if ser.isOpen()

def start():
	rospy.Subscriber('/thrusters/cmd_vel', ThrusterCmd, sendThrusterSerial)

def sendThrusterSerial(data):
	ouput = bytearray()
	sum = 0
	for thruster in data.cmd:
		speed = int(thruster%65536)
		output.append(chr(speed%256))
		output.append(chr(speed >> 8))
		sum += speed
	output.append(chr(sum%256))
	ser.write(encode(output))

'''
Copyright (c) 2010 Craig McQueen

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
'''
def encode(in_bytes):
    if isinstance(in_bytes, str):
        raise TypeError('Unicode-objects must be encoded as bytes first')
    in_bytes_mv = _get_buffer_view(in_bytes)
    final_zero = True
    out_bytes = bytearray()
    idx = 0
    search_start_idx = 0
    for in_char in in_bytes_mv:
        if in_char == b'\x00':
            final_zero = True
            out_bytes.append(idx - search_start_idx + 1)
            out_bytes += in_bytes_mv[search_start_idx:idx]
            search_start_idx = idx + 1
        else:
            if idx - search_start_idx == 0xFD:
                final_zero = False
                out_bytes.append(0xFF)
                out_bytes += in_bytes_mv[search_start_idx:idx+1]
                search_start_idx = idx + 1
        idx += 1
    if idx != search_start_idx or final_zero:
        out_bytes.append(idx - search_start_idx + 1)
        out_bytes += in_bytes_mv[search_start_idx:idx]
return bytes(out_bytes)
