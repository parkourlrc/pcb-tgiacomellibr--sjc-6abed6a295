#!/usr/bin/env python3
from warnings import catch_warnings
import time
import minimalmodbus
import serial
from pytictoc import TicToc


instrument = minimalmodbus.Instrument('COM6', 1)  # port name, slave address (in decimal)
# instrument.serial.port =                      # this is the serial port name
instrument.serial.baudrate = 115200        # Baud
instrument.serial.bytesize = 8
instrument.serial.parity   = serial.PARITY_NONE
instrument.serial.stopbits = 1
instrument.serial.timeout  = 0.5

instrument.address = 1                         # this is the slave address number
instrument.mode = minimalmodbus.MODE_RTU   # rtu or ascii mode
instrument.clear_buffers_before_each_transaction = True

# temp = instrument.read_register(2, 10, 3)  # Registernumber, number of decimals
# print(temp)
t = TicToc()
Tmin = 1000
Tmax = 0
fails = 0

for i in range(1000):
    try:
        t.tic() #Start timer
        uptime = instrument.read_long(1001, 4, byteorder=0)  # Registernumber, number of decimals
        mode = instrument.read_register(1003, 0, 4)  # Registernumber, number of decimals
        error = instrument.read_float(1004, 4, 2)  # Registernumber, number of decimals
        reference = instrument.read_float(1006, 4, 2)  # Registernumber, number of decimals
        print(f'Uptime: {uptime} ms - Mode: {mode:.3f} - Mode: {error:.3f} - Mode: {reference:.3f}')        
        temp4 = instrument.read_register(1001, 10, 4)  # Registernumber, number of decimals        
        temp5 = instrument.read_register(1001, 10, 4)  # Registernumber, number of decimals
        temp6 = instrument.read_register(1001, 10, 4)  # Registernumber, number of decimals
        tocval = 1000*t.tocvalue()
        Tmin = min(Tmin, tocval)
        Tmax = max(Tmax, tocval)
        print(chr(27) + "[2J")
        print(f'Elap (ms): {tocval:.3f} ({Tmin:.3f} - {Tmax:.3f}) - Fails: {fails}')

    except IOError:
        print("Failed to read from instrument")
        fails = fails+1

print(Tmin)
print(Tmax)
print(fails)
## Change temperature setpoint (SP) ##
# NEW_TEMPERATURE = 95
# instrument.write_register(24, NEW_TEMPERATURE, 1)  # Registernumber, value, number of decimals for storage