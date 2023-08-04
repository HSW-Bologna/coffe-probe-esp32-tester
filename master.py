#!/usr/bin/env python3
import minimalmodbus

# port name, slave address (in decimal)
instrument = minimalmodbus.Instrument('/dev/ttyUSB0', 1, debug=True)
# https://minimalmodbus.readthedocs.io/en/stable/usage.html#default-values
instrument.serial.baudrate = 115200
instrument.serial.timeout = 0.05
# instrument.close_port_after_each_call = True

instrument.serial.reset_input_buffer()
instrument.serial.readline()

try:
    # register 0, value 1
    print(instrument.write_bit(0, 1))
except IOError as err:
    print("Failed to write to instrument")
    print(err)

try:
    # register 1 
    print(instrument.read_register(1))
except IOError as err:
    print("Failed to read from instrument")
    print(err)
