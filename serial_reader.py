__author__ = 'Will'

import serial
import os

from serial.tools import list_ports

def list_serial_ports():
    # Windows
    if os.name == 'nt':
        # Scan for available ports.
        available = []
        for i in range(256):
            try:
                s = serial.Serial(i)
                available.append('COM' + str(i + 1))
                s.close()
            except serial.SerialException:
                pass
        return available
    else:
        # Mac / Linux
        return [port[0] for port in list_ports.comports()]


#print list_serial_ports()


port = serial.Serial('com18',baudrate=115200)



while True:
    while 1:
        print ord(port.read(1))