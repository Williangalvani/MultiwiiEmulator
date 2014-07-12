__author__ = 'Will'

import serial
import os
import time
from random import randint, randrange

MSP_IDENT = 100  # out message         multitype + multiwii version + protocol version + capability variable
MSP_STATUS = 101  # out message         cycletime & errors_count & sensor present & box activation & current setting number
MSP_RAW_IMU = 102  # out message         9 DOF
MSP_SERVO = 103  # out message         8 servos
MSP_MOTOR = 104  # out message         8 motors
MSP_RC = 105  # out message         8 rc chan and more
MSP_RAW_GPS = 106  # out message         fix, numsat, lat, lon, alt, speed, ground course
MSP_COMP_GPS = 107  # out message         distance home, direction home
MSP_ATTITUDE = 108  # out message         2 angles 1 heading
MSP_ALTITUDE = 109  # out message         altitude, variometer
MSP_ANALOG = 110  # out message         vbat, powermetersum, rssi if available on RX
MSP_RC_TUNING = 111  # out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
MSP_PID = 112  # out message         P I D coeff (9 are used currently)
MSP_BOX = 113  # out message         BOX setup (number is dependant of your setup)
MSP_MISC = 114  # out message         powermeter trig
MSP_MOTOR_PINS = 115  # out message         which pins are in use for motors & servos, for GUI
MSP_BOXNAMES = 116  # out message         the aux switch names
MSP_PIDNAMES = 117  # out message         the PID names
MSP_WP = 118  # out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
MSP_BOXIDS = 119  # out message         get the permanent IDs associated to BOXes
MSP_SERVO_CONF = 120  # out message         Servo settings

MSP_SET_RAW_RC = 200  # in message          8 rc chan
MSP_SET_RAW_GPS = 201  # in message          fix, numsat, lat, lon, alt, speed
MSP_SET_PID = 202  # in message          P I D coeff (9 are used currently)
MSP_SET_BOX = 203  # in message          BOX setup (number is dependant of your setup)
MSP_SET_RC_TUNING = 204  # in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
MSP_ACC_CALIBRATION = 205  # in message          no param
MSP_MAG_CALIBRATION = 206  # in message          no param
MSP_SET_MISC = 207  # in message          powermeter trig + 8 free for future use
MSP_RESET_CONF = 208  # in message          no param
MSP_SET_WP = 209  # in message          sets a given WP (WP#,lat, lon, alt, flags)
MSP_SELECT_SETTING = 210  # in message          Select Setting Number (0-2)
MSP_SET_HEAD = 211  # in message          define a new heading hold direction
MSP_SET_SERVO_CONF = 212  # in message          Servo settings
MSP_SET_MOTOR = 214  # in message          PropBalance function

MSP_BIND = 240  # in message          no param

MSP_EEPROM_WRITE = 250  # in message          no param

MSP_DEBUGMSG = 253  # out message         debug string buffer
MSP_DEBUG = 254  # out message         debug1,debug2,debug3,debug4


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


print list_serial_ports()

byte_buffer = bytearray()


def serialize8(a):
    global byte_buffer
    if isinstance(a, int):
        a = chr(a)
    byte_buffer += a;
    checksum[0] ^= ord(a);


def serialize16(a):
    serialize8((a   ) & 0xFF);
    serialize8((a >> 8) & 0xFF);


def serialize32(a):
    serialize8((a    ) & 0xFF);
    serialize8((a >> 8) & 0xFF);
    serialize8((a >> 16) & 0xFF);
    serialize8((a >> 24) & 0xFF);


checksum = [0, 0, 0, 0]


def headSerialResponse(s, type):
    global byte_buffer
    byte_buffer = bytearray()
    serialize8('$');
    serialize8('M');
    serialize8('>');
    checksum[0] = 0;  # // start calculating a new checksum
    serialize8(s);
    serialize8(type);


def headSerialReply(s, type):
    headSerialResponse(s, type);


def tailSerialReply():
    global byte_buffer
    byte_buffer += chr(checksum[0])


def send_gps(lat=0, lon=0, numsats=5, alt=0, speed=0, fix=1):
    GPS_ground_course = 0
    headSerialReply(16, MSP_RAW_GPS)
    serialize8(fix)
    serialize8(numsats)
    serialize32(lat)
    serialize32(lon)
    serialize16(alt)
    serialize16(speed)
    serialize16(GPS_ground_course)
    tailSerialReply()
    port.write(str(byte_buffer))


def send_comp_gps(distance, direction):
    headSerialReply(5, MSP_COMP_GPS);
    serialize16(distance);
    serialize16(direction);
    serialize8(1);
    tailSerialReply()
    port.write(str(byte_buffer))


def send_attitude(x=0, y=0, z=0):
    headSerialReply(6, MSP_ATTITUDE)
    serialize16(x * 10)
    serialize16(y * 10)
    serialize16(z * 10)
    tailSerialReply()
    port.write(str(byte_buffer))


def send_analog(vbat=15, power=0, rssi=0, current=1234):
    headSerialReply(7, MSP_ANALOG)
    serialize8(vbat)
    serialize16(power)
    serialize16(rssi)
    serialize16(current)
    tailSerialReply()
    port.write(str(byte_buffer))


def send_altitude(alt=123, vario=123):
    headSerialReply(6, MSP_ALTITUDE)
    serialize32(alt)
    serialize16(vario)
    tailSerialReply()
    port.write(str(byte_buffer))


def send_status(stable=0, baro=0, mag=0):
    headSerialReply(10, MSP_STATUS)
    serialize16(0)
    serialize16(0)
    serialize16(0)
    serialize32(0)
    tailSerialReply()
    port.write(str(byte_buffer))


def send_debug(debug1, debug2, debug3,debug4):
    headSerialReply(8, MSP_DEBUG)
    serialize16(debug1)
    serialize16(debug2)
    serialize16(debug3)
    serialize16(debug4)
    tailSerialReply()
    port.write(str(byte_buffer))


def send_rc(channels):
    headSerialReply(12 * 2, MSP_RC)
    for i in range(12):
        serialize16(channels[i])
    tailSerialReply()
    port.write(str(byte_buffer))


def send_pid():
    headSerialReply(10 * 3, MSP_PID)
    for i in range(10):
        serialize8(123)
        serialize8(234)
        serialize8(78)
    tailSerialReply()
    port.write(str(byte_buffer))


def send_bicopter_identifier():
    headSerialResponse(7, MSP_IDENT);
    serialize8(32);
    serialize8(4);  # codigo do bicoptero
    serialize8(0);  # not used
    serialize32(0);
    tailSerialReply();
    port.write(str(byte_buffer))

servo = 0
def send_servos(servo_esquerdo,servo_direito):
    headSerialResponse(16, MSP_SERVO);

    serialize16(1500);
    serialize16(1500);
    serialize16(1500);
    serialize16(1500);

    serialize16(servo_esquerdo);
    serialize16(servo_direito);
    serialize16(1500);
    serialize16(1500);

    tailSerialReply();
    port.write(str(byte_buffer))

def send_motor_pins():
    headSerialResponse(8, MSP_MOTOR_PINS)
    serialize8(1);
    serialize8(2);
    serialize8(0);
    serialize8(0);

    serialize8(0);
    serialize8(0);
    serialize8(0);
    serialize8(0);

    tailSerialReply();
    port.write(str(byte_buffer))

def send_motor(forca_esquerdo,forca_direito):
    headSerialResponse(16, MSP_MOTOR)
    serialize16(forca_esquerdo*100);
    serialize16(forca_direito*100);
    serialize16(1300);
    serialize16(1300);

    serialize16(1300);
    serialize16(1300);
    serialize16(1300);
    serialize16(1300);

    tailSerialReply();
    port.write(str(byte_buffer))


port = serial.Serial(list_serial_ports()[0], baudrate=460800, timeout=1)
angle = 0
distance = 0

while True:
    distance += 1
    while port.read() != "<":
        pass
    distance += 1
    send_gps(23 + distance / 100000, 24, speed=distance * 10, alt=distance, fix=1)
    while port.read() != "<":
        pass
    send_comp_gps(distance, (distance % 360) - 180)
    while port.read() != "<":
        pass
    angle += 1
    send_attitude(x=distance, y=distance % 90 - 45)
    while port.read() != "<":
        pass
    send_analog(rssi=distance)
    while port.read() != "<":
        pass
    send_altitude(-distance, vario=333)
    while port.read() != "<":
        pass
    send_status()
    while port.read() != "<":
        pass
    send_rc([1900, 1900, 1500, 1100, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000])
    while port.read() != "<":
        pass
    send_bicopter_identifier()
    while port.read() != "<":
        pass
    send_motor_pins()
    while port.read() != "<":
        pass
    send_motor(distance % 20,12)
    while port.read() != "<":
        pass
    send_servos(1234,1235)
    while port.read() != "<":
        pass
    send_debug(1,2,3,4)
    print distance
