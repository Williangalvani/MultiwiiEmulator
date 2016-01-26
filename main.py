__author__ = 'Will'

import serial
from threading import Thread
import os
import time
import struct
from random import randint, randrange
from dataread.provant_serial import ProvantSerial

global SERIALPORT
global SERIALPORT2
if os.name == "posix":
    SERIALPORT = "/dev/ttyVirtual1"
    SERIALPORT2 = "/dev/ttyVirtual2"
else:
    SERIALPORT = "COM5"

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
MSP_MOTOR_PINS = 115  # out message         which pins are in use for motors & vos, for GUI
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




#############################################################
#
MSP_ESCDATA                 =99
MSP_CONTROLDATAOUT          =98
MSP_CONTROLDATAIN           =97
#
#############################################################


print "main"

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
#port = serial.Serial(SERIALPORT, baudrate=460800, timeout=1)
port2 = serial.Serial(SERIALPORT2, baudrate=460800, timeout=1)


com = ProvantSerial(window = None, serial_name=SERIALPORT, baudrate_value=460800, debug_mode=False)



def waitForRequest():
    time.sleep(0.001)

global until100
global lastSerialAvailable
global angle
global distance
angle = 0
distance = 0
until100=0
lastSerialAvailable=0
#Testando possiveis Threads para a Multiwii
def checkserial(a,b):
	while True:
		com.update()

def principal(c,d):
	global until100
	global lastSerialAvailable
	global angle
	global distance
	kStart = False
	kStop = True
	kReset = False 
	while True:
		if com.control == "i":
				if kStart == True:
					print "Start"
					kReset = True
					kStop = True
					kStart = False
				if until100>99:
					until100=0			
				until100 += 1
				distance += 1
				waitForRequest()
				distance += 1
				com.send_gps(23 + distance / 100000, 24, speed=distance * 10, alt=distance, fix=1)
				waitForRequest()
				com.send_comp_gps(distance, (distance % 360) - 180)
				waitForRequest()
				angle += 1
				com.send_attitude(angle,angle,angle)
				if(angle>180):
					angle=-180
				waitForRequest()
				com.send_analog(rssi=distance)
				waitForRequest()
				com.send_altitude(666, 333)
				waitForRequest()
				com.send_status()
				waitForRequest()
				com.send_rc([600,700,1000,1700,1500,800,300,400,600,700,100,400,300])
				waitForRequest()
				com.send_rc_normalize([100-2*until100, 2*until100-100, 100-2*until100, 2*until100-100, until100, 100-until100, until100, 65, 33, 100, 89, 70])         #provant msg
				waitForRequest()
				com.send_bicopter_identifier()
				waitForRequest()
				com.send_motor_pins()
				waitForRequest()
				com.send_motor(12,12)
				waitForRequest()
				com.send_servos(5,-5)
				waitForRequest()
				com.send_debug(1,2,3,4)
				waitForRequest()
				com.send_raw_imu([1,2,3],[4,5,6],[7,8,9])
				waitForRequest()
				com.send_carga(20.20,21.21,22.22,23.23,24.24,25.25)
				com.sendControldatain(rpy=[1.1,2.2,3.3],drpy=[4.4,5.5,6.6],position=[7.7,8.8,9.9],velocity=[10.10,11.11,12.12], servo = [13.13,14.14],dservo = [15.15,16.16])
				com.sendControldataref(r_rpy=[1.1,2.2,3.3],r_drpy=[4.4,5.5,6.6],r_position=[7.7,8.8,9.9],r_velocity=[10.10,11.11,12.12], r_servo = [13.13,14.14],r_dservo = [15.15,16.16])
				waitForRequest()
				com.sendControldataout(servo=[128,137],esc=[1.1,2.2,3.3,4.4])
				waitForRequest()
				com.sendEscdata(rpm=[123,321],current=[1.1,2.2],voltage=[3.3,4.4])
				waitForRequest()
				print distance
				while(port2.inWaiting()>1024):
					waitForRequest()
		elif com.control == "r":
				if kReset == True:
					print "Reset"
					kStart = True
					kStop = True
					kReset = False
					angle = 0
					distance = 0
					until100=0
					lastSerialAvailable=0
					waitForRequest()
					com.send_gps(0 , 24, speed=distance * 10, alt=distance, fix=1)
					waitForRequest()
					com.send_comp_gps(distance, (distance % 360) - 180)
					waitForRequest()
					com.send_attitude(angle,angle,angle)
					if(angle>180):
						angle=-180
					waitForRequest()
					com.send_analog(rssi=distance)
					waitForRequest()
					com.send_altitude(666, 333)
					waitForRequest()
					com.send_status()
					waitForRequest()
					com.send_rc([600,700,1000,1700,1500,800,300,400,600,700,100,400,300])
					waitForRequest()
					com.send_rc_normalize([100-2*until100, 2*until100-100, 100-2*until100, 2*until100-100, until100, 100-until100, until100, 65, 33, 100, 89, 70])         #provant msg
					waitForRequest()
					com.send_bicopter_identifier()
					waitForRequest()
					com.send_motor_pins()
					waitForRequest()
					com.send_motor(12,12)
					waitForRequest()
					com.send_servos(5,-5)
					waitForRequest()
					com.send_debug(1,2,3,4)
					waitForRequest()
					com.send_raw_imu([1,2,3],[4,5,6],[7,8,9])
					waitForRequest()
					com.sendControldatain(rpy=[1.1,2.2,3.3],drpy=[4.4,5.5,6.6],position=[7.7,8.8,9.9],velocity=[10.10,11.11,12.12], servo = [13.13,14.14],dservo = [15.15,16.16])
					com.sendControldataref(r_rpy=[1.1,2.2,3.3],r_drpy=[4.4,5.5,6.6],r_position=[7.7,8.8,9.9],r_velocity=[10.10,11.11,12.12], r_servo = [13.13,14.14],r_dservo = [15.15,16.16])
					waitForRequest()
					com.sendControldataout(servo=[128,137],esc=[1.1,2.2,3.3,4.4])
					waitForRequest()
					com.sendEscdata(rpm=[123,321],current=[1.1,2.2],voltage=[3.3,4.4])
					waitForRequest()
					print distance
					while(port2.inWaiting()>1024):
						waitForRequest()
		elif com.control == "s":
				if kStop == True:
					print "Stop"
					kStart = True
					kReset = True
					kStop = False


th1 = Thread(target=checkserial, args = ('',''))
th2 = Thread(target=principal, args = ('',''))
th1.start()
th2.start()
