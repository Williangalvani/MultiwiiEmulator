__author__ = 'Will'

import serial
from threading import Thread
import os
import time
import struct
from random import randint, randrange

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
port = serial.Serial(SERIALPORT, baudrate=460800, timeout=1)
port2 = serial.Serial(SERIALPORT2, baudrate=460800, timeout=1)

######################################## Encoding ################################################
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

def serializeFloat(a):
    b=struct.pack('<f', a)
    for x in xrange(0,4):
        serialize8(b[x])


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

def sendControldatain(rpy=[0,0,0],drpy=[0,0,0],position=[0,0,0],velocity=[0,0,0]):
    headSerialReply(48, MSP_CONTROLDATAIN)
    for x in xrange(0,3):
        serializeFloat(rpy[x])
    for x in xrange(0,3):
        serializeFloat(drpy[x])
    for x in xrange(0,3):
        serializeFloat(position[x])
    for x in xrange(0,3):
        serializeFloat(velocity[x])
    tailSerialReply()
    port.write(str(byte_buffer))

def sendControldataout(servo=[0,0],esc=[0,0,0,0]):
    headSerialReply(24, MSP_CONTROLDATAOUT)
    serializeFloat(servo[0])
    serializeFloat(esc[0])
    serializeFloat(esc[1])
    serializeFloat(servo[1])
    serializeFloat(esc[2])
    serializeFloat(esc[3])
    tailSerialReply()
    port.write(str(byte_buffer))

def sendEscdata(rpm=[0,0],current=[0,0],voltage=[0,0]):
    headSerialReply(20, MSP_ESCDATA)
    for x in xrange(0,2):
        serialize16(rpm[x])
        serializeFloat(current[x])
        serializeFloat(voltage[x])
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

def send_raw_imu(gyro, acc, mag):
    headSerialResponse(18, MSP_RAW_IMU)
    for i in range(3):
        serialize16(gyro[i])
    for i in range(3):
        serialize16(acc[i])
    for i in range(3):
        serialize16(mag[i])
    tailSerialReply()
    port.write(str(byte_buffer))


def send_motor_pins():
    headSerialResponse(8, MSP_MOTOR_PINS)
    serialize8(1)
    serialize8(2)
    serialize8(0)
    serialize8(0)

    serialize8(0)
    serialize8(0)
    serialize8(0)
    serialize8(0)

    tailSerialReply();
    port.write(str(byte_buffer))

def send_motor(forca_esquerdo,forca_direito):
    headSerialResponse(16, MSP_MOTOR)
    serialize16(forca_esquerdo);
    serialize16(forca_direito);
    serialize16(1300);
    serialize16(1300);

    serialize16(1300);
    serialize16(1300);
    serialize16(1300);
    serialize16(1300);

    tailSerialReply();
    port.write(str(byte_buffer))

#provant_msg
def send_rc_normalize(channels):
    MSP_RCNORMALIZE             =96
    headSerialReply(12 * 2, MSP_RCNORMALIZE)
    for i in range(12):
        serialize16(channels[i])
    tailSerialReply()
    port.write(str(byte_buffer))
#########################################################################################

###################################################### Decoding ######################################################################
def decodeFloat(data):
	return struct.unpack('<f', ''.join(data))[0]

def decode32(data):
	#print data
	result = (ord(data[0]) & 0xff) + ((ord(data[1]) & 0xff) << 8) + ((ord(data[2]) & 0xff) << 16) + ((ord(data[3]) & 0xff) << 24)
	is_negative = ord(data[3]) >= 128
	if is_negative:
		result -= 2**32
	return result

def decode16(data):
	#print data
	result = (ord(data[0]) & 0xff) + ((ord(data[1]) & 0xff) << 8)
	is_negative = ord(data[1]) >= 128
	if is_negative:
		result -= 2**16
	return result

def decode216(result):
	#print data
	is_negative = ((result>>8)&0xff) >= 128
	if is_negative:
		result -= 2**16
	return result

def checksum_matches():
	check = who ^ size
	for x in xrange(0, size):
		check ^= ord(L[x])
	if((check == ord(L[size]))):
		sampleCount+=1
	return (check == ord(L[size]))

def update():
	while ser.inWaiting() > 10:
		takeHead()

def takeHead():
	if (ord(port.read()) == MSP_HEAD[0]):  # checkhead1
		if (ord(portread()) == MSP_HEAD[1]):  #checkhead2
			if (ord(port.read()) == MSP_HEAD[2]):  #checkhead3
				solve_type()

def solve_type():
	size = ord(port.read())  # pega tamanho
	who = ord(port.read())  # descobre quem e
	word = port.read(size + 1)  # pega os dados + checksum
	L = list(word)  # passa para uma lista
	takeData()

def readSampleCount():
	return self.sampleCount

###############  PROPER MESSAGE DECODING IS HERE ##################################

def takeData():
	if (who == MSP_ATTITUDE):
		if checksum_matches():
			attitude.roll = decode16(self.L[0:2])/10
			attitude.pitch = decode16(self.L[2:4])/10
			attitude.yaw = decode16(self.L[4:6])

		if window:
			window.addArray('Attitude.Filtered',
			(attitude.roll, attitude.pitch, attitude.yaw),
			('Roll','Pitch','Yaw'))



	if (who == MSP_RAW_GPS):
		if checksum_matches():
			raw_gps.fix = ord(L[0])
			raw_gps.numsats = ord(L[1])
			raw_gps.lat = decode32(L[2:6])
			raw_gps.lon = decode32(L[6:10])
			raw_gps.alt = decode16(L[10:12])
			raw_gps.speed = decode16(L[12:14])
			raw_gps.ggc = decode16(L[14:16])

	if window:
		window.addArray('GPS',
			         (raw_gps.fix,raw_gps.numsats,raw_gps.lat,raw_gps.lon,raw_gps.alt,raw_gps.speed,raw_gps.ggc),
			         ('Fix','Numsats','Lat','Long','Alt','Veloc','GGC'))

	if (who == MSP_COMP_GPS):
		if checksum_matches():
			comp_gps.distance = decode16(L[0:2])
			comp_gps.direction = decode16(L[2:4])
			comp_gps.update = ord(L[4])

		if window:
			window.addArray('Comp_gps',
					 (comp_gps.distance,comp_gps.direction,comp_gps.update),
					 ('Dist','Direction','Update'))                

	if (who == MSP_ANALOG):
		if checksum_matches():
			analog.vbat = ord(L[0])
			analog.power = decode16(L[1:3])
			analog.rssi = decode16(L[3:5])
			analog.current = decode16(L[5:7])

		if window:
			window.addArray('Analog',
				         (analog.vbat, analog.power,analog.rssi,analog.current),
				         ('Vbat','Power','Rssi','Current'))

	if (who == MSP_ALTITUDE):
		if checksum_matches():
			altitude.alt = decode32(L[0:4])
			altitude.vario = decode16(L[4:6])

		if window:
			window.addArray('Altitude',
		         (altitude.alt,altitude.vario),
		         ('Alt','Vario'))
			window.verticalSlider_3.setValue(altitude.alt)


	if (who == MSP_STATUS):
		if checksum_matches():
			status.cycleTime = decode16(L[0:2])
			status.i2cec = decode16(L[2:4])
			status.sensor = decode16(L[4:6])
			status.flag = decode32(L[6:10])
			status.gccs = ord(L[10])

		if window:
			window.addArray('Status',
			         (status.cycleTime,status.i2cec,status.sensor,status.flag,status.gccs),
			         ('CycleTime','Numi2cerror','Sensor','Flag','Gccs'))

	if (who == MSP_DEBUG):
		if checksum_matches():
			for i in xrange(0, size / 2):
				debug.debug[i] = decode16(L[i*2:i*2+2])

		if window:
			window.addArray('Debug', debug.debug)


	if (who == MSP_RC):
		if checksum_matches():
			for x in xrange(0, size / 2):
				rc.channel[x] = ord(L[x * 2]) + (ord(L[x * 2 + 1]) << 8)

		if window:
			window.addArray('RC_channel', rc.channel)

	if (who == MSP_PID):
		if checksum_matches():
			for x in xrange(0, size):
				pid.pid[x] = ord(L[x])


	if (who == MSP_IDENT):
		if checksum_matches():
			ident.version = ord(L[0])
			ident.multtype = ord(L[1])
			ident.mspversion = ord(L[2])
			ident.capability = ord(L[3]) + (ord(L[4]) << 8) + (ord(L[5]) << 16) + (
			ord(L[6]) << 24)

	if (who == MSP_SERVO):
		if checksum_matches():
			for x in xrange(0, size / 2):
				servo.servo[x] = decode16(L[x*2:x*2+2])
		if window:
			window.addArray('ServoAngle',
		         servo.servo[4:6],
		         ('LServoAngle', 'RServoAngle'))


	if (who == MSP_MOTOR_PINS):
		if checksum_matches():
			pins = [None] * (size)
		for x in xrange(0, size):
			motor_pins.pin[x] = ord(L[x])

	if (who == MSP_RAW_IMU):
		if checksum_matches():
			for i in range(3):
				imu.acc[i] = decode16(L[i*2:i*2+2])
			for i in range(3, 6):
				imu.gyr[i-3] = decode16(L[i*2:i*2+2])
			for i in range(6, 9):
				imu.mag[i-6] = decode16(L[i*2:i*2+2])

		if window:
			window.addArray('Attitude.Gyro', imu.gyr,('X','Y','Z'))
			window.addArray('Attitude.Acc', imu.acc,('X','Y','Z'))
			window.addArray('Attitude.Mag', imu.mag,('X','Y','Z'))

	if (who == MSP_MOTOR):
		if checksum_matches():
			for x in xrange(0, self.size / 2):
				motor.motor[x] = ord(L[x * 2]) + (ord(L[x * 2 + 1]) << 8)

		if window:
			window.addArray('MotorSetpoint',
					 motor.motor[0:2],
					 ('Lmotor', 'Rmotor'))
			window.lMotorSetpoint.setValue(motor.motor[0])
			window.rMotorSetpoint.setValue(motor.motor[1])



	if (who == MSP_CONTROLDATAIN):
		if checksum_matches():
			for x in xrange(0, 3):
				controldatain.rpy[x]= decodeFloat(L[x*4:4+x*4])
			for x in xrange(3, 6):
				controldatain.drpy[x-3]= decodeFloat(L[x*4:4+x*4])
			for x in xrange(6, 9):
				controldatain.position[x-6]= decodeFloat(L[x*4:4+x*4])
			for x in xrange(9, 12):
				controldatain.velocity[x-9]= decodeFloat(L[x*4:4+x*4])
		if window:
			data = controldatain
			window.addArray("Data.rpy",
					 data.rpy,)
			window.addArray("Data.drpy",
					 data.drpy,)
			window.addArray("Data.Position",
					 data.position,)
			window.addArray("Data.Velocity",
					 data.velocity,)

	if (who == MSP_CONTROLDATAOUT):
		if checksum_matches():
			controldataout.servoLeft = decodeFloat(L[0:4])
			controldataout.escLeftNewtons  = decodeFloat(L[4:8])
			controldataout.escRightNewtons = decodeFloat(L[8:12])
			controldataout.servoRight = decodeFloat(L[12:16])
			controldataout.escLeftSpeed = decodeFloat(L[16:20])
			controldataout.escRightSpeed = decodeFloat(L[20:24])
		if window:
			data = controldataout
			window.addArray("ActuatorsCommand.Left",
					 (data.escLeftNewtons, data.escLeftSpeed, data.servoLeft),
					 ('Newtons', 'Speed', 'Servo'))
			window.addArray("ActuatorsCommand.Right",
					 (data.escRightNewtons, data.escRightSpeed, data.servoRight),
					 ('Newtons', 'Speed', 'Servo'))

	if (who == MSP_ESCDATA):
		if checksum_matches():
			for x in xrange(0, 2):
				escdata.rpm[x] = decode16(L[x*10:x*10+2])
				escdata.current[x] = decodeFloat(L[x*10+2:x*10+6])
				escdata.voltage[x] = decodeFloat(L[x*10+6:x*10+10])
		if window:
			for i, escName in enumerate(['EscFeedbackLeft', 'EscFeedbackRight']):
				window.addArray(escName,
						     (escdata.rpm[i], escdata.current[i], escdata.voltage[i]),
						     ('Rpm', 'Current', 'Voltage'))
				window.lMotorRpm.setValue(escdata.rpm[0])
				window.rMotorRpm.setValue(escdata.rpm[1])

	if (who == MSP_RCNORMALIZE):
		if checksum_matches():
			for x in xrange(0, size / 2):
				rcn.channel[x] = decode16(L[x*2:x*2+2])
	#test
		if window:
			window.addArray('RCN', rcn.channel[0:7])
			window.left_joystick.move(rcn.channel[3], rcn.channel[2])
			window.right_joystick.move(rcn.channel[0],rcn.channel[1])

	#                 self.window.verticalSlider_2.setValue(self.rcn.channel[0])
		window.radioButton.setAutoExclusive(False);
		window.radioButton_2.setAutoExclusive(False);
		if rcn.channel[5]>50:
			window.radioButton.setChecked(1)
		else:
			window.radioButton.setChecked(0)
		if rcn.channel[6]>50:
			window.radioButton_2.setChecked(1)
		else:
			window.radioButton_2.setChecked(0)
			window.dial.setValue(rcn.channel[4])
####################################################################################

print "connected to port " , port

def waitForRequest():
    time.sleep(0.001)

global control
global until100
global lastSerialAvailable
global angle
global distance
angle = 0
distance = 0
until100=0
lastSerialAvailable=0
control = "s"
#Testando possiveis Threads para a Multiwii
def checkserial(a,b):
	global control
	L = []
	while True:
		take = port.read()
		if (take == ""):
			pass
		elif ( (take == "i") or (take == "s") or (take == "r") ):
			control = take

def principal(c,d):
	global control
	global until100
	global lastSerialAvailable
	global angle
	global distance
	kStart = True
	kStop = True
	KReset = True
	while True:
		if (control == "i"):
			if (kStart == True):
				print "Start"
			else:
				pass
			kStart = False
			kStop = True
			kReset = True
			if until100>99:
				until100=0			
			until100 += 1
			distance += 1
			waitForRequest()
			distance += 1
			send_gps(23 + distance / 100000, 24, speed=distance * 10, alt=distance, fix=1)
			waitForRequest()
			send_comp_gps(distance, (distance % 360) - 180)
			waitForRequest()
			angle += 1
			send_attitude(angle,angle,angle)
			if(angle>180):
				angle=-180
			waitForRequest()
			send_analog(rssi=distance)
			waitForRequest()
			send_altitude(666, 333)
			waitForRequest()
			send_status()
			waitForRequest()
			send_rc([600,700,1000,1700,1500,800,300,400,600,700,100,400,300])
			waitForRequest()
			send_rc_normalize([100-2*until100, 2*until100-100, 100-2*until100, 2*until100-100, until100, 100-until100, until100, 65, 33, 100, 89, 70])         #provant msg
			waitForRequest()
			send_bicopter_identifier()
			waitForRequest()
			send_motor_pins()
			waitForRequest()
			send_motor(12,12)
			waitForRequest()
			send_servos(5,-5)
			waitForRequest()
			send_debug(1,2,3,4)
			waitForRequest()
			send_raw_imu([1,2,3],[4,5,6],[7,8,9])
			waitForRequest()
			sendControldatain(rpy=[1.1,2.2,3.3],drpy=[4.4,5.5,6.6],position=[7.7,8.8,9.9],velocity=[10.10,11.11,12.12])
			waitForRequest()
			sendControldataout(servo=[128,137],esc=[1.1,2.2,3.3,4.4])
			waitForRequest()
			sendEscdata(rpm=[123,321],current=[1.1,2.2],voltage=[3.3,4.4])
			waitForRequest()
			print distance
			while(port2.inWaiting()>1024):
				waitForRequest()
		elif (control == "r"):
			if (kReset == True):
				print "Reset"
			else:
				pass
			kStart = True
			kStop = True
			kReset = False
			until100 = 0
			distance = 0
			angle = 0
			send_gps()
			waitForRequest()
		    	sendControldatain()
			waitForRequest()
			sendControldataout()
			waitForRequest()
			sendEscdata()
			waitForRequest()		
			send_comp_gps(0,0)
			waitForRequest()	
			send_attitude()
			waitForRequest()
			send_analog()
			waitForRequest()
			send_altitude()
			waitForRequest()
			send_status()
			waitForRequest()
			send_rc([600,700,1000,1700,1500,800,300,400,600,700,100,400,300])
			waitForRequest()
			send_rc_normalize([600,700,1000,1700,1500,800,300,400,600,700,100,400,300])
			waitForRequest()
			send_bicopter_identifier()
			waitForRequest()
			send_motor_pins()
			waitForRequest()
			send_motor(12,12)
			waitForRequest()
			send_servos(5,-5)
			waitForRequest()	
			send_debug(1,2,3,4)
			waitForRequest()	
			send_raw_imu([1,2,3],[4,5,6],[7,8,9])
			waitForRequest()
			control = 'i'
		elif (control == "s"):
			if (kStop == True):
				print "Stop"
			else:
				pass
			kStart = True
			kStop = False
			kReset = True

		else:
			pass
		waitForRequest()

th1 = Thread(target=checkserial, args = ('',''))
th2 = Thread(target=principal, args = ('',''))
th1.start()
th2.start()
