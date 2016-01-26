#! /usr/bin/env python
# -*- coding:utf-8 -*-
# 
__author__ = 'Patrick'
import serial
import struct
import sys
from multwii_const import *
from provant_const import *
from array import *



class ProvantSerial:
    def __init__(self,window = None, serial_name='/dev/ttyVirtual2', baudrate_value=460800, debug_mode=False):
        ser = serial.Serial(serial_name, baudrate_value)
        ser.flush()
        self.window = window
        self.ser = ser
        self.debug = debug_mode
        self.attitude = Attitude()
        self.raw_gps = Raw_gps()
        self.comp_gps = Comp_gps()
        self.analog = Analog()
        self.altitude = Altitude()
        self.status = Status()
        self.debug = Debug()
        self.rc = Rc()
        self.rcn = Rcnormalize()
        self.pid = Pid()
        self.ident = Ident()
        self.servo = Servo()
        self.motor_pins = Motor_pins()
        self.motor = Motor()
        self.imu = RawIMU()
        self.controldatain = Controldatain()
	self.controldataref = Controldataref()
        self.controldataout = Controldataout()
        self.escdata = Escdata()
	self.carga = Carga()
	self.status = Status()
        self.sampleCount=0
	byte_buffer = bytearray()
	global checksum
	checksum = [0, 0, 0, 0]
	servo = 0
	self.control = "s"
	self.s = ""

##### ENCODING FUNCTIONS ###################################


    def serialize8(self, a):
	global byte_buffer
	global checksum
	if isinstance(a, int):
		a = chr(a)
	byte_buffer += a;
	checksum[0] ^= ord(a);


    def serialize16(self, a):
	self.serialize8((a   ) & 0xFF);
	self.serialize8((a >> 8) & 0xFF);


    def serialize32(self, a):
	self.serialize8((a    ) & 0xFF);
	self.serialize8((a >> 8) & 0xFF);
	self.serialize8((a >> 16) & 0xFF);
	self.serialize8((a >> 24) & 0xFF);

    def serializeFloat(self, a):
	b=struct.pack('<f', a)
	for x in xrange(0,4):
	    self.serialize8(b[x])

    def headSerialResponse(self, s, type):
	global byte_buffer
	global checksum
	byte_buffer = bytearray()
	self.serialize8('$');
	self.serialize8('M');
	self.serialize8('>');
	checksum[0] = 0;  # // start calculating a new checksum
	self.serialize8(s);
	self.serialize8(type);


    def headSerialReply(self, s, type):
	self.headSerialResponse(s, type);


    def tailSerialReply(self):
	global byte_buffer
	global checksum
	byte_buffer += chr(checksum[0])


    def send_gps(self, lat=0, lon=0, numsats=5, alt=0, speed=0, fix=1):
	GPS_ground_course = 0
	self.headSerialReply(16, MSP_RAW_GPS)
	self.serialize8(fix)
	self.serialize8(numsats)
	self.serialize32(lat)
	self.serialize32(lon)
	self.serialize16(alt)
	self.serialize16(speed)
	self.serialize16(GPS_ground_course)
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))

    def sendControldatain(self,rpy=[0,0,0],drpy=[0,0,0],position=[0,0,0],velocity=[0,0,0],servo=[0,0],dservo=[0,0]):
	self.headSerialReply(64, MSP_CONTROLDATAIN)
	for x in xrange(0,3):
	    self.serializeFloat(rpy[x])
	for x in xrange(0,3):
	    self.serializeFloat(drpy[x])
	for x in xrange(0,3):
	    self.serializeFloat(position[x])
	for x in xrange(0,3):
	    self.serializeFloat(velocity[x])
	for x in xrange(0,2):
	    self.serializeFloat(servo[x])
	for x in xrange(0,2):
	    self.serializeFloat(dservo[x])
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))

    def sendControldataref(self,r_rpy=[0,0,0],r_drpy=[0,0,0],r_position=[0,0,0],r_velocity=[0,0,0],r_servo=[0,0],r_dservo=[0,0]): ##Testeeee
	self.headSerialReply(64, MSP_CONTROLDATAREF)
	for x in xrange(0,3):
	    self.serializeFloat(r_rpy[x])
	for x in xrange(0,3):
	    self.serializeFloat(r_drpy[x])
	for x in xrange(0,3):
	    self.serializeFloat(r_position[x])
	for x in xrange(0,3):
	    self.serializeFloat(r_velocity[x])
	for x in xrange(0,2):
	    self.serializeFloat(r_servo[x])
	for x in xrange(0,2):
	    self.serializeFloat(r_dservo[x])
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))

    def send_carga(self, x=[0], dx=[0], y=[0], dy=[0], z=[0], dz=[0]):
	self.headSerialReply(24, MSP_CARGA)
	self.serializeFloat(x)
	self.serializeFloat(dx)
	self.serializeFloat(y)
	self.serializeFloat(dy)
	self.serializeFloat(z)
	self.serializeFloat(dz)
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))

    def sendControldataout(self,servo=[0,0],esc=[0,0,0,0]):
	self.headSerialReply(24, MSP_CONTROLDATAOUT)
	self.serializeFloat(servo[0])
	self.serializeFloat(esc[0])
	self.serializeFloat(esc[1])
	self.serializeFloat(servo[1])
	self.serializeFloat(esc[2])
	self.serializeFloat(esc[3])
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))

    def sendEscdata(self,rpm=[0,0],current=[0,0],voltage=[0,0]):
	self.headSerialReply(20, MSP_ESCDATA)
	for x in xrange(0,2):
	    self.serialize16(rpm[x])
	    self.serializeFloat(current[x])
	    self.serializeFloat(voltage[x])
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))

    def send_comp_gps(self, distance, direction):
	self.headSerialReply(5, MSP_COMP_GPS);
	self.serialize16(distance);
	self.serialize16(direction);
	self.serialize8(1);
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))


    def send_attitude(self, x=0, y=0, z=0):
	self.headSerialReply(6, MSP_ATTITUDE)
	self.serialize16(x * 10)
	self.serialize16(y * 10)
	self.serialize16(z * 10)
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))


    def send_analog(self, vbat=15, power=0, rssi=0, current=1234):
	self.headSerialReply(7, MSP_ANALOG)
	self.serialize8(vbat)
	self.serialize16(power)
	self.serialize16(rssi)
	self.serialize16(current)
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))


    def send_altitude(self, alt=123, vario=123):
	self.headSerialReply(6, MSP_ALTITUDE)
	self.serialize32(alt)
	self.serialize16(vario)
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))


    def send_status(self, stable=0, baro=0, mag=0):
	self.headSerialReply(10, MSP_STATUS)
	self.serialize16(0)
	self.serialize16(0)
	self.serialize16(0)
	self.serialize32(0)
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))
    
    def send_provant_status(self, sts):
	self.headSerialReply(4, MSP_PROVANT_STATUS)	
	self.serialize8(sts[0])
	self.serialize8(sts[1])
	self.serialize8(sts[2])
	self.serialize8(sts[3])
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))

    def send_debug(self, debug1, debug2, debug3,debug4):
	self.headSerialReply(8, MSP_DEBUG)
	self.serialize16(debug1)
	self.serialize16(debug2)
	self.serialize16(debug3)
	self.serialize16(debug4)
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))

    def send_debug_float(self,f_debug1,f_debug2,f_debug3,f_debug4): #Teste!!!!
	self.headSerialReply(16, MSP_DEBUG2)
	self.serializeFloat(f_debug1)
	self.serializeFloat(f_debug2)
	self.serializeFloat(f_debug3)
	self.serializeFloat(f_debug4)
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))

    def send_rc(self, channels):
	self.headSerialReply(12 * 2, MSP_RC)
	for i in range(12):
	    self.serialize16(channels[i])
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))


    def send_pid(self):
	self.headSerialReply(10 * 3, MSP_PID)
	for i in range(10):
	    self.serialize8(123)
	    self.serialize8(234)
	    self.serialize8(78)
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))


    def send_bicopter_identifier(self):
	self.headSerialResponse(7, MSP_IDENT);
	self.serialize8(32);
	self.serialize8(4);  # codigo do bicoptero
	self.serialize8(0);  # not used
	self.serialize32(0);
	self.tailSerialReply();
	self.ser.write(str(byte_buffer))


    def send_servos(self, servo_esquerdo,servo_direito):
	self.headSerialResponse(16, MSP_SERVO);

	self.serialize16(1500);
	self.serialize16(1500);
	self.serialize16(1500);
	self.serialize16(1500);

	self.serialize16(servo_esquerdo);
	self.serialize16(servo_direito);
	self.serialize16(1500);
	self.serialize16(1500);

	self.tailSerialReply();
	self.ser.write(str(byte_buffer))

    def send_raw_imu(self, gyro, acc, mag):
	self.headSerialResponse(18, MSP_RAW_IMU)
	for i in range(3):
	    self.serialize16(gyro[i])
	for i in range(3):
	    self.serialize16(acc[i])
	for i in range(3):
	    self.serialize16(mag[i])
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))


    def send_motor_pins(self):
	self.headSerialResponse(8, MSP_MOTOR_PINS)
	self.serialize8(1)
	self.serialize8(2)
	self.serialize8(0)
	self.serialize8(0)

	self.serialize8(0)
	self.serialize8(0)
	self.serialize8(0)
	self.serialize8(0)

	self.tailSerialReply();
	self.ser.write(str(byte_buffer))

    def send_string(self, string):
	self.headSerialResponse(len(string), MSP_STRING)
	for i in range(len(string)):
		self.serialize8(string[i])
	self.tailSerialReply();
	self.ser.write(str(byte_buffer))

    def send_motor(self, forca_esquerdo,forca_direito):
	self.headSerialResponse(16, MSP_MOTOR)
	self.serialize16(forca_esquerdo);
	self.serialize16(forca_direito);
	self.serialize16(1300);
	self.serialize16(1300);

	self.serialize16(1300);
	self.serialize16(1300);
	self.serialize16(1300);
	self.serialize16(1300);

	self.tailSerialReply();
	self.ser.write(str(byte_buffer))

#provant_msg
    def send_rc_normalize(self, channels):
	MSP_RCNORMALIZE             =96
	self.headSerialReply(12 * 2, MSP_RCNORMALIZE)
	for i in range(12):
	    self.serialize16(channels[i])
	self.tailSerialReply()
	self.ser.write(str(byte_buffer))
###################################################################################################################3
    def decodeFloat(self, data):
        return struct.unpack('<f', ''.join(data))[0]

    def decode32(self, data):
        #print data
        result = (ord(data[0]) & 0xff) + ((ord(data[1]) & 0xff) << 8) + ((ord(data[2]) & 0xff) << 16) + ((ord(data[3]) & 0xff) << 24)
        is_negative = ord(data[3]) >= 128
        if is_negative:
            result -= 2**32
        return result

    def decode16(self, data):
        #print data
        result = (ord(data[0]) & 0xff) + ((ord(data[1]) & 0xff) << 8)
        is_negative = ord(data[1]) >= 128
        if is_negative:
            result -= 2**16
        return result

    def decode216(self, result):
        #print data
        is_negative = ((result>>8)&0xff) >= 128
        if is_negative:
            result -= 2**16
        return result

    def checksum_matches(self):
        check = self.who ^ self.size
        for x in xrange(0, self.size):
            check ^= ord(self.L[x])
        if((check == ord(self.L[self.size]))):
            self.sampleCount+=1
        return (check == ord(self.L[self.size]))

    def update(self):
        while self.ser.inWaiting() > 10:
            self.takeHead()

    def takeHead(self):
        if (ord(self.ser.read()) == MSP_HEAD[0]):  # checkhead1
            if (ord(self.ser.read()) == MSP_HEAD[1]):  #checkhead2
                if (ord(self.ser.read()) == MSP_HEAD[2]):  #checkhead3
                    self.solve_type()

    def solve_type(self):
        self.size = ord(self.ser.read())  # pega tamanho
        self.who = ord(self.ser.read())  # descobre quem é
        self.word = self.ser.read(self.size + 1)  # pega os dados + checksum
        self.L = list(self.word)  # passa para uma lista
        self.takeData()

    def readSampleCount(self):
        return self.sampleCount

###############  PROPER MESSAGE DECODING IS HERE ##################################

    def takeData(self):
        if (self.who == MSP_ATTITUDE):
            if self.checksum_matches():
                self.attitude.roll = self.decode16(self.L[0:2])/10
                self.attitude.pitch = self.decode16(self.L[2:4])/10
                self.attitude.yaw = self.decode16(self.L[4:6])

                if self.window:
                    self.window.addArray('Attitude.Filtered',
                                         (self.attitude.roll, self.attitude.pitch, self.attitude.yaw),
                                         ('Roll','Pitch','Yaw'))



        if (self.who == MSP_RAW_GPS):
            if self.checksum_matches():
                self.raw_gps.fix = ord(self.L[0])
                self.raw_gps.numsats = ord(self.L[1])
                self.raw_gps.lat = self.decode32(self.L[2:6])
                self.raw_gps.lon = self.decode32(self.L[6:10])
                self.raw_gps.alt = self.decode16(self.L[10:12])
                self.raw_gps.speed = self.decode16(self.L[12:14])
                self.raw_gps.ggc = self.decode16(self.L[14:16])

                if self.window:
                    self.window.addArray('GPS',
                                         (self.raw_gps.fix,self.raw_gps.numsats,self.raw_gps.lat,self.raw_gps.lon,self.raw_gps.alt,self.raw_gps.speed,self.raw_gps.ggc),
                                         ('Fix','Numsats','Lat','Long','Alt','Veloc','GGC'))

        if (self.who == MSP_COMP_GPS):
            if self.checksum_matches():
                self.comp_gps.distance = self.decode16(self.L[0:2])
                self.comp_gps.direction = self.decode16(self.L[2:4])
                self.comp_gps.update = ord(self.L[4])

                if self.window:
                    self.window.addArray('Comp_gps',
                                         (self.comp_gps.distance,self.comp_gps.direction,self.comp_gps.update),
                                         ('Dist','Direction','Update'))                

        if (self.who == MSP_ANALOG):
            if self.checksum_matches():
                self.analog.vbat = ord(self.L[0])
                self.analog.power = self.decode16(self.L[1:3])
                self.analog.rssi = self.decode16(self.L[3:5])
                self.analog.current = self.decode16(self.L[5:7])

                if self.window:
                    self.window.addArray('Analog',
                                         (self.analog.vbat, self.analog.power,self.analog.rssi,self.analog.current),
                                         ('Vbat','Power','Rssi','Current'))

        if (self.who == MSP_ALTITUDE):
            if self.checksum_matches():
                self.altitude.alt = self.decode32(self.L[0:4])
                self.altitude.vario = self.decode16(self.L[4:6])

                if self.window:
                    self.window.addArray('Altitude',
                                         (self.altitude.alt,self.altitude.vario),
                                         ('Alt','Vario'))
                    self.window.verticalSlider_3.setValue(self.altitude.alt)


        if (self.who == MSP_STATUS):
            if self.checksum_matches():
                self.status.cycleTime = self.decode16(self.L[0:2])
                self.status.i2cec = self.decode16(self.L[2:4])
                self.status.sensor = self.decode16(self.L[4:6])
                self.status.flag = self.decode32(self.L[6:10])
                self.status.gccs = ord(self.L[10])

                if self.window:
                    self.window.addArray('Status',
                                         (self.status.cycleTime,self.status.i2cec,self.status.sensor,self.status.flag,self.status.gccs),
                                         ('CycleTime','Numi2cerror','Sensor','Flag','Gccs'))

        if (self.who == MSP_DEBUG):
            if self.checksum_matches():
                for i in xrange(0, self.size / 2):
                    self.debug.debug[i] = self.decode16(self.L[i*2:i*2+2])
                
                if self.window:
                    self.window.addArray('Debug', self.debug.debug)


        if (self.who == MSP_RC):
            if self.checksum_matches():
                for x in xrange(0, self.size / 2):
                    self.rc.channel[x] = ord(self.L[x * 2]) + (ord(self.L[x * 2 + 1]) << 8)

                if self.window:
                    self.window.addArray('RC_channel', self.rc.channel)

        if (self.who == MSP_PID):
            if self.checksum_matches():
                for x in xrange(0, self.size):
                    self.pid.pid[x] = ord(self.L[x])


        if (self.who == MSP_IDENT):
            if self.checksum_matches():
                self.ident.version = ord(self.L[0])
                self.ident.multtype = ord(self.L[1])
                self.ident.mspversion = ord(self.L[2])
                self.ident.capability = ord(self.L[3]) + (ord(self.L[4]) << 8) + (ord(self.L[5]) << 16) + (
                ord(self.L[6]) << 24)

        if (self.who == MSP_SERVO):
            if self.checksum_matches():
                for x in xrange(0, self.size / 2):
                    self.servo.servo[x] = self.decode16(self.L[x*2:x*2+2])
                if self.window:
                    self.window.addArray('ServoAngle',
                                         self.servo.servo[4:6],
                                         ('LServoAngle', 'RServoAngle'))


        if (self.who == MSP_MOTOR_PINS):
            if self.checksum_matches():
                pins = [None] * (self.size)
                for x in xrange(0, self.size):
                    self.motor_pins.pin[x] = ord(self.L[x])

        if (self.who == MSP_RAW_IMU):
            if self.checksum_matches():
                for i in range(3):
                    self.imu.acc[i] = self.decode16(self.L[i*2:i*2+2])
                for i in range(3, 6):
                    self.imu.gyr[i-3] = self.decode16(self.L[i*2:i*2+2])
                for i in range(6, 9):
                    self.imu.mag[i-6] = self.decode16(self.L[i*2:i*2+2])

                if self.window:
                    self.window.addArray('Attitude.Gyro', self.imu.gyr,('X','Y','Z'))
                    self.window.addArray('Attitude.Acc', self.imu.acc,('X','Y','Z'))
                    self.window.addArray('Attitude.Mag', self.imu.mag,('X','Y','Z'))

        if (self.who == MSP_MOTOR):
            if self.checksum_matches():
                for x in xrange(0, self.size / 2):
                    self.motor.motor[x] = ord(self.L[x * 2]) + (ord(self.L[x * 2 + 1]) << 8)

                if self.window:
                    self.window.addArray('MotorSetpoint',
                                         self.motor.motor[0:2],
                                         ('Lmotor', 'Rmotor'))
                    self.window.lMotorSetpoint.setValue(self.motor.motor[0])
                    self.window.rMotorSetpoint.setValue(self.motor.motor[1])



        if (self.who == MSP_CONTROLDATAIN):
            if self.checksum_matches():
                for x in xrange(0, 3):
                    self.controldatain.rpy[x]= self.decodeFloat(self.L[x*4:4+x*4])
                for x in xrange(3, 6):
                    self.controldatain.drpy[x-3]= self.decodeFloat(self.L[x*4:4+x*4])
                for x in xrange(6, 9):
                    self.controldatain.position[x-6]= self.decodeFloat(self.L[x*4:4+x*4])
                for x in xrange(9, 12):
                    self.controldatain.velocity[x-9]= self.decodeFloat(self.L[x*4:4+x*4])
		for x in xrange(12,14):
		    self.controldatain.servo[x-12] = self.decodeFloat(self.L[x*4:4+x*4])
		for x in xrange(14,16):
		    self.controldatain.dservo[x-14] = self.decodeFloat(self.L[x*4:4+x*4])
                if self.window:
                    data = self.controldatain
                    self.window.addArray("Data.rpy",
                                         data.rpy,)
                    self.window.addArray("Data.drpy",
                                         data.drpy,)
                    self.window.addArray("Data.Position",
                                         data.position,)
                    self.window.addArray("Data.Velocity",
                                         data.velocity,)
		    self.window.addArray("Data.servo", 
                                         data.servo)
		    self.window.addArray("Data.dservo",
                                         data.dservo)

        if (self.who == MSP_CONTROLDATAREF):
            if self.checksum_matches():
                for x in xrange(0, 3):
                    self.controldataref.r_rpy[x]= self.decodeFloat(self.L[x*4:4+x*4])
                for x in xrange(3, 6):
                    self.controldataref.r_drpy[x-3]= self.decodeFloat(self.L[x*4:4+x*4])
                for x in xrange(6, 9):
                    self.controldataref.r_position[x-6]= self.decodeFloat(self.L[x*4:4+x*4])
                for x in xrange(9, 12):
                    self.controldataref.r_velocity[x-9]= self.decodeFloat(self.L[x*4:4+x*4])
		for x in xrange(12,14):
		    self.controldataref.r_servo[x-12] = self.decodeFloat(self.L[x*4:4+x*4])
		for x in xrange(14,16):
		    self.controldataref.r_dservo[x-14] = self.decodeFloat(self.L[x*4:4+x*4])
                if self.window:
                    dataref = self.controldataref
                    self.window.addArray("Dataref.rpy",
                                         dataref.r_rpy)
                    self.window.addArray("Dataref.drpy",
                                         dataref.r_drpy)
                    self.window.addArray("Dataref.Position",
                                         dataref.r_position)
                    self.window.addArray("Dataref.Velocity",
                                         dataref.r_velocity,)
		    self.window.addArray("Dataref.servo", 
                                         dataref.r_servo)
		    self.window.addArray("Dataref.dservo",
                                         dataref.r_dservo)

	if (self.who == MSP_CARGA):
	    if self.checksum_matches():
		self.carga.x[0] = self.decodeFloat(self.L[0:4])
		self.carga.dx[0] = self.decodeFloat(self.L[4:8])
		self.carga.y[0] = self.decodeFloat(self.L[8:12])
		self.carga.dy[0] = self.decodeFloat(self.L[12:16])
		self.carga.z[0] = self.decodeFloat(self.L[16:20])
		self.carga.dz[0] = self.decodeFloat(self.L[20:24])
		if self.window:
			data = self.carga
			self.window.addArray("Carga.x", data.x)
			self.window.addArray("Carga.dx", data.dx)
			self.window.addArray("Carga.y", data.y)
			self.window.addArray("Carga.dy", data.dy)
			self.window.addArray("Carga.z", data.z)
			self.window.addArray("Carga.dz", data.dz)

	if (self.who == MSP_PROVANT_STATUS):
		for i in range(len(self.L)-1):
			self.status.msg.append(self.decode8(self.L[i]))
			print self.status.msg

        if (self.who == MSP_CONTROLDATAOUT):
            if self.checksum_matches():
                self.controldataout.servoLeft = self.decodeFloat(self.L[0:4])
                self.controldataout.escLeftNewtons  = self.decodeFloat(self.L[4:8])
                self.controldataout.escRightNewtons = self.decodeFloat(self.L[8:12])
                self.controldataout.servoRight = self.decodeFloat(self.L[12:16])
                self.controldataout.escLeftSpeed = self.decodeFloat(self.L[16:20])
                self.controldataout.escRightSpeed = self.decodeFloat(self.L[20:24])
                if self.window:
                    data = self.controldataout
                    self.window.addArray("ActuatorsCommand.Left",
                                         (data.escLeftNewtons, data.escLeftSpeed, data.servoLeft),
                                         ('Newtons', 'Speed', 'Servo'))
                    self.window.addArray("ActuatorsCommand.Right",
                                         (data.escRightNewtons, data.escRightSpeed, data.servoRight),
                                         ('Newtons', 'Speed', 'Servo'))
        if (self.who == MSP_ESCDATA):
            if self.checksum_matches():
                for x in xrange(0, 2):
                    self.escdata.rpm[x] = self.decode16(self.L[x*10:x*10+2])
                    self.escdata.current[x] = self.decodeFloat(self.L[x*10+2:x*10+6])
                    self.escdata.voltage[x] = self.decodeFloat(self.L[x*10+6:x*10+10])
                if self.window:
                    for i, escName in enumerate(['EscFeedbackLeft', 'EscFeedbackRight']):
                        self.window.addArray(escName,
                                             (self.escdata.rpm[i], self.escdata.current[i], self.escdata.voltage[i]),
                                             ('Rpm', 'Current', 'Voltage'))
                    self.window.lMotorRpm.setValue(self.escdata.rpm[0])
                    self.window.rMotorRpm.setValue(self.escdata.rpm[1])

	if (self.who == MSP_STRING):
		if self.checksum_matches():
			self.s = ""
			for i in range(len(self.L) -1):
				self.s = self.s + self.L[i]
			print self.s
			if (self.s == "i") or (self.s == "r") or (self.s == "s"):
				self.control = self.s
			else:
				pass
			


        if (self.who == MSP_RCNORMALIZE):
            if self.checksum_matches():
                for x in xrange(0, self.size / 2):
                    self.rcn.channel[x] = self.decode16(self.L[x*2:x*2+2])
                #test
                if self.window:
                    self.window.addArray('RCN', self.rcn.channel[0:7])
                    self.window.left_joystick.move(self.rcn.channel[3], self.rcn.channel[2])
                    self.window.right_joystick.move(self.rcn.channel[0],self.rcn.channel[1])

   #                 self.window.verticalSlider_2.setValue(self.rcn.channel[0])
                    self.window.radioButton.setAutoExclusive(False);
                    self.window.radioButton_2.setAutoExclusive(False);
                    if self.rcn.channel[5]>50:
                        self.window.radioButton.setChecked(1)
                    else:
                        self.window.radioButton.setChecked(0)
                    if self.rcn.channel[6]>50:
                        self.window.radioButton_2.setChecked(1)
                    else:
                        self.window.radioButton_2.setChecked(0)
                    self.window.dial.setValue(self.rcn.channel[4])

if __name__ == '__main__':
    provant = ProvantSerial()

    while (1):
        provant.update()
        print("attitude", provant.attitude.roll, provant.attitude.pitch, provant.attitude.yaw)
        print("gps raw", provant.raw_gps.fix, provant.raw_gps.numsats, provant.raw_gps.lat, provant.raw_gps.lon,
              provant.raw_gps.alt, provant.raw_gps.speed, provant.raw_gps.ggc)
        print("gps comp", provant.comp_gps.distance, provant.comp_gps.direction, provant.comp_gps.update)
        print("analog", provant.analog.vbat, provant.analog.power, provant.analog.rssi, provant.analog.current)
        print("altitude", provant.altitude.alt, provant.altitude.vario)
        print("status", provant.status.cycleTime, provant.status.i2cec, provant.status.sensor, provant.status.flag,
              provant.status.gccs)
        print("debug", provant.debug.debug)
        print("rc", provant.rc.channel)
        print("pid", provant.pid.pid)
        print("ident", provant.ident.version, provant.ident.multtype, provant.ident.mspversion, provant.ident.capability)
        print("imu:", provant.imu.gyr,provant.imu.acc,provant.imu.mag)
        print("servo", provant.servo.servo)
        print("motor pins", provant.motor_pins.pin)
        print("motor", provant.motor.motor)
        print("controldatain",provant.controldatain.rpy,provant.controldatain.drpy,provant.controldatain.position,provant.controldatain.velocity,
provant.controldatain.servo,provant.controldatain.dservo)
	print("controldataref",provant.controldataref.r_rpy,provant.controldataref.r_drpy,provant.controldataref.r_position,provant.controldataref.r_velocity,                               provant.controldataref.r_servo, provant.controldataref.r_dservo )
        print("controldataout",provant.controldataout.servoLeft,provant.controldataout.escLeftNewtons,provant.controldataout.escLeftSpeed,provant.controldataout.servoRight,
provant.controldataout.escRightNewtons,provant.controldataout.escRightSpeed)
        print("escdata",provant.escdata.rpm,provant.escdata.current,provant.escdata.voltage)
