This is a badly done program to mimic a Multiwii flight controller,
However the communication is not well done, it doesnt read any answers, just spits messages when receives a '<', which is part of the protocol header

If you are using linux:
	please install socat and use
	$ sudo socat PTY,link=/dev/ttyUSB10 PTY,link=/dev/ttyUSB11
The MultwiiEmulator will send data on /dev/ttyUSB10 and connect your program with /dev/ttyUSB11
