This is a badly done program to mimic a Multiwii flight controller,y
However the communication is not well done, it doesnt read any answers, just spits messages every X milliseconds.

If you are using linux:
	please install socat and use
	./socat.sh   (without sudo)
The MultwiiEmulator will send data on /dev/ttyUSB10, so connect your program with /dev/ttyUSB11
and