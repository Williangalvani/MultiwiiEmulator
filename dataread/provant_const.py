__author__ = 'Patrick'

#MSP_HEAD                 ='$','M','>'
MSP_ESCDATA                 =99
MSP_CONTROLDATAOUT          =98
MSP_CONTROLDATAIN           =97
MSP_CONTROLDATAREF          =95
MSP_RCNORMALIZE             =96
MSP_CARGA		    =94
MSP_PROVANT_STATUS	    =93

class Controldataref:
	r_rpy					=[0]*3	#float[3]					
	r_drpy					=[0]*3	#float[3]
	r_position				=[0]*3	#float[3]
	r_velocity				=[0]*3	#float[3]
	r_servo					=[0]*2  #float[2]
	r_dservo				=[0]*2  #float[2]

class Controldatain:
	rpy					=[0]*3	#float[3]					
	drpy					=[0]*3	#float[3]
	position				=[0]*3	#float[3]
	velocity				=[0]*3	#float[3]
	servo					=[0]*2  #float[2]
	dservo					=[0]*2  #float[2]

class Status:
	msg					=[]

class Carga:
	x					=[0]    #float[1]
	dx					=[0]    #float[1]
	y					=[0]	#float[1]
	dy					=[0]	#float[1]
	z					=[0]	#float[1]
	dz					=[0]	#float[1]

class Controldataout:
	servoLeft				=0   	#float32
	escLeftNewtons			=0		#float32
	escLeftSpeed			=0		#float32

	servoRight				=0		#float32
	escRightNewtons		    =0		#float32
	escRightSpeed			=0		#float32
	

class Escdata:
	rpm						=[0]*2		#nibble
	current					=[0]*2		#char
	voltage					=[0]*2		#char		value*10

class Rcnormalize:
	channel		=[0]*12


