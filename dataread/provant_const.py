__author__ = 'Patrick'

#MSP_HEAD                 ='$','M','>'
MSP_ESCDATA                 =99
MSP_CONTROLDATAOUT          =98
MSP_CONTROLDATAIN           =97
MSP_RCNORMALIZE             =96



class Controldatain:
	rpy						=[0]*3	#float[3]					
	drpy					=[0]*3	#float[3]
	position				=[0]*3	#float[3]
	velocity				=[0]*3	#float[3]

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


