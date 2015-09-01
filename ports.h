//
//		#wiringpi2.wiringPiSetup() # For sequential pin numbering, one of these MUST be called before using IO functions
//		#wiringpi2.wiringPiSetupSys() # For /sys/class/gpio with GPIO pin numbering
//		#wiringpi2.wiringPiSetupGpio() # For GPIO pin numbering
//
//		wiringpi2.wiringPiSetupGpio() # For GPIO pin numbering
//
//
//		led_output_port=1
//
//		# MOTOR DRIVER PORTS
//
//		# enable A of the H-bridge
#define PORT_NUMBER_MOTOR_LEFT_PWM 18

//input 1 of the H-bridge
#define PORT_NUMBER_MOTOR_LEFT_FORWARD 24

//input 2 of the H-bridge
#define PORT_NUMBER_MOTOR_LEFT_BACKWARD 23

//enable B of the H-bridge
#define PORT_NUMBER_MOTOR_RIGHT_PWM 13

//input 3 of the H-bridge
#define PORT_NUMBER_MOTOR_RIGHT_FORWARD 12

//input 4 of the H-bridge
#define PORT_NUMBER_MOTOR_RIGHT_BACKWARD 25

#define PORT_NUMBER_ENCODER_LEFT_A 19
#define PORT_NUMBER_ENCODER_LEFT_B 26

#define PORT_NUMBER_ENCODER_RIGHT_A 27
#define PORT_NUMBER_ENCODER_RIGHT_B 17
