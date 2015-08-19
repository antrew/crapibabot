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
int port_motor_left_pwm = 18;
//
//		# input 1 of the H-bridge
int port_motor_left_forward = 24;
//
//		# input 2 of the H-bridge
int port_motor_left_backward = 23;
//
//		# enable B of the H-bridge
int port_motor_right_pwm = 13;
//
//		# input 3 of the H-bridge
int port_motor_right_forward = 12;
//
//		# input 4 of the H-bridge
int port_motor_right_backward = 25;
//
int port_encoder_left_a = 19;
int port_encoder_left_b = 26;
//
int port_encoder_right_a=17;
int port_encoder_right_b=27;
