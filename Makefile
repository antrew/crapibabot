HDRS = helper_3dmath.h I2Cdev.h MPU6050_6Axis_MotionApps20.h MPU6050.h
CMN_OBJS = I2Cdev.o MPU6050.o MyMPU6050.o ComplementaryFilter.o Motor.o Encoder.o
MY_OBJS = Control.o
CFLAGS=-Wall

all: control sensor test_ir

$(CMN_OBJS) $(MY_OBJS) : $(HDRS)


control: $(CMN_OBJS) $(MY_OBJS)
	$(CXX) -o $@ $^ -lwiringPi -lm -lrt

sensor: $(CMN_OBJS) MyMPU6050.o Sensor.o
	$(CXX) -o $@ $^ -lwiringPi -lm -lrt

test_ir: test_ir.c
	$(CXX) $(CFLAGS) -o $@ $^ -lwiringPi -I.

test_echo: test_echo.c
	$(CXX) $(CFLAGS) -o $@ $^ -lwiringPi -lrt -I.

clean:
	rm -f $(CMN_OBJS) $(MY_OBJS) control

