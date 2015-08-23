HDRS = helper_3dmath.h I2Cdev.h MPU6050_6Axis_MotionApps20.h MPU6050.h
CMN_OBJS = I2Cdev.o MPU6050.o MyMPU6050.o ComplementaryFilter.o
MY_OBJS = Control.o

all: control sensor

# Set DMP FIFO rate to 20Hz to avoid overflows on 3d demo.  See comments in
# MPU6050_6Axis_MotionApps20.h for details.

$(CMN_OBJS) $(MY_OBJS) : $(HDRS)

control: $(CMN_OBJS) $(MY_OBJS)
	$(CXX) -o $@ $^ -lwiringPi -lm -lrt

sensor: $(CMN_OBJS) MyMPU6050.o Sensor.o
	$(CXX) -o $@ $^ -lwiringPi -lm -lrt

clean:
	rm -f $(CMN_OBJS) $(MY_OBJS) control

