all: 
	gcc mpu6050.c kalman.c six_axis_comp_filter.c -o mpu6050 -lwiringPi -lm -rdynamic `pkg-config --libs gtk+-3.0` `pkg-config --cflags gtk+-3.0` -g
