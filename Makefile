all: 
	gcc mpu6050.c kalman.c -o mpu6050 -lwiringPi -lm -rdynamic `pkg-config --libs gtk+-3.0` `pkg-config --cflags gtk+-3.0`
