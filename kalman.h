#ifndef _Kalman_h_
#define _Kalman_h_

typedef struct {
float Q_angle; // Process noise variance for the accelerometer
float Q_bias; // Process noise variance for the gyro bias
float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector

float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
} Kalman;

void kalman_init(Kalman *p_kalman);
float kalman_get_angle(Kalman *p_kalman, float newAngle, float newRate, float dt);
void setAngle(Kalman *p_kalman, float angle);
float getAngle(Kalman *p_kalman);

#endif