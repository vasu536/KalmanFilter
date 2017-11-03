# KalmanFilter

Here is the c++ code for a Kalman filter designed for a PUMA 3DOF robotic arm. This code is being used for velocity estimation as this is much more accurate than just differentiating position.

I made assumptions for my noise and sensor models to simplify the implementation. I also initialize my covariance as an identity matrix. In my code I let it converge and save it to a text file that I can read every time I start the filter.
