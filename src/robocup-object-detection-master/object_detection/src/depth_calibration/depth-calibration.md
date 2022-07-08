# Camera calibration
This folder is for calibrating the realsense camera so it can put objects in 3d space.

## The camera model
1. There is a translation and rotation to go from the depth camera to the tracking camera
1. There is translation and rotation to get from the tracking camera to a stationary reference frame

The second translation and rotation is recorded by the tracking camera. The first rotation and translation is a property of the mount and it's hard to measure

If $\mathbf{x_d}$ is the point relative to the depth camera and $\mathbf{x_w}$ is the position in the world coordinates:

$$
\mathbf{x_w} = Q_t\left(Q_m\mathbf{x_p} + \mathbf{t_m}\right) + \mathbf{t_t}
$$

The task is to find $Q_m$ and $\mathbf{t_m}$

## collect_raw_data.cpp 
This file takes the depth and tracking camera streams and records the position data for a QR code. It records:

$\mathbf{x_p}$, $Q_t$ and $\mathbf{t_t}$ at different times.

## calibrate.ipynb

$Q$ is an orthogonal matrix so it can be described using 3 angles (pitch, roll, yaw).  
The task is then to minimise the variance in position (the average square distance to the centre. This puts all the points close together in space and doesn't require knowledge of the actual position $\mathbf{x_w}$) by varying the 6 parameters for $Q_m$ and $\mathbf{t_m}$