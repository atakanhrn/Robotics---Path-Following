# Robotics---Path-Following
Robot programming to follow checkpoints and orientation.
theta_diff = waypoint_theta - robot_theta
The equation above is used for angular velocity to follow the path. For the case where angle becomes negative, it is handled by some mathematical operations, which is commented in the code.

Speed was determined by distance calculation. In addition to this, if robot_theta is close enough to waypoint_theta additional speedup is given.

Additional check is done for the case where robot needs only rotate not move forward. A flag is used for this, so that other calculations know that this is a stop case and robot should not move forward.

Then, it is checked whether robot needs to go back. This is checked by robot_theta and x_diff being negative, also stopflag needs to be zero.

Then an angle fix is done for not moving further from the checkpoints. Because theta_diff gets the robot same angle with the waypoints, but sometimes robot can move further from the route little by little. Angle fix uses arctangent function and gets a weighted average depending the case.
