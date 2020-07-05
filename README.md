# MPC-quadrotor
Practical Assignment Robot Motion Planning and Control
Group 36 - MAV - MPC
Q3 2019/202
------------------------------------------------------

For simulating the racing drone, the MPC_main.m file has
to be run. This file generates the simulation environment
including the randomly placed rings. Furthermore, the 
settings for the MPC algorithm can be adjusted.

The function file MPC_Controller.m is called within the
main file. This function file calculates the waypoints
based on the location of the rings. It then sets up
a optimization problem and solves it while also simulating
the system. It outputs the state of the quadrotor over the
simulation time.

Best regards,

Daan Bruggink
Bart de Jong
Tom Kerssemakers
Mike Pesselse
