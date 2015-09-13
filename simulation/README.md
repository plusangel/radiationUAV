## Simulation - Obstacle Avoidance Application

The hector quad-rotor simulation customised and an additional packega, my_hector, to perform assisted teleoperation (obstacle avoidance)

In this simulation the operator controls the flight manually using a joystick or the keyboard.  If the UAV encounters an obstacle in front of it or senses something closer than 1m around it, then the control passes to the obstacle avoidance algorithm.  From this point, the UAV based on the FSM will perform a single or a series of manoeuvers.  When it assess that there is no obstacle to avoid, it will pass the control back to the operator.  After the avoidance procedure, the algorithm will try to deliver the UAV in the initial heading that this had before the algorithm took the control.  In complicated cases, when the UAV performs a sequence of maneuvers this could not be possible.  This description corresponds to an assistive teleoperation solution where the application helps the operator by performing the avoidance manoeuvers.

----

In the “Simulation Background and Configuration” appendix there is a section related to the general description of ROS, Gazebo as well as a terminology section.  In addition, there is a configuration section that includes useful guides in order to allow the easy setup of the simulation systems, regarding both hardware and software.  A technical description that explains the implementation details of the obstacle avoidance application can be found in the “Simulation Development” appendix.
