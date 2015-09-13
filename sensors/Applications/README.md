## Sensors Applications

Mapping application - In the application, the laser sensor performs 180o sweep and every to two degrees the current angle and the measured distance are transmitted to the serial port.  On the other side, Matlab converts the data to Cartesian coordinates and applies the RANSAC algorithm to identify existing lines.   

Proximity application - This application was implemented in order to measure the distance in individual predefined directions.  This application was implemented using the ROS framework and can be utilised by any ROS-enabled robotic structure.  For this project the predefined directions are at -90o, 0o and 90o.

Please read the "Sensors Circuits" appendix for more technical details
