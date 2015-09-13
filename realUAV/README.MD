## Real UAV

Basic approach - The human plays the role of the decision center.  More precisely, in this approach the operator gets real time information about the surrounding obstacles, and changes the control of the UAV accordingly.  The working scenario is that the operator controls a hex-copter via a radio transmitter and has real-time feedback about the distances around the UAV.  To improve the usability, the operator simply checks colour-encoded regions around the UAV on the screen instead of actual distances, since it is very difficult to control a UAV and at the same time read distance values.  Having this information, the operator can identify dangerous situations and keep away from obstacles in its proximity.

Advanced approach - The primary difference with the previous system is that in this case there is an onboard computer that makes the decisions for the avoidance manoeuvres and not the operator.  Hence, the onboard computer receives input data from the appropriate sensors and executes the obstacle avoidance algorithm, which was analyzed in the simulation part, in order to execute the required application.  The system was built using ROS providing us the required framework to develop the code that is capable to engage with all the individual system’s components.

Please read details about the technical implementation in the "Real UAV Development" appendix 
