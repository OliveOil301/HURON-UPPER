# HURON-UPPER
This respository holds the code and other materials for the HURON Upper Body project. 


## MATLAB Code
The MATLAB code folder holds multiple scrips that serve as references for the inverse kinematics of the torso. The following scripts are included:

- Monte_Carlo_IK.m

     - This generates random rotations and translations of the actuations plate. Each transformation is checked with the lengths of the actuators to ensure it is a valid movement of the torso. Movements can be generated until a specified nuber of valid ones are found. The random movements are limited to ranges specified in the beginning of the script. The valid movements are then displayed in a 3d plot to the user.
- IK_Generation_GUI.m
     - Displays a simulated torso model to the user  with controls to change the roll, pitch, and yaw of the top plate. There is also a button to reset the position to the home position.
     - If the simulated robot is moved to a position that is unreachable by one or more actuators, the actuators that cannot move to that position are hilighted in red.
     - Allows the user to search for COM ports, choose the one that is conected to the robot, and open it. Also allows the user to close the COM port when finished.
     - Once a serial port has been opened, the user can sent a MOVE command to the robot with the actuator positions shown in the simulation.
