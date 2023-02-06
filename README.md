# HURON-UPPER
This respository holds the code and other materials for the HURON Upper Body project. 


## MATLAB Code
The MATLAB code folder holds multiple scrips that serve as references for the inverse kinematics of the torso. The following scripts are included:
 - Monte_Carlo_IK.m
     - This generates random rotations and translations of the actuations plate. Each transformation is checked with the lengths of the actuators to ensure it is a valid movement of the torso. Movements can be generated until a specified nuber of valid ones are found. The random movements are limited to ranges specified in the beginning of the script. The valid movements are then displayed in a 3d plot to the user. 
 - Torso_Control.m
     - In progress
     - Will allow the user to input pitch, roll, and yaw angles of the top plate, check if they are a valid movement, and then send them to the robot over a USB connection.
