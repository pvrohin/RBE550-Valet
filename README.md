# RBE550-Valet
This assignment involves path planning in a constrained environment that pose specific kinematic constraints for each vehicle

## The algorithm for the above assignment 

This is the RBE550 Valet homework assignment, completed for the Spring 2023 term.

The `RBE550_Valet.pdf` contains the assignment writeup. The `videos` folder contains examples of each of these robots working.

The following scripts are your entrypoints:

* `drive_diff_drive.py` - Navigate a map with the skid drive robot
* `drive_ackermann.py` - Navigate a map with the Ackermann drive robot
* `trailer_drive.py` - Navigate a map with the Ackermann drive robot and an attached trailer

Each robot script will automatically place the initial pose of the robot at a fixed location. From this pose, the robot will navigate to the goal location using the algorithm as shown in the image below

<img src="Give goal location.png"/>

Te results are shown below:

<img src="videos/skid.gif" width=300/>
<img src="videos/ackermann.gif" width=300/>
