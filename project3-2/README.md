ENPM 661 - Project 3- Phase2 - A* Algorithm 
Kiran Kommaraju - komkiran - 120408366 
Abhinav Bhamidipati - abhinavb7 - 120424731

Github Link :https://github.com/KomK2/ENPM661/tree/hotfix/3-2
Branch: hotfix/3-2
### Instructions

## Part 01

You need the following libraries to run the code numpy, cv2, math, queue, matplotlib.pyplot, matplotlib.path

Steps to Run the code:

1. Run the given a_star_kiran_abhinav.py file.
2. Enter the x coordinates, y coordinates and orientations of the start and goal node one by one.
3. The orientations should be multiple of 30 degrees.
4. Enter the clearance of the robot.
5. You will get the output of optimal path on the map.
6. The video will get saved in the background with Node exploration and Optimal path.
7. The uploaded video has the start node as (200,200,60) and goal node as (5500,200). Taken Clearance is 150mm.

Video Link : https://drive.google.com/file/d/1MEY6ymRR-24Ugi8W5W1CLPFNxZ5jtnar/view?usp=drive_link


## Part 02

Copy the given package into our workspace

the path_plan.py is located in the scripts folder. 

1. colcon build
2. source install/setup.bash
3. ros2 launch turtlebot3_project3 competition_world.launch.py 

The above steps will launch the package

# To run the ros nodes

1. To run the A* ros2 run turtlebot3_project3 path_plan.py
2. Give the goal coordinates in terms of X and Y.
3. The uploaded Gazebo video has the following Goal coordinates (4500, 200)

Video Link : https://drive.google.com/file/d/1XC33JDitWcm1zKLBANkaWhPLNr_Pvb2O/view?usp=drive_link
