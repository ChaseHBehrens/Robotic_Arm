# Robotic Arm
The goal of this project was to enable the robot to detect,
extract, relocate, and sort items within its workspace. To enable the robot arm to perform these tasks, 
we used the DavitHartenberg convention to determine the forward kinematics and the inverse kinematics for the arm. 
We used Jacobian and differential kinematics to prevent the robot from moving into singularity positions.
The robot is a RRRR (Revolute-Revolute-RevoluteRevolute) robot according to the Davit-Hartenberg convention.
The arm is controlled by four DYNAMIXEL servo motors; the first one has its z-axis pointing upward, 
while the rest have their z-axes pointing out of the page.

<img width="1004" height="700" alt="image" src="https://github.com/user-attachments/assets/dd168f3d-2dd8-4b3d-b413-905da02b832d" />
