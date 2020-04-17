# MoveIt Control with HTC Vive
This package is made to be able to control MoveIt move groups with HTC vive. This is realized for:  
- [x] Controlling one arm at a time  
- [x] Two arms simultaniously  

This package is not optimized but should work as a proof of concept of how this could be realized.  

## How does it work (big picture)
It firstly uses three other packages, two for gazebo and moveIt configuration for a dual panda system [panda_dual_gazebo](https://github.com/Machine-Jonte/panda_dual_gazebo) and [panda_dual_gazebo_moveit_config](https://github.com/Machine-Jonte/panda_dual_gazebo_moveit_config), and then another for publishing the HTC Vive controllers pose, [vive_ros](https://github.com/Machine-Jonte/vive_ros). Note, the vive_ros package in my repository is a modified version of [vive_ros](https://github.com/robosavvy/vive_ros). 

<p align="center">
  <img src="./assets/images/VR_Control_grid.png" width="" title="Graph showing how the this node was used in relation to other packages.">
  <!-- <img src="your_relative_path_here_number_2_large_name" width="350" alt="accessibility text"> -->
</p>

Read pose information from controllers. When menu button is pressed the controllers relaitve position according to the robot can be moved.

## One arm scenario
The robot tries and go to the pose which the user is showing. This behavior can be a bit weird as it is not caring about trajectory but only end pose. This means that moving from A -> B can involve the pose to go through point C along the way which is not close to the current pose.

<p align="center">
  <img src="./assets/images/path_illustrator.png" width="" title="Graph showing how the this node was used in relation to other packages.">
  <!-- <img src="your_relative_path_here_number_2_large_name" width="350" alt="accessibility text"> -->
</p>

## Two arm scenario
When the trigger is pressed the relative position of the controllers from the robot is saved. Then when the gripper is pressed the robot goes to that pose. Collision of the two arms are accounted for and they shall not collide. Same goes here, the path between A -> B can go through an arbitary point C.

### Trajectory movement (dual arm)
There is an alternative (unsafe) mode of controlling the arms for follwing a trajectory instead. This is done by recording each pose who shall be in the path. Then moveit calculated the carthesian path. This trajectory is then directly fed to the arm controllers. This method do not account for collisions.
