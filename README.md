<h2 align="center">RoboND-Localization-Project</h2>
<h3 align="center">Where Am I?</h3>
<h4 align="left">Abstract</h4>
<p>Two different robot models are tested and evaluated in Gazebo and RVIZ utilizing Adaptive Monte Carlo Localization (AMCL) and the move-base plug in for navigation to successfully arrive at a goal that was either pre-determined or user-defined in real time.</p>
<h4 align="left">Introduction</h4>
<p>Autonomous navigation and localization is one of the most challenging tasks we have been tasked to acomplish. In order to do this, several things must happen; the robot must perceive its surroundings and interpret the sensor readings to localize itself. Once the robot has localized itself, or determined its position and pose it can decide on its next action and then continue onto executing that action</p>

<p>In order to do this two different methods were tested: Kalaman Filters (KF) and Monte Carlo Locatization (MCL). Based on the performance characterisitcs of Adaptive Monte Carlo Locatlization (AMCL) and its performance benifits over an Extended Kalman Filter(EKF) AMCL was chosen as the desired localization algorithm.</p>

<p>In this paper two different robot models are evauluated with two different LIDAR sensors using Adaptive MCL. With extensive expiramentation various parameters of the alogirthms are used to tune and optimize localization and navigation through the provided maze.</p>

<center><img src="/home/nickzanobini/catkin_ws/src/RoboND-Localization-Project/watermarked/lidars.png"></center>
<center><img src="/home/nickzanobini/catkin_ws/src/RoboND-Localization-Project/watermarked/maze.png"></center>

<h4 align="left">Simulation Results</h4>
<p>This project was simulated in Gazebo and RVIZ was used for visualization of the sensor readings, and how the different parameters help the robot localize itself.</p>

<p>To test out this simulation the following commands need to be run in their own terminal windows on an Ubuntu 16.04 LTS System with ROS Kinetic Kame</p>

<h5>Udacity Baseline Model</h5>

```
roslaunch udacity_bot udacity_world.launch
roslaunch udacity_bot amcl.launch
rosrun udacity_bot navigation_goal
```

<center><img src="/home/nickzanobini/catkin_ws/src/RoboND-Localization-Project/watermarked/udacity_bot_at_goal.png"></center>

<h5>Nicks Model</h5>

```
roslaunch nicks_bot udacity_world.launch
roslaunch nicks_bot amcl.launch
rosrun nicks_bot navigation_goal
```

<center><img src="/home/nickzanobini/catkin_ws/src/RoboND-Localization-Project/watermarked/nicks_bot_at_goal_point_cloud.png"></center>

<p>As shown in the images above, both robots were able to reach their goal location. The main difference between the two models is the nicks_bot package uses a Velodyne VLP-16 Lidar. This lidar is very popular commercailly and is commonly found on many self driving cars. The use of this lidar in simulation required the parameters of the map to be greatly reduced as the amount of data recived from the Velodyne Lidar was significantly higher than that of the Hokuyo Lidar. In order to achieve results I felt were acceptable I also had to fine tune the AMCL parameters beyond the defualt values that worked initally for the udacity_bot package.</p>

<h4 align="left">Discussion</h4>

<p>The Velodyne Lidar output a PointCloud2 by default and inorder to not have to change the behavior of the AMCL package the PointCloud2 data had to be converted into a LaserScan. This required settting a z value to average the sensor readings over inorder to determine where to estimate the laserscan at. Additonally I was unable to update the map and the controller as fast because this conversion took a significantly larger amount of processing power.</p>

<p>The nicks_bot robot clearly takes a more direct path to the goal and the particle filter converges significantly faster. However, due to the decreased frequency the robot is able to update itself they both achieve similar results, reaching the goal location in under five minutes.</p>


