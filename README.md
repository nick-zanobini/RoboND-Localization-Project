### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
cd ~/catkin_ws/src
git clone https://bitbucket.org/DataspeedInc/velodyne_simulator
mv velodyne_simulator/velodyne_gazebo_plugins/ ./ && mv velodyne_simulator/velodyne_description/ ./
rm -rf velodyne_simulator
git clone https://github.com/nick-zanobini/RoboND-Localization-Project.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
catkin_make
source devel/setup.bash
```

Now from a terminal window:

```sh
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```