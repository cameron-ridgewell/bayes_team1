# bayes_team1
Bayesian Robotics Team1

##Installation
Installation of the required ROS dependencies is automated via the `install.sh` file. Navigate to `bayes_team1` and execute 

```
sudo sh install.sh
```

This should install the required ROS dependencies that are not present (below)

 * `jackal_desktop`
 * `jackal_simulator`

##World Launch and Control
To launch the jackal in Gazebo and the corresponding rviz control and visualization, first, perform a `catkin_make` in the `bayes_team1/catkin_ws` directory.
```
user@user:~/bayes_team1/catkin_ws$ catkin_make
user@user:~/bayes_team1/catkin_ws$ source ./devel/setup.bash
```
Launch using the `mbzirc_task2` launch file. This file will first transfer several models into the `~./gazebo` folder, and then initialize the world in Gazebo, spawning the `jackal_mbzirc` model, and adding control through rviz
```
user@user:~/bayes_team1/catkin_ws$ roslaunch mbzirc_task2 mbzirc_task2_sim.launch
```
