# robotics-2021-projects
This repository contains the projects for the 2021 Robotics [089013] course held at Politecnico di Milano by Prof. Matteo Matteucci. Each folder of the repo is a ROS package.

## Team members
- Alberto Giusti - MSc student, Politecnico di Milano
- Andrea Lanubile - MSc student, Politecnico di Milano
- Alessandro Nardi - MSc student, Politecnico di Milano

## Project 1 - AgileX Scout 2.0 odometry

AgileX Scout 2.0 odometry computation with ROS.

### Install
Inside your `<catkin_workspace>/src/` folder:

		git clone https://github.com/zabaio/robotics-2021-projects.git source
		mv source/robotics_hw1 source/scout_odometry .
		rm -rf source
		cd .. && catkin_make

### Usage

		roslaunch scout_odometry scout_odometry.launch 

This will run two nodes, `synchronizer` and `integrator` under the namespace `scout`.

Now publish synchronized `robotics_hw1/MotorSpeed` messages on topics`/motor_speed_{fl|fr|rl|rr}`.
>  The synchronization filtering policy is ApproximateTime.

- `synchronizer` will read the `MotorSpeeds` and convert them to the linear and angular velocity of the robot.
- `integrator` will read  linear and angular velocity publish
    - the `tf` between frame `odom` and frame `scout`
    - an `Odometry` message on topic `/scout/scout_odom`
    - and a `State` message, containing the `Odometry` and the integration method, on topic `/scout/scout_state`.

#### Features

- Service `/scout/reset_pose`. Will place the robot in `[x=0, y=0, theta=0]` and reset its speed.
- Service `/scout/set_pose`. Requires argument `<x> <y> <theta>`. Will place the robot in the requested pose, and reset its speed.
- Use dynamic reconfigure to modify `/scout/integrator/integration_method`. Supported methods:
    - Euler
    - Runge-Kutta

#### Visualization

To quickly visualize with `rviz` the computation you can use `scout_odometry/rviz/config.rviz`.

For a simulation run, you can use one of the bags in `scout_odometry/bag/`. They will publish, besides the motor speeds, the ground truth position measured with an Optitrack system, on topic `/gt_pose`.
You will also need to publish a static transform between `world` and `odom` frames, to take into account the starting position of the robot. For `bag1.bag` you can simply uncomment this line in `scout_odometry.launch`:

	<!-- <node pkg="tf2_ros" type="static_transform_publisher" name="world_odom_tf"
    	args="-0.53 0.1 0.32  0 0 0.5311862 0.8472551 world odom" /> -->
