# Gazebo Simulation for HiPeR Lab at UC Berkeley
* ROS Kinetic
* Ubuntu 16.04
* Gazebo v 7.0.0 (should come with ROS Kinetic)

## Setup
1. Make sure ROS is installed. Tutorial can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)
2. Make sure the [lab code](https://github.com/muellerlab/GeneralCode) is properly installed.
3. Clone this repo into your catkin workspace.
```
cd ~/catkin_ws/src
git clone https://www.github.com/mchen0037/hiperlab_gazebo
```
4. Edit the ```makeEclipseProjects.sh``` so that you compile this new package. Include hiperlab_gazebo in the first line. ex:
```
catkin build hiperlab_rostools hiperlab_hardware hiperlab_components hiperlab_common hiperlab_gazebo --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -D_ECLIPSE_VERSION=4.6 -DCMAKE_CXX_COMPILER_ARG1=-std=c++11
```
5. Build the package. (Note: For some reason, I've had to run this twice, because something in rostools compiles before a custom message does, and it is dependent on that custom message.)
```
sh makeEclipseProjects.sh
```
6. Source the new packages *OR* add these to your ```.bashrc```
```
source ~/catkin_ws/devel/setup.bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/catkin_ws/devel/lib
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/hiperlab_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$~/catkin_ws/devel/lib
```
7. Checkout ```mighty/gazebosim``` branch in [GeneralCode](https://github.com/muellerlab/GeneralCode/tree/mighty/gazebosim).
```
git checkout mighty/gazebosim
```

Note: This works as of 8/6/18 with the most recent push to master (11f6501). If this doesn't work in the future, the only change I made to 11f6501 is including the physical properties of the Iris Model into a custom quadcopter type in ```Components/Components/Logic/QuadcopterConstants.hpp``` under case 37.


## Running the Simulator
Run the launch file to Gazebo environment with Iris Model.
```
roslaunch hiperlab_gazebo iris.launch
```
Run the rates controller.
```
rosrun hiperlab_rostools quad_mocap_rates_controller 37
```
Run joystick input.
```
rosrun hiperlab_hardware keyboard
```

