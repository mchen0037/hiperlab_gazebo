# Gazebo Simulation for HiPeR Lab at UC Berkeley
* ROS Kinetic
* Ubuntu 16.04
* Gazebo v 7.0.0 (should come with ROS Kinetic)

## Setup
1. Make sure ROS is installed. Tutorial can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)
2. Make sure the [lab code](https://github.com/muellerlab/GeneralCode) is properly installed.
3. Clone this repo into your catkin workspace.
```
$ cd ~/catkin_ws/src
$ git clone https://www.github.com/mchen0037/hiperlab_gazebo
```
4. Edit the ```makeEclipseProjects.sh``` so that you compile this new package. Include hiperlab_gazebo in the first line. ex:
```
catkin build hiperlab_rostools hiperlab_hardware hiperlab_components hiperlab_common hiperlab_gazebo --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -D_ECLIPSE_VERSION=4.6 -DCMAKE_CXX_COMPILER_ARG1=-std=c++11
```
5. Build the package. (Note: For some reason, I've had to run this twice, because something in rostools compiles before a custom message does, and it is dependent on that custom message.)
```
$ sh makeEclipseProjects.sh
```
6. Source the new packages *OR* add these to your ```.bashrc```
```
$ source ~/catkin_ws/devel/setup.bash
$ export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/catkin_ws/devel/lib
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/hiperlab_gazebo/models
$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$~/catkin_ws/devel/lib
```
7. Checkout ```mighty/gazebosim``` branch in [GeneralCode](https://github.com/muellerlab/GeneralCode/tree/mighty/gazebosim).
```
$ git checkout mighty/gazebosim
```

*Note: This works as of 8/6/18 with the most recent push to master (11f6501). If this doesn't work in the future, the only change I made to 11f6501 is including the physical properties of the Iris Model into a custom quadcopter type in ```Components/Components/Logic/QuadcopterConstants.hpp``` under case 37.*


## Running the Simulator
Run the launch file to Gazebo environment with Iris Model.
```
$ roslaunch hiperlab_gazebo iris.launch
```
Run the rates controller.
```
$ rosrun hiperlab_rostools quad_mocap_rates_controller 37
```
Run joystick input.
```
$ rosrun hiperlab_hardware keyboard
```

## Writing Gazebo Plugins
Make sure you install Gazebo dev.
```
$ sudo apt install libgazebo7-dev
```
### Plugins Description
There are 6 different types of Gazebo plugins: World, Model, Sensor, System, Visual, GUI. This simulator primarily uses Model, but I can see usage of Sensor in the future. Depending on the type of plugin, you will inherit from a parent class in the Gazebo C++ library.

You can specify which ones to run by including them in .sdf files (or .urdf). For example, [iris.sdf](https://github.com/mchen0037/hiperlab_gazebo/blob/master/models/iris/iris.sdf) has 7 plugins running when it is loaded into gazebo: 
* libgazebo_ros_interface
* libgazebo_multirotor_base_plugin
* libgazebo_motor_model.so (4 instances running on each rotor)
* libgazebo_imu_plugin.so

For a more in depth tutorial about writing Plugins, see [here](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5).
The [API](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/index.html) was pretty helpful too.

### Transporting Data between Gazebo Plugins
<center>
<img src="https://github.com/mchen0037/hiperlab_gazebo/blob/master/doc/Gazebo%20Block%20Diagram.png">
</center>

Transporting data between Gazebo Plugins is just like any publish-subscribe system (ROS). In this case, the plugin __ros_gazebo_interface__ wants to publish motor speed data to the second plugin, __gazebo_motor_model__. It does so by compacting a message of a certain data type and then sending it out to a topic (motor speeds pub (GZ), which corresponds to [cmd_motor_speeds](https://github.com/mchen0037/hiperlab_gazebo/blob/master/src/gazebo_ros_interface.cpp)

### Protobuf
Data types are stored as Protobuf messages. The best analogy I can think of is ROS Custom Messages. A list of built-in messages are at ```/usr/include/gazebo-7/gazebo/msgs/proto```.

You can also create custom messages, which can be found in the [/msgs](https://github.com/mchen0037/hiperlab_gazebo/tree/master/msgs) folder. To compile these messages, you need to specify to compile them in the [CMakeLists.txt](https://github.com/mchen0037/hiperlab_gazebo/blob/master/CMakeLists.txt). To be honest, I have no idea how it was able to comile, but it did, so just extend the CMakeLists.txt for any new plugins (I mostly just copy-pasted from [PX4/sitl_gazebo](https://github.com/PX4/sitl_gazebo/blob/c1ca87e37b831cd4b64ac642957e701875738909/CMakeLists.txt).

You can use ```msg.DebugString()``` to print out the values of the message, and ```vector_msg.x()``` to access the elements of the message. 

## Transporting Data to/from ROS
Gazebo plugins can also be registered as ROS Nodes, but I handled all of the ROS-ness inside of [ros_gazebo_interface.cpp](https://github.com/mchen0037/hiperlab_gazebo/blob/master/src/gazebo_ros_interface.cpp). There's nothing too new about this--describe the subscription options a corresponding callback function, and publish the same way as any other ROS node.

Plotting it in rqt_graph should give you something like this:
FIXME: <img src="https://github.com/mchen0037/hiperlab_gazebo/blob/master/doc/Gazebo%20rqt_graph.png">

## Using New Models
You're able to convert Solidworks files into an .urdf package. See Nathan for more details.

I recommend converting the model into a .sdf file before anything--it saves you some xml tags and looks a little cleaner. The way I did this:
1. Place model package into catkin_ws and compile it. Models should come as ROS packages, with a package.xml and CMakeLists.txt.
2. Load Gazebo
```
$ roslaunch gazebo_ros empty_world.launch
```
3. Load URDF model into the GUI.
```
$ rosrun gazebo_ros spawn_model -file (path to file).urdf -urdf -model my_object
```
4. Right click the model in the GUI to open the Model Editor (Edit model).
5. Ctrl-X will close the Model Editor and prompt and ask if you want to save.
6. Save the model, which will be an SDF file.

I placed the sdf file in [/models](https://github.com/mchen0037/hiperlab_gazebo/tree/master/models), and other meshes in [/rotors_description](https://github.com/mchen0037/hiperlab_gazebo/tree/master/models/rotors_description). As long as you correctly identify the path to where bits and pieces of the model are in your sdf file, you should be okay.

For example, the rotors point to a [.dae](https://github.com/mchen0037/hiperlab_gazebo/tree/master/models/rotors_description/meshes) file inside of models/rotors_description/meshes, which is specified in line 124, ```<uri>model://rotors_description/meshes/iris_prop_ccw.dae</uri>``` of [iris.sdf](https://github.com/mchen0037/hiperlab_gazebo/blob/master/models/iris/iris.sdf).

model:// is defined by the line that we export earlier in the setup.
```
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/hiperlab_gazebo/models
```

## TODO List
* Fix dynamics of Crazyflie
* Handle different flight controller types (only works with quad_mocap_rates_controller).
* Include [Contact Sensor](http://gazebosim.org/tutorials?tut=contact_sensor), write a corresponding contact plugin
* Include [Camera](gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros) on Models, write a corresponding camera plugin
* Find/Create different worlds to simulate vehicle in, rather than empty_world.
