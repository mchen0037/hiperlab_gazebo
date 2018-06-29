#!/usr/bin/env python
import rospy
import os
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

print('hello world')
rospy.init_node('insert_object',log_level=rospy.INFO)

initial_pose = Pose()
initial_pose.position.x = 1
initial_pose.position.y = 1
initial_pose.position.z = 0

f = open(os.getenv("HOME") + '/catkin_ws/src/hiperlab_gazebo/models/iris/iris.sdf','r')
sdff = f.read()

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
spawn_model_prox("iris", sdff, "iris", initial_pose, "world")
