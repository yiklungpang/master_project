#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Yik Lung Pang

""" Autonomous Exploration of Affordances
This service takes a list of tools and objects as input and perform
autonomous exploration of affordances by carrying out the pre-defined
actions and recording the effect.
The experimental results are saved as JSON files.
"""

import copy
import json
import moveit_commander
import moveit_msgs.msg
import os
import random
import rospy
import rospkg
import sys
import tf
from affordance_experiment.srv import *
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from geometry_msgs.msg import *
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from math import pi


def main(req):
    """Main function to perform the experiment and record the effects

    Parameters
    ----------
    req : list of str
        The list of tools and objects names

    """
    rospy.loginfo('-------------------------------------------------')
    rospy.loginfo('Starting affordance experiment')
    rospy.loginfo('-------------------------------------------------')

    # Initialise moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Interface to the group of joints belonging to the left UR5 arm
    group_name = 'left_arm'
    group = moveit_commander.MoveGroupCommander(group_name)

    # Position tool parallel to the table
    joint_goal = group.get_current_joint_values()
    joint_goal[5] = pi/3.8
    group.go(joint_goal, wait=True)
    group.stop()

    # Get list of objects to experiment on
    object_list = req.object_list

    # Initialise dictionary for storing experiment results
    experiment_data = {}
    experiment_data['objects'] = {}
    experiment_data['tools'] = req.tool_name

    # Get absolute path to package
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('affordance_experiment')

    # Perform affordance experiments for each object
    for object_name in object_list:

        # Path to object urdf file
        object_urdf_path = str(pkg_path+'/models/'+object_name+'.urdf')

        # Initialise dictionary for storing experiment results
        experiment_data['objects'][object_name] = {}
        experiment_data['objects'][object_name]['results'] = []

        # Number of times to repeat each experiment
        experiments_remaining = req.repeat_no
        while experiments_remaining > 0:
            experiments_remaining -= 1

            # Perform push experiment
            result = {}
            result['action'] = 'push'
            reset(group)
            spawn_object(object_name, object_urdf_path, 0.0, 0.35, 1.1)
            init_push(group)
            # Record start position
            result['start_pos'] = get_model_pose(object_name)
            # Perform action and wait so the effect is completed
            push(group)
            rospy.sleep(4)
            # Record end position
            result['end_pos'] = get_model_pose(object_name)
            # Delete object and save results
            delete_object(object_name)
            experiment_data['objects'][object_name]['results'].append(result)
            del result
            

            # Perform tap from left experiment
            result = {}
            result['action'] = 'tap_from_left'
            reset(group)
            spawn_object(object_name, object_urdf_path, 0.0, 0.35, 1.1)
            init_tap_from_left(group)
            # Record start position
            result['start_pos'] = get_model_pose(object_name)
            # Perform action and wait so the effect is completed
            tap_from_left(group)
            rospy.sleep(4)
            # Record end position
            result['end_pos'] = get_model_pose(object_name)
            # Delete object and save results
            delete_object(object_name)
            experiment_data['objects'][object_name]['results'].append(result)
            del result

            # Perform tap from right experiment
            result = {}
            result['action'] = 'tap_from_right'
            reset(group)
            spawn_object(object_name, object_urdf_path, 0.0, 0.35, 1.1)
            init_tap_from_right(group)
            # Record start position
            result['start_pos'] = get_model_pose(object_name)
            # Perform action and wait so the effect is completed
            tap_from_right(group)
            rospy.sleep(4)
            # Record end position
            result['end_pos'] = get_model_pose(object_name)
            # Delete object and save results
            delete_object(object_name)
            experiment_data['objects'][object_name]['results'].append(result)
            del result

            # Perform pull experiment
            result = {}
            result['action'] = 'pull'
            reset(group)
            spawn_object(object_name, object_urdf_path, 0.0, 0.35, 1.1)
            init_pull(group, req.tool_name)
            # Record start position
            result['start_pos'] = get_model_pose(object_name)
            # Perform action and wait so the effect is completed
            pull(group)
            rospy.sleep(4)
            # Record end position
            result['end_pos'] = get_model_pose(object_name)
            # Delete object and save results
            delete_object(object_name)
            experiment_data['objects'][object_name]['results'].append(result)
            del result

    # Write experiment data to json
    fname = req.tool_name+'_data.json'
    file_path = str(pkg_path+'/bn/data/'+fname)
    with open(file_path, 'w') as outfile:
        json.dump(experiment_data, outfile, ensure_ascii=False, indent=4)

    # Return experiment status
    return PerformExperimentResponse('Finished')

def perform_experiment_server():
    """Set up the server for the record visual service

    """

    # Initiate this node
    rospy.init_node('perform_experiment_server', anonymous=True)
    s = rospy.Service('perform_experiment', PerformExperiment, main)

    # keep node running
    rospy.spin()


def spawn_object(model_name, model_path, spawn_x, spawn_y, spawn_z):
    """Spawn the object in the specified position

    Parameters
    ----------
    model_name : str
        The name of the tool or object to be spawned
    model_path : str
        Absolute path to the object's urdf
    spawn_x : float
        X position for the object to spawn in
    spawn_y : float
        Y position for the object to spawn in
    spawn_z : float
        Z position for the object to spawn in

    """

    # Wait until Gazebo spawning service is available
    rospy.loginfo('-------------------------------------------------')
    rospy.loginfo('Spawining object')
    rospy.loginfo('-------------------------------------------------')
    print("Waiting for gazebo services...")
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    print("Got it.")

    # Call spawning service
    try:
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

        # Open urdf file
        with open(model_path, "r") as f:
            model_xml = f.read()

        # Spawn the object
        quaternion = tf.transformations.quaternion_from_euler(0,0,0)
        model_pose = Pose(Point(x=spawn_x, y=spawn_y, z=spawn_z), Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
        spawn_model(model_name, model_xml, "", model_pose, "world")
        print("Object spawned.")

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def delete_object(model_name):
    """Delete object by it's name

    Parameters
    ----------
    model_name : str
        The name of the tool or object to be deleted

    """

    # Wait until Gazebo deleting service is available
    rospy.loginfo('-------------------------------------------------')
    rospy.loginfo('Deleting object')
    rospy.loginfo('-------------------------------------------------')
    print("Waiting for gazebo services...")
    rospy.wait_for_service("/gazebo/delete_model")
    print("Got it.")

    # Attempt to delete the object until it is successful
    success = False
    while success == False:
        try:
            # Delete the object
            delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
            response = delete_model(model_name)
            success = response.success
            print("Object deleted.")
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


def reset(group):
    """Move the arm to the resting position

    Parameters
    ----------
    group : MoveGroupCommander
        MoveGroupCommander of the arm to be controlled

    """

    # Define waypoint
    startpose = group.get_current_pose().pose
    startpose.position.x = -0.5
    startpose.position.y = 0.35
    startpose.position.z = 1.025

    # Compute and execute path
    plan,_ = group.compute_cartesian_path([startpose], 0.01, 0.0)
    group.execute(plan, wait=True)
    group.stop()


def init_push(group):
    """Move the arm in preparation for push action

    Parameters
    ----------
    group : MoveGroupCommander
        MoveGroupCommander of the arm to be controlled

    """

    # Define waypoint
    startpose = group.get_current_pose().pose
    startpose.position.x = -0.3
    # startpose.position.y = 0.35 + (random.random()-0.5)/100.0
    startpose.position.y = 0.35
    startpose.position.z = 1.025

    # Compute and execute path
    plan,_ = group.compute_cartesian_path([startpose], 0.01, 0.0)
    group.execute(plan, wait=True)
    group.stop()


def init_tap_from_left(group):
    """Move the arm in preparation for tap from left action

    Parameters
    ----------
    group : MoveGroupCommander
        MoveGroupCommander of the arm to be controlled

    """

    # Define waypoint
    startpose = group.get_current_pose().pose
    # startpose.position.x = -0.15 + (random.random()-0.5)/100.0
    startpose.position.x = -0.15
    startpose.position.y = 0.45
    startpose.position.z = 1.025

    # Compute and execute path
    plan,_ = group.compute_cartesian_path([startpose], 0.01, 0.0)
    group.execute(plan, wait=True)
    group.stop()


def init_tap_from_right(group):
    """Move the arm in preparation for tap from right action

    Parameters
    ----------
    group : MoveGroupCommander
        MoveGroupCommander of the arm to be controlled

    """

    # Define waypoint
    startpose = group.get_current_pose().pose
    # startpose.position.x = -0.15 + (random.random()-0.5)/100.0
    startpose.position.x = -0.15
    startpose.position.y = 0.25
    startpose.position.z = 1.025

    # Compute and execute path
    plan,_ = group.compute_cartesian_path([startpose], 0.01, 0.0)
    group.execute(plan, wait=True)
    group.stop()


def init_pull(group, tool_name):
    """Move the arm in preparation for pull action

    Parameters
    ----------
    group : MoveGroupCommander
        MoveGroupCommander of the arm to be controlled
    tool_name : str
        Name of tool

    """

    # Define waypoint
    startpose = group.get_current_pose().pose
    startpose.position.x = -0.5
    # startpose.position.y = 0.32 + (random.random()-0.5)/100.0
    startpose.position.y = 0.32
    if tool_name == 'fork':
        # Avoid lowering the tool directly onto the object
        startpose.position.y = 0.31
    startpose.position.z = 1.1

    # Compute and execute path
    plan,_ = group.compute_cartesian_path([startpose], 0.01, 0.0)
    group.execute(plan, wait=True)
    group.stop()


def push(group):
    """Perform push action

    Parameters
    ----------
    group : MoveGroupCommander
        MoveGroupCommander of the arm to be controlled

    """

    # Define waypoints
    waypoints = []
    wpose = group.get_current_pose().pose
    # Move in +ve x direction
    wpose.position.x = -0.12
    waypoints.append(copy.deepcopy(wpose))

    # Compute and execute path
    rospy.loginfo('-------------------------------------------------')
    rospy.loginfo('Planning motion trajectory')
    rospy.loginfo('-------------------------------------------------')
    plan,_ = group.compute_cartesian_path(waypoints, 0.01, 0.0)
    rospy.loginfo('-------------------------------------------------')
    rospy.loginfo('Executing planned trajectory')
    rospy.loginfo('-------------------------------------------------')
    group.execute(plan, wait=True)
    # Ensure movement is complete
    group.stop()


def tap_from_left(group):
    """Perform tap from left action

    Parameters
    ----------
    group : MoveGroupCommander
        MoveGroupCommander of the arm to be controlled

    """

    # Define waypoints
    waypoints = []
    wpose = group.get_current_pose().pose
    # Move in -ve y direction
    wpose.position.y = 0.35
    waypoints.append(copy.deepcopy(wpose))

    # Compute and execute path
    rospy.loginfo('-------------------------------------------------')
    rospy.loginfo('Planning motion trajectory')
    rospy.loginfo('-------------------------------------------------')
    plan,_ = group.compute_cartesian_path(waypoints, 0.01, 0.0)
    rospy.loginfo('-------------------------------------------------')
    rospy.loginfo('Executing planned trajectory')
    rospy.loginfo('-------------------------------------------------')
    group.execute(plan, wait=True)
    # Ensure movement is complete
    group.stop()


def tap_from_right(group):
    """Perform tap from right action

    Parameters
    ----------
    group : MoveGroupCommander
        MoveGroupCommander of the arm to be controlled

    """

    # Define waypoints
    waypoints = []
    wpose = group.get_current_pose().pose
    # Move in +ve y direction
    wpose.position.y = 0.35
    waypoints.append(copy.deepcopy(wpose))

    # Compute and execute path
    rospy.loginfo('-------------------------------------------------')
    rospy.loginfo('Planning motion trajectory')
    rospy.loginfo('-------------------------------------------------')
    plan,_ = group.compute_cartesian_path(waypoints, 0.01, 0.0)
    rospy.loginfo('-------------------------------------------------')
    rospy.loginfo('Executing planned trajectory')
    rospy.loginfo('-------------------------------------------------')
    group.execute(plan, wait=True)
    # Ensure movement is complete
    group.stop()


def pull(group):
    """Perform pull action

    Parameters
    ----------
    group : MoveGroupCommander
        MoveGroupCommander of the arm to be controlled

    """

    # Define waypoints
    waypoints = []
    wpose = group.get_current_pose().pose
    # Move in +ve x direction, position above object
    wpose.position.x = -0.1
    waypoints.append(copy.deepcopy(wpose))
    # Move in -ve z direction, lower tool
    wpose.position.z = 1.025
    waypoints.append(copy.deepcopy(wpose))
    # Move in -ve x direction, pull tool
    wpose.position.x = -0.23
    waypoints.append(copy.deepcopy(wpose))

    # Compute and execute path
    rospy.loginfo('-------------------------------------------------')
    rospy.loginfo('Planning motion trajectory')
    rospy.loginfo('-------------------------------------------------')
    plan,_ = group.compute_cartesian_path(waypoints, 0.01, 0.0)
    rospy.loginfo('-------------------------------------------------')
    rospy.loginfo('Executing planned trajectory')
    rospy.loginfo('-------------------------------------------------')
    group.execute(plan, wait=True)
    # Ensure movement is complete
    group.stop()


def get_model_pose(object_name):
    """Get the position of an object

    Parameters
    ----------
    object_name : str
        The name of the object

    Returns
    ----------
    pose : dictionary
        dictionary containing the x, y and z position of the object
    """

    # Call service get_model_state
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    try:
        coordinates = get_model_state(object_name, 'world')
        # Return position as dictionary
        pose = {
            "x": coordinates.pose.position.x,
            "y": coordinates.pose.position.y,
            "z": coordinates.pose.position.z
        }
        return pose
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

## Main function
if __name__ == '__main__':
    try:
        perform_experiment_server()
    except rospy.ROSInterruptException:
        pass
