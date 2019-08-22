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

""" Data Collection Overview
This node controls the process of data collection which consists of
two steps.
1. recordVisuals - record the 2D and 3D images of the tools and
                   objects
2. performExperiment - perform the action on the objects with the
                       tool and record the outcome
A new session of Gazebo and rViz is launched for each step and for
each tool in the performExperiment step.
"""

import rospy
import roslaunch
import rospkg
from affordance_experiment.srv import PerformExperiment, RecordVisuals
from std_msgs.msg import String

def data_collection():
    """Collect visual and experiment data for the affordance experiment

    """

    # Initiate node for data collection
    rospy.init_node('data_collection', anonymous=True)
    
    # Define list of tools and objects
    tool_list = ['stick', 'l_stick', 'bone', 'umbrella', 'fork']
    object_list = ['cube', 'sphere', 'cylinder']
    combined_list = tool_list + object_list

    # Record 2D and 3D images of tools and objects
    recordVisuals(combined_list)

    # Perform experiment for each tool on all objects
    for tool_name in tool_list:
        performExperiment(tool_name, object_list, 10)

def recordVisuals(combined_list):
    """Record 2D and 3D images of tools and objects

    Parameters
    ----------
    combined_list : list of str
        The combined list of tools and objects names

    """

    # Launch new session of Gazebo and rViz
    # Launch record_visuals node and point_cloud_to_pcd nodes
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    rospack = rospkg.RosPack()
    cli_args1 = [rospack.get_path('arq_gazebo')+'/launch/record_visuals.launch']
    roslaunch_file1 = [(roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0])]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file1)
    parent.start()

    # Wait to make sure simulation is loaded completely
    rospy.sleep(10)

    # Start recording
    record_visuals = rospy.ServiceProxy('/record_visuals', RecordVisuals)
    try:
        record_visuals(combined_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

    # Kill the simulation and other nodes
    parent.shutdown()

def performExperiment(tool_name, object_list, repeat_no):
    """Perform affordance experiment with a tool on objects

    Parameters
    ----------
    combined_list : list of str
        The combined list of tools and objects names

    """

    # Launch new session of Gazebo and rViz
    # Launch record_visuals node and point_cloud_to_pcd nodes
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)   

    rospack = rospkg.RosPack()
    cli_args1 = [rospack.get_path('arq_gazebo')+'/launch/maria_simulation_'+tool_name+'.launch']
    roslaunch_file1 = [(roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0])]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file1)    
    parent.start()

    # Wait to make sure simulation is loaded completely
    rospy.sleep(10)

    # Start experiment
    perform_experiment = rospy.ServiceProxy('/perform_experiment', PerformExperiment)
    try:
        perform_experiment(tool_name, object_list, repeat_no)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    # Kill the simulation and other nodes
    parent.shutdown()


## Main function
if __name__ == '__main__':
    try:
        data_collection()
    except rospy.ROSInterruptException:
        pass
