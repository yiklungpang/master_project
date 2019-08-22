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

""" Recording 2D and 3D visuals of tools and objects
This service places each tool and object in the middle of the work
table in 4 different poses and records the scene with the 
Kinect sensor.
The RGB is retrieved, published to image_saver and saved as jpg.
The point cloud data is retrieved, published to point_cloud_to_pcd
nodes and saved as PCD.
"""

import copy
import json
import moveit_commander
import moveit_msgs.msg
import rospy
import rospkg
import sys
import tf
from affordance_experiment.srv import *
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from geometry_msgs.msg import *
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import String
from math import pi


def main(req):
    """Main function for the record visuals service

    Parameters
    ----------
    req : list of str
        The combined list of tools and objects names

    """
    rospy.loginfo('-------------------------------------------------')
    rospy.loginfo('Recording visual features of tools and objects')
    rospy.loginfo('-------------------------------------------------')

    # Getting the list of tools and objects to record
    object_list = req.object_list

    # Start recording
    for object_name in object_list:
        # Record 2D visuals in one pose
        record_rgb(object_name)
        # Record 3D visuals in 4 different poses
        record_pc(object_name, 'front', 0.0, 0.35, 1.05, pi/2.0, pi/2.0, 0.0)
        record_pc(object_name, 'left',  0.0, 0.35, 1.05, pi/2.0, pi/2.0, pi/2.0)
        record_pc(object_name, 'back',  0.0, 0.35, 1.05, pi/2.0, pi/2.0, pi+0.0000001) # for some reason pi is not converted correctly... need to add small number
        record_pc(object_name, 'right', 0.0, 0.35, 1.05, pi/2.0, pi/2.0, 3.0*pi/2.0)
    
    # Return recording status
    return RecordVisualsResponse('Finished')

def record_rgb(object_name):
    """Recording 2D visuals
    Spawn the object, record the visuals and delete the object

    Parameters
    ----------
    object_name : str
        The name of the tool or object to be recorded

    """

    # Spawn the object
    rospack = rospkg.RosPack()
    object_urdf_path = str(rospack.get_path('affordance_experiment')+'/models/'+object_name+'.urdf')
    pitch = pi/2
    yaw = 0
    if object_name == 'cylinder':
        yaw = pi/2
    spawn_object(object_name, object_urdf_path, 0.0, 0.35, 1.05, pi/2, pitch, yaw)

    # Wait for the object to rest
    rospy.sleep(2)

    # Get the RGB image from the Kinect sensor
    # Note: There are issues with wait_for_message not getting the
    # most up to date message from /camera/rgb/image_raw.
    # This is a hack to wait for the updated message
    current_time = rospy.get_rostime()
    orig_rgb = rospy.wait_for_message('/camera/rgb/image_raw', Image)
    while orig_rgb.header.stamp < current_time:
        orig_rgb = rospy.wait_for_message('/camera/rgb/image_raw', Image)

    # Copy the recevied Image message to a new Image message
    copy_rgb = Image()
    copy_rgb.header = orig_rgb.header
    copy_rgb.height = orig_rgb.height
    copy_rgb.width = orig_rgb.width
    copy_rgb.encoding = orig_rgb.encoding
    copy_rgb.is_bigendian = orig_rgb.is_bigendian
    copy_rgb.step = orig_rgb.step
    copy_rgb.data = orig_rgb.data

    # Publish the new Image to be saved with the specific name
    rgb_pub = rospy.Publisher(object_name+'_rgb', Image, queue_size=1)
    rospy.loginfo('-------------------------------------------------')
    rospy.loginfo('Generating RGB')
    rospy.loginfo('-------------------------------------------------')
    rospy.sleep(3)
    rgb_pub.publish(copy_rgb)

    # Delete object
    delete_object(object_name)
    rospy.sleep(3)

def record_pc(object_name, mode, pos_x, pos_y, pos_z, row, pitch, yaw):
    """Recording 3D visuals
    Spawn the object, record the visuals and delete the object

    Parameters
    ----------
    object_name : str
        The name of the tool or object to be recorded
    mode : str
        Label of the pose of the object
    pos_x : float
        X position for the object to spawn in
    pos_y : float
        Y position for the object to spawn in
    pos_z : float
        Z position for the object to spawn in
    row : float
        Row pose for the object to spawn in
    pitch : float
        Pitch pose for the object to spawn in
    yaw : float
        Yaw pose for the object to spawn in

    """

    # Spawn the object
    rospack = rospkg.RosPack()
    object_urdf_path = str(rospack.get_path('affordance_experiment')+'/models/'+object_name+'.urdf')
    # Change pose for specific objects
    # if object_name == 'cylinder':
    #     pitch = 0.0
    
    spawn_object(object_name, object_urdf_path, pos_x, pos_y, pos_z, row, pitch, yaw)
    
    # Wait for the object to rest
    rospy.sleep(2)

    # Copy the recevied PointCloud2 message to a new PointCloud2 message
    orig_pc = rospy.wait_for_message('/camera/depth/points', PointCloud2)
    copy_pc = PointCloud2()
    copy_pc.header = orig_pc.header
    copy_pc.height = orig_pc.height
    copy_pc.width = orig_pc.width
    copy_pc.fields = orig_pc.fields
    copy_pc.is_bigendian = orig_pc.is_bigendian
    copy_pc.point_step = orig_pc.point_step
    copy_pc.row_step = orig_pc.row_step
    copy_pc.data = orig_pc.data
    copy_pc.is_dense = orig_pc.is_dense

    # Publish the new PointCloud2 to be saved with the specific name
    pc_pub = rospy.Publisher(object_name+'_'+mode, PointCloud2, queue_size=10)
    rospy.loginfo('-------------------------------------------------')
    rospy.loginfo('Generating PCD')
    rospy.loginfo('-------------------------------------------------')
    rospy.sleep(3)
    pc_pub.publish(copy_pc)

    # Delete object
    delete_object(object_name)
    rospy.sleep(3)

def record_visuals_server():
    """Set up the server for the record visual service

    """

    # Initiate record visual node
    rospy.init_node('record_visuals_server', anonymous=True)
    s = rospy.Service('record_visuals', RecordVisuals, main)

    # keep node running
    rospy.spin()


def spawn_object(model_name, model_path, spawn_x, spawn_y, spawn_z, row, pitch, yaw):
    """Spawn the object in the specified position and pose

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
    row : float
        Row pose for the object to spawn in
    pitch : float
        Pitch pose for the object to spawn in
    yaw : float
        Yaw pose for the object to spawn in

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
        quaternion = tf.transformations.quaternion_from_euler(row, pitch, yaw)
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


## Main function
if __name__ == '__main__':
    try:
        record_visuals_server()
    except rospy.ROSInterruptException:
        pass
