#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from copy import deepcopy
from tf.transformations import quaternion_from_euler

cube_sdf_model = """<?xml version="1.0">
<sdf version = "1.4">
    <model name="MODELNAME">
        <static>true</static>
        <link name="camera_depth_frame">
        <pose>0.12 0 0.65 -1.5708 0 -1.5708</pose>
        </link>
        <link name="map"/>

        <link name="camera_link">
            <pose>0 0 0 0 0 0</pose>
        
            <visual name="camera_visual">
                <pose>0 0 0 0 0 0</pose>
                <geometry>
                <box>
                    <size>SIZEXYZ</size>
                </box>
                </geometry>
                <material>
                <ambient>0 0 0 1.0</ambient>
                <diffuse>0 0 0 1.0</diffuse>
                <specular>0.0 0.0 0.0 1.0</specular>
                <emissive>0.0 0.0 0.0 1.0</emissive>
                </material>
            </visual>    
        
            <sensor name="depth_camera" type="camera">
                <always_on>true</always_on>
                <visualize>true</visualize>
                <update_rate>5</update_rate>
                <camera name="camera">
                    <horizontal_fov>1.02974</horizontal_fov>
                    <image>
                        <width>640</width>
                        <height>480</height>
                        <format>R8G8B8</format>
                    </image>
                    <clip>
                        <near>0.02</near>
                        <far>10</far>
                    </clip>
                    <noise>
                        <type>gaussian</type>
                        <!-- Noise is sampled independently per pixel on each frame.
                            That pixel's noise value is added to each of its color
                            channels, which at that point lie in the range [0,1]. -->
                        <mean>0.0</mean>
                        <stddev>0.007</stddev>
                    </noise>
                </camera>
                <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
                    <alwaysOn>true</alwaysOn>
                    <updateRate>10.0</updateRate>
                    <camera_name>depth_camera</camera_name>
                    <frame_name>camera_depth_frame</frame_name>

                    <imageTopicName>tag/rgb/image_raw</imageTopicName>
                    <depthImageTopicName>tag/depth/image_raw</depthImageTopicName>
                    <pointCloudTopicName>tag/depth/points</pointCloudTopicName>
                    <cameraInfoTopicName>tag/rgb/camera_info</cameraInfoTopicName>              
                    <depthImageCameraInfoTopicName>tag/depth/camera_info</depthImageCameraInfoTopicName> 

                    <hack_baseline>0</hack_baseline>
                    <min_depth>0.001</min_depth>                         
                </plugin>
            </sensor>
        </link>
    </model>
</sdf>
"""

def create_cube_request(sdf_model,modelname,color,px,py,pz,rr,rp,ry,sx,sy,sz):
    """Create a SpawnModelRequest with the parameters of the cube given.
    modelname: name of the model for gazebo
    px py pz: position of the cube (and it's collision cube)
    rr rp ry: rotation (roll, pitch, yaw) of the model
    sx sy sz: size of the cube"""
    cube = deepcopy(sdf_model)
    # Replace size of model
    size_str = str(round(sx, 3)) + " " + \
        str(round(sy, 3)) + " " + str(round(sz, 3))
    cube = cube.replace('SIZEXYZ', size_str)
    
    # Replace modelname
    cube = cube.replace('MODELNAME', str(modelname))
    cube = cube.replace('COLOR', str(color))

    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = cube
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req


if __name__ == '__main__':
    #Initialize the spawn_box_models node
    rospy.init_node('spawn_box_models')
    
    #https://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams
    
    #Create a coallable object or serivce client for the service /gazebo/spawn_sdf_model
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
    #Wait for the service to get available
    spawn_srv.wait_for_service()
    rospy.loginfo("Connected to service!")

    
    # Spawn Box
    rospy.loginfo("Spawning package$1")

    #Create a request to create cube
    request= create_cube_request(cube_sdf_model, "package$1","Green",
                              -0.9, 4.2, 0.7, 0, 0.4, -1.867,  # rotation
                              0.015, 0.08, 0.022)  # size
    
    #Call the service to spawn the box
    spawn_srv.call(request)
    #spawn_srv.call(request2)
    #spawn_srv.call(request3)
    #spawn_srv.call(request4)

    rospy.sleep(1.0)
