#!/usr/bin/env python

import rospy
import os
from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnModel
import subprocess
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Quaternion

def spawn_robot_in_gazebo(xacro_file_path, robot_name, robot_namespace='', pose=None):
    rospy.init_node('spawn_robot_in_gazebo', anonymous=True)

    # 1. Convert the Xacro to URDF
    # This converts the Xacro file into a URDF string
    try:
        urdf_xml = subprocess.check_output(['rosrun', 'xacro', 'xacro', xacro_file_path])
        urdf_xml = urdf_xml.decode('utf-8')  # Decode bytes to string
    except subprocess.CalledProcessError as e:
        rospy.logerr("Error converting Xacro to URDF: %s", e)
        return

    # 2. Set the default pose if not provided (position (0,0,0) and orientation (0,0,0,1))
    if pose is None:
        pose = Pose()
        pose.position.x = 1.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

    # 3. Wait for the spawn model service to be available
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    
    try:
        # 4. Create the service proxy for spawning the model
        spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        
        # 5. Call the spawn_model service
        response = spawn_model_service(
            robot_name,                # The name of the robot in Gazebo
            urdf_xml,                  # The URDF of the robot
            '',                        # Optional robot namespace
            pose,                      # The pose to spawn the robot at
            robot_namespace            # Robot namespace (empty or custom)
        )
        rospy.loginfo("Robot spawned successfully.")
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    # Path to the Xacro file
    xacro_file = "/home/vboxuser/cpp_ros/src/mobot_arm/src/xacro/mobot_arm.xacro"  # Update the path accordingly
    
    # Robot name to use in Gazebo
    robot_name = "my_robot"  # Choose a name for the robot in Gazebo
    
    # Call the function to spawn the robot with pose set to (0, 0, 0, 0, 0, 0, 1)
    spawn_robot_in_gazebo(xacro_file, robot_name)


