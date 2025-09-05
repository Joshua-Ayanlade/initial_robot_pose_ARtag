#! /usr/bin/env python

import rospy
import geometry_msgs
from mobot_arm.srv import amclPose,amclPoseResponse
from geometry_msgs.msg import PoseWithCovarianceStamped,Point

class AmclPose(object):
    def __init__(self):
        self.server = rospy.Service('/amcl_pose_server',amclPose,self.amclSrvCb)
        self.amclPoseSub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amclPoseCb)
        self.amcl_position = Point()
        self.response = amclPoseResponse()

    def amclPoseCb(self,amcl_msg):
        self.amcl_position.x = amcl_msg.pose.pose.position.x
        self.amcl_position.y = amcl_msg.pose.pose.position.y
    
    def amclSrvCb(self,srv_msg):
        if srv_msg:
            self.response.pos_x = self.amcl_position.x
            self.response.pos_y = self.amcl_position.y
            #print(self.response.pos_x)
        
        return self.response  

if __name__ == '__main__':
    amclPose = AmclPose()
    rospy.init_node('amcl_pose',anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")