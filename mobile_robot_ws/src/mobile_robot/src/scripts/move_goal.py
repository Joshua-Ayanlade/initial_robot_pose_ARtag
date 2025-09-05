#!/usr/bin/env python
import rospy, sys, actionlib,tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from geometry_msgs.msg import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist,Point
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import quaternion_from_euler
from tf import TransformListener
from mobot_arm.srv import amclPose,amclPoseRequest

global tag_id

class Move_Base:
    def __init__(self):
        self.initialPosePub = rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=5)
        self.amclPosePub = rospy.Publisher('/amcl_pose',PoseWithCovarianceStamped,queue_size=5)
        self.amclPoseSub = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.amclPoseCb)   
        self.cmdPub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.arMarkerSub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers,self.arMarkerCb)
        self.arPose = Point()
        self.amclPose = Point()
        self.speed = Twist()
        self.stop = False
        self.tagMove = ""
        self.initPose = False


    def initialPose(self,trans_x,trans_y,trans_z,rot_x,rot_y,rot_z,rot_w):
        initialPose = PoseWithCovarianceStamped()
        r,p,y = tf.transformations.euler_from_quaternion([rot_x, rot_y,rot_z,rot_w])
        orient_tag3 = Quaternion(*quaternion_from_euler(0,0,-1.57+y))
        orient_tag4 = Quaternion(*quaternion_from_euler(0,0,-1.57-0.436685-y))
        #initialPose.pose.pose.position.x = -trans_x
        if tag_id == "1":
            initialPose.pose.pose.position.x = 2.201060
            initialPose.pose.pose.position.y = -0.043282
            initialPose.pose.pose.position.z = 0.0
            initialPose.pose.pose.orientation.x = 0
            initialPose.pose.pose.orientation.y = 0
            initialPose.pose.pose.orientation.z = -rot_z
            initialPose.pose.pose.orientation.w = rot_w 

        if tag_id == "3":
            initialPose.pose.pose.position.x = 2.554712
            initialPose.pose.pose.position.y = 2.337136
            initialPose.pose.pose.position.z = 0.0
            initialPose.pose.pose.orientation.x = 0
            initialPose.pose.pose.orientation.y = 0
            initialPose.pose.pose.orientation.z = -orient_tag3.z
            initialPose.pose.pose.orientation.w = orient_tag3.w 

        if tag_id == "4":
            initialPose.pose.pose.position.x = -0.210896
            initialPose.pose.pose.position.y = -1.542397
            initialPose.pose.pose.position.z = 0.0
            initialPose.pose.pose.orientation.x = 0
            initialPose.pose.pose.orientation.y = 0
            initialPose.pose.pose.orientation.z = -orient_tag4.z
            initialPose.pose.pose.orientation.w = -orient_tag4.w 

        initialPose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

        self.initialPosePub.publish(initialPose)
        self.amclPosePub.publish(initialPose)

    def move_base_goal(self):
        move_base = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        move_base.wait_for_server()

        goalPose = MoveBaseGoal()
        goalPose.target_pose.header.frame_id='map'
        orient = Quaternion(*quaternion_from_euler(0,0,0))

        goalPose.target_pose.pose.position.x = 4
        goalPose.target_pose.pose.orientation = orient
        move_base.send_goal(goalPose)
        move_base.wait_for_result()
    
    def arMarkerCb(self,msg):
        if msg.markers[0].pose.pose.position.x<2.0:
            self.stop = True

        elif msg.markers[0].id==1 or msg.markers[0].id == 3 or msg.markers[0].id == 4 and msg.markers[0].pose.pose.position.x>=2.0:
            self.arPose = msg.markers[0].pose.pose.position
            print("Tag id {0} Detected" .format(msg.markers[0].id))
            print("Robot is {0} m from Tag {1}".format(msg.markers[0].pose.pose.position.x,msg.markers[0].id))
            if self.tagMove == "True":
                self.tag_move()
                #print(msg.markers[0].pose.pose.position.x)
        
        else:
            self.arPose = Point()
            #print("tag not detected")
            #print(msg.markers[0].id)
    
    def tag_move(self):
        self.speed.linear.x = 0.2
        self.speed.angular.z = self.arPose.y / 2

        #print(self.speed)
        print("Navigating towards detected tag....")
        
        self.cmdPub.publish(self.speed)

    def amclPoseCb(self,msg):
        self.amclPos_x = msg.pose.pose.position.x


class Transform:
    def __init__(self):
        self.tf_x,self.tf_y,self.tf_z = 0,0,0
        self.tf_x_rot,self.tf_y_rot,self.tf_z_rot,self.tf_w_rot = 0,0,0,0

    def lookup_tf(self,tf_listener,item1,item2):
        try:
            (trans,rot) = tf_listener.lookupTransform(item1,item2,rospy.Time(0))
            return trans,rot
        except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
            rospy.logerr("TF exception")
            return None
    
    def detect_tag(self,tag_id):
        tf_listener = TransformListener()
        move_base = Move_Base()
        rospy.sleep(2)
        if tag_id == "1":
            tf_values = self.lookup_tf(tf_listener,'base_link','marker_1_rot')
            print(tf_values)
            detectTag = True
        if tag_id == "3":
            tf_values = self.lookup_tf(tf_listener,'base_link','marker_3_rot')
            print(tf_values)
            detectTag = True
        if tag_id == "4":
            tf_values = self.lookup_tf(tf_listener,'base_link','marker_4_rot')
            #tf_values = self.lookup_tf(tf_listener,'base_link','map')
            print(tf_values)
            detectTag = True
        while tf_values is not None and detectTag:
            self.tf_x,self.tf_y,self.tf_z = tf_values[0][0],tf_values[0][1],tf_values[0][2]
            self.tf_x_rot,self.tf_y_rot,self.tf_z_rot,self.tf_w_rot = tf_values[1][0],tf_values[1][1],tf_values[1][2],tf_values[1][3]
            rospy.loginfo("%s tf_xyz: [%f,%f,%f]", tag_id, self.tf_x,self.tf_y,self.tf_z)
            rospy.loginfo("%s tf_xyzw: [%f,%f,%f,%f]", tag_id, self.tf_x_rot,self.tf_y_rot,self.tf_z_rot,self.tf_w_rot)
            break
        detectTag = False

if __name__ == '__main__':
    rospy.init_node('move_goal')
    amcl_srv_client = rospy.ServiceProxy('/amcl_pose_server',amclPose)
    amcl_srv_client.wait_for_service()

    move_base = Move_Base()
    move_base.tagMove = "True"
    transform = Transform()
    global amclInitPos_x
    global amclInitPos_y
    global done
    done= False

    if len(sys.argv) != 2:
        rospy.logerr("Usage: python script_name.py, Input tag for setting inital pose")
        sys.exit(1)

    if sys.argv[1]=="1":
        tag_id = "1"
    
    elif sys.argv[1]=="3":
        tag_id = "3"
    
    elif sys.argv[1]=="4":
        tag_id = "4"
    else:
        rospy.logerr("Tag 1,3 and 4 exist")
        sys.exit(1)

    while not rospy.is_shutdown():
        #transform.detect_tag('marker_1_rot')

        if move_base.stop == True:
           amclPoseReq = amclPoseRequest()
           amclPoseReq.getPose = True
           amclInitPos_x = amcl_srv_client(amclPoseReq).pos_x
           amclInitPos_y = amcl_srv_client(amclPoseReq).pos_y
           print("amclInitPos_x: ",amclInitPos_x)

           print("Reference distance from Tag reached!")
           print("Setting Robot initial Pose using Tag {0}...".format(tag_id))
           move_base.initPose = True
           move_base.tagMove = "False"
        if move_base.initPose == True:
            init = True
            while init:
                amclPoseReq = amclPoseRequest()
                amclPoseReq.getPose = True
                if amclInitPos_x != amcl_srv_client(amclPoseReq).pos_x and amclInitPos_y != amcl_srv_client(amclPoseReq).pos_y:
                    done = True
                    print("Robot successfully moved to Intial Pose")
                    break
                if tag_id == "1":
                    transform.detect_tag('1')
                    move_base.initialPose(transform.tf_x,transform.tf_y,transform.tf_z,transform.tf_x_rot,transform.tf_y_rot,transform.tf_z_rot,transform.tf_w_rot)
                if tag_id == "3":
                    transform.detect_tag('3')
                    move_base.initialPose(transform.tf_x,transform.tf_y,transform.tf_z,transform.tf_x_rot,transform.tf_y_rot,transform.tf_z_rot,transform.tf_w_rot)
                if tag_id == "4":
                    transform.detect_tag('4')
                    move_base.initialPose(transform.tf_x,transform.tf_y,transform.tf_z,transform.tf_x_rot,transform.tf_y_rot,transform.tf_z_rot,transform.tf_w_rot)
        if done == True:
            break
        rospy.sleep(1.0)
        #break
