import rospy
import time
import tf
import numpy as np

from geometry_msgs.msg import PoseStamped, Pose, Vector3, Point, Quaternion, PoseWithCovarianceStamped 
from visualization_msgs.msg import Marker
from roboy_control_msgs.srv import GetLinkPose 
from std_msgs.msg import Header, ColorRGBA

import pdb

global LHR_74656729_pose, first
first = True
LHR_74656729_pose = PoseStamped()
global initial_pose_controller
initial_pose_controller = PoseStamped()

def _get_link_pose(link_name):
    s = rospy.ServiceProxy("/get_link_pose", GetLinkPose)
    return s(link_name).pose

def LHR_74656729_cb(msg):
	global LHR_74656729_pose, first, initial_pose_controller

	if first:
		initial_pose_controller.pose.position = msg.pose.pose.position
		initial_pose_controller.pose.orientation = msg.pose.pose.orientation
		initial_pose_controller.header.frame_id = "vive_world"
		first = False
	else:
		x = msg.pose.pose.position.x -initial_pose_controller.pose.position.x 
		y = msg.pose.pose.position.y -initial_pose_controller.pose.position.y
		z = msg.pose.pose.position.z -initial_pose_controller.pose.position.z
		LHR_74656729_pose.pose.position = Point(x,y,z)
		LHR_74656729_pose.pose.orientation = msg.pose.pose.orientation
		LHR_74656729_pose.header.frame_id = "vive_world"
	

if __name__ == "__main__":  
    rospy.init_node('tracker_transformer')
    li = tf.TransformListener()
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
    pose_sub = rospy.Subscriber('/vive/LHR_74656729_pose', PoseWithCovarianceStamped, LHR_74656729_cb)
    time.sleep(1)

    hand_left_pose = _get_link_pose("hand_left")
    

    while not rospy.is_shutdown():
        current_pose_controller_vive = LHR_74656729_pose
        current_pose_controller_torso = li.transformPose("LHR_29508350", current_pose_controller_vive) #li.lookupTransform('LHR_74656729', 'world', rospy.Time(0))
        initial_pose_controller_torso = li.transformPose("LHR_29508350", initial_pose_controller)
        current_pose_controller_torso.pose.position.x += hand_left_pose.position.x #- initial_pose_controller_torso.pose.position.x
        current_pose_controller_torso.pose.position.y += hand_left_pose.position.y #- initial_pose_controller_torso.pose.position.y
        current_pose_controller_torso.pose.position.z += hand_left_pose.position.z #- initial_pose_controller_torso.pose.position.z
      

        marker_msg = Marker()
        marker_msg.header.frame_id = "LHR_29508350"
        marker_msg.id = 102
        marker_msg.type = 1
        marker_msg.action = 0
        marker_msg.text = "tracker"
        marker_msg.scale = Vector3(0.1,0.1,0.1)
        marker_msg.color = ColorRGBA(0.0,  255.0, 0.0, 40.0)
        # import pdb; pdb.set_trace()
        marker_msg.pose = current_pose_controller_torso.pose
        marker_pub.publish(marker_msg)
