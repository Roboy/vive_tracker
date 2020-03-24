#!/usr/bin/python3
import rospy
import time
import tf
import numpy as np
import math

from geometry_msgs.msg import PoseStamped, Pose, Vector3, Point, Quaternion, PoseWithCovarianceStamped 
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from roboy_control_msgs.srv import GetLinkPose 
from std_msgs.msg import Header, ColorRGBA, Bool
from roboy_middleware_msgs.srv import InverseKinematics, InverseKinematicsRequest
from scipy import interpolate

import pdb

rospy.init_node("roboy_publisher")
global reset
reset = False

def _get_link_pose(link_name):
	s = rospy.ServiceProxy("/get_link_pose", GetLinkPose)
	return s(link_name).pose

class Tracker:
	def __init__(self, name, endeffector, ik_srv, reference_frame="LHR_29508350", marker_pub=None):
		self.current_pose = PoseStamped()
		self.current_pose_roboy = PoseStamped()
		self.first_update = True
		self.name = name
		self.initial_pose = PoseStamped()
		self.pose_sub = rospy.Subscriber("/vive/%s_pose"%self.name, PoseWithCovarianceStamped, self.pose_cb)
		self.id = None
		self.marker_pub = marker_pub
		self.endeffector = endeffector
		self.endeffector_pose = _get_link_pose(self.endeffector)
		self.reference_frame = reference_frame
		self.ik_srv = ik_srv
		self.li = tf.TransformListener()
		time.sleep(1)
		rospy.loginfo("done init for %s"%self.name)


	def pose_cb(self, msg):
		if self.first_update:
			self.initial_pose.pose.position = msg.pose.pose.position
			self.initial_pose.pose.orientation = msg.pose.pose.orientation
			self.initial_pose.header.frame_id = "vive_world"
			self.first_update = False
		else:
			x = msg.pose.pose.position.x - self.initial_pose.pose.position.x
			y = msg.pose.pose.position.y - self.initial_pose.pose.position.y
			z = msg.pose.pose.position.z - self.initial_pose.pose.position.z

			self.current_pose.pose.position = Point(x,y,z)
			self.current_pose.pose.orientation = msg.pose.pose.orientation
			self.current_pose.header.frame_id = "vive_world"

	def to_roboy_frame(self):
		self.current_pose_roboy = self.li.transformPose(self.reference_frame, self.current_pose)

		self.current_pose_roboy.pose.position.x += self.endeffector_pose.position.x
		self.current_pose_roboy.pose.position.y += self.endeffector_pose.position.y
		self.current_pose_roboy.pose.position.z += self.endeffector_pose.position.z

		return self.li.transformPose("world", self.current_pose_roboy) # TODO: double check this

	def track_ik(self, marker=False):
		requested_pose = self.to_roboy_frame()
		ik_request = InverseKinematicsRequest(endeffector=self.endeffector,
											  target_frame=self.endeffector,
											  pose=requested_pose.pose, type=1)
		self.ik_srv(ik_request)
		if marker:
			self.publish_marker(requested_pose, self.reference_frame)

	def generate_id(self):
		import hashlib
		m = hashlib.md5()
		m.update(self.name.encode('utf-8'))
		self.id = int(str(int(m.hexdigest(), 16))[0:4])

	def publish_marker(self, pose, reference_frame):
		if self.marker_pub is None:
			rospy.logwarn_throttle(5, "Publisher not initialized, won't publish marker messages")

		else:
			if self.id is None:
				self.generate_id()

			marker_msg = Marker()
			# marker_msg.header.frame_id = "LHR_29508350"
			marker_msg.header.frame_id = "world"
			marker_msg.id = self.id
			marker_msg.type = 1
			marker_msg.action = 0
			# marker_msg.text = "tracker"
			marker_msg.scale = Vector3(0.1,0.1,0.1)
			marker_msg.color = ColorRGBA(0.0,  255.0, 0.0, 40.0)
			marker_msg.pose = pose.pose
			self.marker_pub.publish(marker_msg)


class Headset:
	def __init__(self, name, joint_pub,reference_frame="LHR_29508350"):
		self.current_pose = PoseStamped()
		self.initial_pose = PoseStamped()
		self.first_update = True
		self.name = name
		self.pose_sub = rospy.Subscriber("/vive/%s_pose"%self.name, PoseWithCovarianceStamped, self.pose_cb)
		self.joint_pub = joint_pub
		self.li = tf.TransformListener()
		self.joint_msg = JointState()
		self.joint_msg.name = ['head_axis0', 'head_axis2', 'head_axis1']
		time.sleep(1)

	def pose_cb(self,msg):
		if self.first_update:
			self.initial_pose.pose.position = msg.pose.pose.position
			self.initial_pose.pose.orientation = msg.pose.pose.orientation
			self.initial_pose.header.frame_id = "vive_world"
			self.first_update = False
		else:
			xx = msg.pose.pose.position.x - self.initial_pose.pose.position.x
			yy = msg.pose.pose.position.y - self.initial_pose.pose.position.y
			zz = msg.pose.pose.position.z - self.initial_pose.pose.position.z
			self.current_pose.pose.position = Point(xx,yy,zz)

			q = tf.transformations.quaternion_about_axis(math.pi/4, (0, 1, 0))
			(qx,qy,qz,qw) = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
			orientation = tf.transformations.quaternion_multiply(q,(qx,qy,qz,qw))
			self.current_pose.pose.orientation = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
			self.current_pose.header.frame_id = "vive_world"

	def track_joints(self):
		current_pose_headset_torso = self.li.transformPose("LHR_29508350", self.current_pose)

		(r_headset, p_headset, y_headset) = list(tf.transformations.euler_from_quaternion(
			[current_pose_headset_torso.pose.orientation.x,
			 current_pose_headset_torso.pose.orientation.y,
			 current_pose_headset_torso.pose.orientation.z,
			 current_pose_headset_torso.pose.orientation.w]))
		self.joint_msg.position = [r_headset, p_headset, -y_headset+1.57]
		self.joint_msg.velocity = np.zeros(len(self.joint_msg.name))
		self.joint_msg.effort = np.zeros(len(self.joint_msg.name))
		self.joint_pub.publish(self.joint_msg)


if __name__ == "__main__":

	# time.sleep(3)
	ik = rospy.ServiceProxy('/execute_ik', InverseKinematics)
	marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
	joints_pub = rospy.Publisher("/joint_targets", JointState, queue_size=1)

	pose = Pose(position=Point(0.226428762078, 0.00383305549622, 0.0686610788107))
	# pose.header.frame_id = "world"
	req = InverseKinematicsRequest(endeffector="hand_left",
									target_frame="hand_left",
									pose=pose, type=1)
	ik(req)

	pose = Pose(position=Point(-0.226428762078, -0.00383305549622, 0.0686610788107))
	# pose.header.frame_id = "world"
	req = InverseKinematicsRequest(endeffector="hand_right",
								   target_frame="hand_right",
								   pose=pose, type=1)
	ik(req)

	rospy.wait_for_message('/trigger',Bool)

	left_tracker = Tracker("LHR_74656729","hand_left", ik,marker_pub=marker_pub)
	right_tracker = Tracker("LHR_EFCEDA8F", "hand_right", ik, marker_pub=marker_pub)
	headset = Headset("LHR_5EA88DE7", joint_pub=joints_pub)
	while not rospy.is_shutdown():
		headset.track_joints()
		left_tracker.track_ik(marker=True)
		right_tracker.track_ik(marker=True)

	'''

		qx = current_pose_controller_vive.pose.orientation.x
		qy = current_pose_controller_vive.pose.orientation.y
		qz = current_pose_controller_vive.pose.orientation.z
		qw = current_pose_controller_vive.pose.orientation.w
		(r, p, y) = tf.transformations.euler_from_quaternion([qx,qy,qz,qw])
		joint_msg.position = [-r-1.57] #p, -y-1.57,
		joints_pub.publish(joint_msg)
		
	'''