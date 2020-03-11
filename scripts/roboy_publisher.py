import rospy
import time
import tf
import numpy as np
import math

from geometry_msgs.msg import PoseStamped, Pose, Vector3, Point, Quaternion, PoseWithCovarianceStamped 
from visualization_msgs.msg import Marker
from roboy_control_msgs.srv import GetLinkPose 
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import JointState
from roboy_middleware_msgs.srv import InverseKinematics, InverseKinematicsRequest
from scipy import interpolate

import pdb

global LHR_74656729_pose, first_LHR_74656729, LHR_5EA88DE7_pose, first_LHR_5EA88DE7,r_init, p_init, y_init, r_cur, y_cur, p_cur
head_axes = ['head_axis0', 'head_axis1', 'head_axis2']
first_LHR_74656729 = True
first_LHR_5EA88DE7 = True
r_init =0
p_init = 0
y_init = 0
r_cur = 0
p_cur = 0
y_cur = 0
LHR_74656729_pose = PoseStamped()
LHR_5EA88DE7_pose = PoseStamped()
global initial_pose_controller, initial_pose_headset
initial_pose_controller = PoseStamped()
initial_pose_headset = PoseStamped()

def _get_link_pose(link_name):
	s = rospy.ServiceProxy("/get_link_pose", GetLinkPose)
	return s(link_name).pose

def interpolate(data):
	x = range(len(data))
	y = np.array(data)
	f = interpolate.interp1d(x, y)
	xnew = np.arange(x[0], x[-1], 0.1)
	return f(xnew)


def running_mean(data):
	sum_x = 0
	sum_y = 0
	sum_z = 0

	for d in data:
		sum_x += d.x
		sum_y += d.y
		sum_z += d.z

	return Point(sum_x/len(data), sum_y/len(data), sum_z/len(data))

def LHR_74656729_cb(msg):
	global LHR_74656729_pose, first_LHR_74656729, initial_pose_controller

	if first_LHR_74656729:
		initial_pose_controller.pose.position = msg.pose.pose.position
		initial_pose_controller.pose.orientation = msg.pose.pose.orientation
		initial_pose_controller.header.frame_id = "vive_world"
		first_LHR_74656729 = False
	else:
		x = msg.pose.pose.position.x - initial_pose_controller.pose.position.x
		y = msg.pose.pose.position.y - initial_pose_controller.pose.position.y
		z = msg.pose.pose.position.z - initial_pose_controller.pose.position.z

		# rx = msg.pose.pose.orientation.x -initial_pose_controller.pose.orientation.x 
		# ry = msg.pose.pose.orientation.y -initial_pose_controller.pose.orientation.y
		# rz = msg.pose.pose.orientation.z -initial_pose_controller.pose.orientation.z
		# w = msg.pose.pose.orientation.w -initial_pose_controller.pose.orientation.w

		LHR_74656729_pose.pose.position = Point(x,y,z)
		# LHR_74656729_pose.pose.orientation = Quaternion(rx, ry, rz, w)
		LHR_74656729_pose.pose.orientation = msg.pose.pose.orientation
		LHR_74656729_pose.header.frame_id = "vive_world"

def LHR_5EA88DE7_cb(msg1):
	global LHR_5EA88DE7_pose, first_LHR_5EA88DE7, initial_pose_headset, r_init, p_init, y_init

	if first_LHR_5EA88DE7:
		initial_pose_headset.pose.position = msg1.pose.pose.position
		initial_pose_headset.pose.orientation = msg1.pose.pose.orientation
		(r_init, p_init, y_init) = tf.transformations.euler_from_quaternion([msg1.pose.pose.orientation.x, msg1.pose.pose.orientation.y, msg1.pose.pose.orientation.z, msg1.pose.pose.orientation.w])
		# p_init += math.pi/4.0
		initial_pose_headset.header.frame_id = "vive_world"
		first_LHR_5EA88DE7 = False
	else:
		#LHR_5EA88DE7_pose.pose = msg1.pose

		xx = msg1.pose.pose.position.x - initial_pose_headset.pose.position.x
		yy = msg1.pose.pose.position.y - initial_pose_headset.pose.position.y
		zz = msg1.pose.pose.position.z - initial_pose_headset.pose.position.z

		# rx = msg1.pose.pose.orientation.x -initial_pose_controller.pose.orientation.x
		# ry = msg1.pose.pose.orientation.y -initial_pose_controller.pose.orientation.y
		# rz = msg1.pose.pose.orientation.z -initial_pose_controller.pose.orientation.z
		# w = msg1.pose.pose.orientation.w -initial_pose_controller.pose.orientation.w

		(r, p, y) = tf.transformations.euler_from_quaternion([msg1.pose.pose.orientation.x, msg1.pose.pose.orientation.y, msg1.pose.pose.orientation.z, msg1.pose.pose.orientation.w])


		(r,p,y) = (r-r_init,p-p_init,y-y_init)
		# p += math.pi/4.0
		# y += math.pi/4.0

		# msg = JointState()
		# msg.position = [r, p, y]
		# msg.name = ['head_axis1', 'head_axis2', 'head_axis0']
		# msg.velocity = [0,0,0]
		# msg.effort = [0,0,0]
		# joint_targets_pub.publish(msg)
		# print("roll %f pitch %f yaw %f"%(r,p,y))

		LHR_5EA88DE7_pose.pose.position = Point(xx,yy,zz)
		# LHR_74656729_pose.pose.orientation = Quaternion(rx, ry, rz, w)
		q = tf.transformations.quaternion_about_axis(math.pi/4, (0, 1, 0))
		(qx,qy,qz,qw) = (msg1.pose.pose.orientation.x, msg1.pose.pose.orientation.y, msg1.pose.pose.orientation.z, msg1.pose.pose.orientation.w)
		orientation = tf.transformations.quaternion_multiply(q,(qx,qy,qz,qw))
		# LHR_5EA88DE7_pose.pose.orientation = Quaternion(tuple(orientation)) # msg1.pose.pose.orientation
		# quaternion = tf.transformations.quaternion_from_euler(r,p,y)
		# quaternion = tf.transformations.quaternion_multiply(quaternion, tf.transformations.quaternion_from_euler(math.pi/4.0, 0.0,0.0))
		# LHR_5EA88DE7_pose.pose.orientation = msg1.pose.pose.orientation
		LHR_5EA88DE7_pose.pose.orientation = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

		LHR_5EA88DE7_pose.header.frame_id = "vive_world"

def quaternion_normalize(q):
	norm = math.sqrt(q[0]*q[0] + q[3]*q[3] + q[2]*q[2] + q[1]*q[1])
	return [q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm]

if __name__ == "__main__":  
	rospy.init_node('tracker_transformer')
	ik = rospy.ServiceProxy('/execute_ik', InverseKinematics)
	li = tf.TransformListener()
	joint_targets_pub = rospy.Publisher("/joint_targets", JointState, queue_size=1)
	# joint_state_sub = rospy.Subscriber('/cardsflow_joint_states', JointState,  joint_state_cb, queue_size=1)
	marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
	controller_pose_sub = rospy.Subscriber('/vive/LHR_74656729_pose', PoseWithCovarianceStamped, LHR_74656729_cb)
	headset_pose_sub = rospy.Subscriber('/vive/LHR_5EA88DE7_pose', PoseWithCovarianceStamped, LHR_5EA88DE7_cb)

	time.sleep(1)

	# hand_left_pose = _get_link_pose("hand_left")
	head_pose = _get_link_pose("head")
	head_rpy = rospy.wait_for_message('/cardsflow_joint_states', JointState)
	for i in range(len(head_rpy.name)):
		if head_rpy.name[i] == head_axes[0]:
			r_head_init = head_rpy.position[i]
		elif head_rpy.name[i] == head_axes[1]:
			p_head_init = head_rpy.position[i]
		elif head_rpy.name[i] == head_axes[2]:
			y_head_init = head_rpy.position[i]

	LHR_29508350_pose = rospy.wait_for_message('/vive/LHR_29508350_pose', PoseWithCovarianceStamped)

	accumulated_targets = []
	rate = rospy.Rate(10)



	while not rospy.is_shutdown():
		# rate.sleep()
		'''
		current_pose_controller_vive = LHR_74656729_pose
		current_pose_controller_torso = li.transformPose("LHR_29508350", current_pose_controller_vive) #li.lookupTransform('LHR_74656729', 'world', rospy.Time(0))
		initial_pose_controller_torso = li.transformPose("LHR_29508350", initial_pose_controller)
		current_pose_controller_torso.pose.position.x += hand_left_pose.position.x #- initial_pose_controller_torso.pose.position.x
		current_pose_controller_torso.pose.position.y += hand_left_pose.position.y #- initial_pose_controller_torso.pose.position.y
		current_pose_controller_torso.pose.position.z += hand_left_pose.position.z #- initial_pose_controller_torso.pose.position.z
		'''
		# accumulated_targets.append(current_pose_controller_torso.pose.position)
		# if len(accumulated_targets) > 50:
		# 	accumulated_targets.pop(0)
		# current_pose_controller_torso.pose.position = running_mean(accumulated_targets)

		'''
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

		current_pose_controller_hand = li.transformPose("hand_left", current_pose_controller_torso)
		current_pose_controller_hand.pose.position.z -= 0.05

		requested_pose = li.transformPose("world", current_pose_controller_hand)

		# requested_pose = li.transformPose("world", current_pose_controller_torso)

		ik_request = InverseKinematicsRequest(endeffector="hand_left", target_frame="hand_left", pose=requested_pose.pose, type=1)

		ik(ik_request)
		'''
		#
		current_pose_tracker_vive = LHR_29508350_pose
		current_pose_headset_vive = LHR_5EA88DE7_pose
		current_pose_headset_torso = li.transformPose("LHR_29508350", current_pose_headset_vive)
		# pdb.set_trace()
		# transform = li.lookupTransform("LHR_29508350", "LHR_5EA88DE7",rospy.Time(0))
		# pdb.set_trace()
		# (r, p, y) = tf.transformations.euler_from_quaternion([transform[1][0], transform[1][1], transform[1][2], transform[1][3]])
		# (r, p, y) = tf.transformations.euler_from_quaternion([current_pose_headset_torso.pose.orientation.x, current_pose_headset_torso.pose.orientation.y, current_pose_headset_torso.pose.orientation.z, current_pose_headset_torso.pose.orientation.w])
		# msg = JointState()
		# (rr, pp, yy) = tf.transformations.euler_from_quaternion([head_pose.orientation.x, head_pose.orientation.y, head_pose.orientation.z, head_pose.orientation.w])
		# msg.position = [rr+p, pp, yy+y]
		# msg.name = ['head_axis1', 'head_axis0', 'head_axis2']
		# msg.velocity = [0,0,0]
		# msg.effort = [0,0,0]
		# joint_targets_pub.publish(msg)
		# print("roll %f pitch %f yaw %f"%(rr,pp,yy))

		current_pose_headset_torso.pose.position.x = head_pose.position.x
		current_pose_headset_torso.pose.position.y = head_pose.position.y
		current_pose_headset_torso.pose.position.z = head_pose.position.z+0.5
		current_pose_headset_torso.pose.orientation = current_pose_headset_torso.pose.orientation
		#
		# current_pose_headset_head = li.transformPose("head", current_pose_headset_torso)
		# current_pose_headset_head.pose.position.z += 0.5

		# requested_pose = li.transformPose("world", current_pose_headset_head)
		#q_tracker_1*q_tracker_2.inverse

		inverse = tf.transformations.quaternion_inverse([current_pose_headset_vive.pose.orientation.x, current_pose_headset_vive.pose.orientation.y, current_pose_headset_vive.pose.orientation.z, current_pose_headset_vive.pose.orientation.w])
		q_tracker = [current_pose_tracker_vive.pose.pose.orientation.x, current_pose_tracker_vive.pose.pose.orientation.y, current_pose_tracker_vive.pose.pose.orientation.z, current_pose_tracker_vive.pose.pose.orientation.w]
		inverse = quaternion_normalize(inverse)
		q_tracker = quaternion_normalize(q_tracker)
		q_new = tf.transformations.quaternion_multiply(inverse, q_tracker)
		# rpy_new = list(tf.transformations.euler_from_quaternion(q_new))
		# rpy_new[2] -= math.pi/4.0
		# q_new = tf.transformations.quaternion_from_euler(rpy_new[0], rpy_new[1], rpy_new[2])
		marker_msg = Marker()
		marker_msg.header.frame_id = "world"
		marker_msg.id = 103
		marker_msg.type = 10
		marker_msg.mesh_resource = 'package://robots/upper_body/meshes/collisions/head.stl'
		marker_msg.mesh_use_embedded_materials = 1
		marker_msg.action = 0
		marker_msg.text = "headset"
		marker_msg.scale = Vector3(0.001,0.001,0.001)
		# marker_msg.color = ColorRGBA(255.0,  255.0, 0.0, 90.0)

		# # import pdb; pdb.set_trace()
		marker_msg.pose.position = current_pose_headset_torso.pose.position
		marker_msg.pose.orientation = current_pose_headset_torso.pose.orientation #Quaternion(x=q_new[0], y=q_new[1], z=q_new[2], w=q_new[3])
		marker_pub.publish(marker_msg)

		joint_msg = JointState()
		joint_msg.name = ['head_axis0', 'head_axis1', 'head_axis2']
		(r_headset, p_headset, y_headset) = list(tf.transformations.euler_from_quaternion(
			[current_pose_headset_torso.pose.orientation.x,
			 current_pose_headset_torso.pose.orientation.y,
			 current_pose_headset_torso.pose.orientation.z,
			 current_pose_headset_torso.pose.orientation.w]))
		joint_msg.position = [r_head_init + r_headset, p_head_init + p_headset, y_head_init+y_headset]
		joint_msg.velocity = np.zeros(len(joint_msg.name))
		joint_msg.effort = np.zeros(len(joint_msg.name))
		# joint_targets_pub.publish(joint_msg)
		# ik_request = InverseKinematicsRequest(endeffector="head", target_frame="head", pose=requested_pose.pose, type=0)
		# ik(ik_request)