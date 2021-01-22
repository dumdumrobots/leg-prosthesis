#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from markers import *
from functions import *

actualKey = ""

def keyPressed(data):
    global actualKey 
    actualKey = data.data

rospy.init_node("testForwardKinematics")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker = BallMarker(color['GREEN'])

# Joint names
jnames = ['Rotational_1', 'Rotational_2', 'Rotational_3',
      	'Rotational_4', 'Rotational_5', 'Rotational_6']
# Joint Configuration
q = [-0.45, 0.0, -1.00, -0.8, 0.0, 0.0]

# End effector from the base
T = fkine(q)

print("\nJoint Configuration: \n" + str(q) + "\n")
print("Transformation from base to end effector\n ")
print(np.round(T, 3))
print("\n")

bmarker.position(T)

# JointState Message
jstate = JointState()
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
jstate.position = q

rate = rospy.Rate(100)

index = 0
delta = 0.005

while not rospy.is_shutdown():

	key = rospy.Subscriber('keys',String,keyPressed)

	if (actualKey == "1"):
		index = 0
		print("ActualIndex =", index)
	
	elif (actualKey == "2"):
		index = 1
		print("ActualIndex =", index)
	
	elif (actualKey == "3"):
		index = 2
		print("ActualIndex =", index)
	
	elif (actualKey == "4"):
		index = 3
		print("ActualIndex =", index)
	
	elif (actualKey == "5"):
		index = 4
		print("ActualIndex =", index)
	
	elif (actualKey == "6"):
		index = 5
		print("ActualIndex =", index)
	
	if (actualKey == "w"):
		q[index] += delta
	
	if (actualKey == "s"):
		q[index] -= delta
	
	if (actualKey == "r"):
		q = np.array([-0.45, 0.0, -1.00, -0.8, 0.0, 0.0])

	jstate.header.stamp = rospy.Time.now()
	jstate.position = q

	T = fkine(q)
	bmarker.position(T)

	jstate.header.stamp = rospy.Time.now()

	pub.publish(jstate)
	bmarker.publish()
	rate.sleep()
