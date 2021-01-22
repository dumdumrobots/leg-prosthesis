#!/usr/bin/env python
from __future__ import print_function
import rospy
import numpy as np
import copy
from sensor_msgs.msg import JointState

from markers import *
from functions import *

rospy.init_node("kinControlPos")
print('\nKinematic Position Control v1.0 initiating ...\n')

pub = rospy.Publisher('joint_states', JointState, queue_size=10)

# Files
fxcurrent = open("/tmp/xcurrent.txt", "w")           	 
fxdesired = open("/tmp/xdesired.txt", "w")
fq = open("/tmp/q.txt", "w")

bmarker_current  = BallMarker(color['RED'])
bmarker_desired = BallMarker(color['GREEN'])

# Joint names
jnames = ['Rotational_1', 'Rotational_2', 'Rotational_3',
      	'Rotational_4', 'Rotational_5', 'Rotational_6']

# Desired position
xd = np.array([0.1827, -1.304, 0])
# Initial configuration
q0  = np.array([-0.7, -0.79, -1.57, -0.8, 0.0, 0.0])

# Resulting initial position 
T = fkine(q0)
x0 = T[0:3,3]
x = x0

print("Initial position: \n")
print(x0)
print("\nDesired position: \n")
print(xd)

# Current Position
bmarker_current.xyz(x0)

# Desired Position
bmarker_desired.xyz(xd)

# JointState message
jstate = JointState()
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
jstate.position = q0

freq = 200
dt = 1.0/freq
rate = rospy.Rate(freq)
e = np.array([[0],[0],[0]])
k = 0.5

q = copy(q0)

# Main loop
while not rospy.is_shutdown():
	jstate.header.stamp = rospy.Time.now()

	#Actual Position
	T = fkine(q)
	x = T[0:3,3]

	#Position Error
	e = x - xd

	#Diff. Error
	e_dot = -k * (e)

	#Jacobian
	J = jacobian_position(q)

	Jinv = np.linalg.pinv(J)

	#Diff. Joints
	q_dot =  Jinv.dot((e_dot))

	#Calculated Joints
	q = q + dt*q_dot
    
	# Log values                                                 	 
	fxcurrent.write(str(x[0])+' '+str(x[1]) +' '+str(x[2])+'\n')
	fxdesired.write(str(xd[0])+' '+str(xd[1])+' '+str(xd[2])+'\n')
	fq.write(str(q[0])+" "+str(q[1])+" "+str(q[2])+" "+str(q[3])+" "+ str(q[4])+" "+str(q[5])+"\n")

	if np.linalg.norm(e) < 0.001:
		break

	# Publish
	jstate.position = q
	pub.publish(jstate)
	bmarker_desired.xyz(xd)
	bmarker_current.xyz(x)
	
	rate.sleep()

print("\nFinal Error: \n")
print(e)
print('\nclosing ...\n')

fxcurrent.close()
fxdesired.close()
fq.close()
