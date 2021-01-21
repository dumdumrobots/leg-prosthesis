#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

from markers import *
from functions import *

rospy.init_node("testForwardKinematics")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker = BallMarker(color['GREEN'])

# Joint names
jnames = ['Rotational_1', 'Rotational_2', 'Rotational_3',
      	'Rotational_4', 'Rotational_5', 'Rotational_6']
# Joint Configuration
q = [-0.45, 0.0, -1.00, -0.8, 0.0, 0.0]

# End effector with respect to the base
T = fkine(q)

print("\nJoint Configuration: \n" + str(q) + "\n")
print("Transformation from base to end effector\n ")
print(np.round(T, 3))
print("\n")

bmarker.position(T)

# Object (message) whose type is JointState
jstate = JointState()
# Set values to the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q

# Loop rate (in Hz)
rate = rospy.Rate(100)
# Continuous execution loop
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    # Wait for the next iteration
    rate.sleep()
