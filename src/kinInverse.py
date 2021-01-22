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

rospy.init_node("InverseKinematic")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)

bmarker      = BallMarker(color['RED'])
bmarker_des  = BallMarker(color['GREEN'])

# Joint names
jnames = ['Rotational_1', 'Rotational_2', 'Rotational_3', 'Rotational_4', 'Rotational_5', 'Rotational_6']

# Posicion deseada
pd = np.array([0.1827, -1.304, 0.04])
# Configuracion inicial de las articulaciones
ang0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# Cinematica Inversa
ang = ikine(pd, ang0)

print("\nInitial Joint Configuration: \n" + str(ang0) + "\n")

print("\nDesired Position: \n" + str(pd) + "\n")

# Posicion resultante del efector final con respecto a la base
T = fkine(ang)

print("Calculated Transformation from base to end effector\n ")
print(np.round(T, 3))
print("\n")

# El marcador rojo indica la posicion deseada
bmarker.xyz(T[0:3,3])
# El marcador verde muestra la posicion alcanzada
bmarker_des.xyz(pd)

# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Asignar valores al mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames

jstate.position = ang

# Cadencia de envios de mensaje (en Hz)
rate = rospy.Rate(100)
delta = 0.001

# Bucle
while not rospy.is_shutdown():

	key = rospy.Subscriber('/keys',String,keyPressed)

	norma = np.linalg.norm(pd)

	if actualKey == "w":
		pd[0] += delta

	if actualKey == "s":
		pd[0] -= delta

	if actualKey == "a":
		pd[1] += delta

	if actualKey == "d":
		pd[1] -= delta

	if actualKey == "q":
		pd[2] += delta

	if actualKey == "e":
		pd[2] -= delta

	if actualKey == "r":
		pd = np.array([0.1827, -1.304, 0.04])
	
	ang = ikine(pd, ang)
	T = fkine(ang)

	bmarker.xyz(T[0:3,3])
	bmarker_des.xyz(pd)

	# Captura del tiempo actual (requerido por ROS)
	jstate.header.stamp = rospy.Time.now()
	jstate.position = ang

	pub.publish(jstate)
	bmarker.publish()
	bmarker_des.publish()

	# Se espera a la siguiente iteracion
	rate.sleep()