#!/usr/bin/env python
 
import rospy
from sensor_msgs.msg import JointState
from markers import *
from functions import *
from roslib import packages
import numpy as np
from copy import copy
import rbdl

bmarker_actual  = BallMarker(color['RED'])
bmarker_deseado = BallMarker(color['GREEN'])

rospy.init_node("control_pdg")
print('\nDynamic Position Control v1.0 initiating ...\n')

fxcurrent = open("/tmp/xcurrent.txt", "w")              
fxdesired = open("/tmp/xdesired.txt", "w")
fq = open("/tmp/q.txt", "w")

pub = rospy.Publisher('joint_states', JointState, queue_size=1000)

 
# Nombres de las articulaciones
jnames = ['Rotational_1', 'Rotational_2', 'Rotational_3',
         'Rotational_4', 'Rotational_5', 'Rotational_6']
# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Valores del mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
 
# =============================================================
# Configuracion articular inicial (en radianes)
q = np.array([-0.7, -0.79, -1.57, -0.8, 0.0, 0.0])

# Velocidad inicial
dq = np.array([0., 0., 0., 0., 0., 0.])

# Aceleracion inicial
ddq = np.array([0., 0., 0., 0., 0., 0.])

# Configuracion articular deseada
qdes = np.array([0.11, 0, 0, -0.8, 0.1, -0.15])

# Velocidad articular deseada
dqdes = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# Aceleracion articular deseada
ddqdes = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# =============================================================
 
# Posicion resultante de la configuracion articular deseada
xdes = fkine(qdes)[0:3,3]

print("Initial configuration: \n")
print(q)
print("\nDesired configuration: \n")
print(qdes)

# Copiar la configuracion articular en el mensaje a ser publicado
jstate.position = q
pub.publish(jstate)
 
# Modelo RBDL
modelo = rbdl.loadModel('/home/stingray/project_ws/src/prosthesis/model/urdf/leg.urdf')
ndof   = modelo.q_size     # Grados de libertad
zeros = np.zeros(ndof)     # Vector de ceros
 
# Frecuencia del envio (en Hz)
freq = 20
dt = 1.0/freq
rate = rospy.Rate(freq)
 
# Simulador dinamico del robot
robot = Robot(q, dq, ndof, dt)
 
# Bucle de ejecucion continua
t = 0.0
 
# Se definen las ganancias del controlador
valores = 1*np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
Kp = np.diag(valores)
Kd = 2*np.sqrt(Kp)
 
# Arrays numpy
zeros = np.zeros(ndof)          # Vector de ceros
tau   = np.zeros(ndof)          # Para torque
g     = np.zeros(ndof)          # Para la gravedad
c     = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
M     = np.zeros([ndof, ndof])  # Para la matriz de inercia
e     = np.eye(6)               # Vector identidad
 
while not rospy.is_shutdown():
 
   # Leer valores del simulador
   q  = robot.read_joint_positions()
   dq = robot.read_joint_velocities()
   # Posicion actual del efector final
   x = fkine(q)[0:3,3]
   # Tiempo actual (necesario como indicador para ROS)
   jstate.header.stamp = rospy.Time.now()
   
   # ----------------------------
   # Control dinamico
   # ----------------------------
   rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
   rbdl.CompositeRigidBodyAlgorithm(modelo,q,M)
   rbdl.NonlinearEffects(modelo,q,dq,c)

   u = M.dot( ddqdes + Kd.dot(dqdes - dq) + Kp.dot(qdes-q) ) + c
 
   if np.linalg.norm(qdes-q)< 0.01:
       break
   # Simulacion del robot
   robot.send_command(u)

   # Log values                                                    
   fxcurrent.write(str(x[0])+' '+str(x[1]) +' '+str(x[2])+'\n')
   fxdesired.write(str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
   fq.write(str(q[0])+" "+str(q[1])+" "+str(q[2])+" "+str(q[3])+" "+ str(q[4])+" "+str(q[5])+"\n")
 
   # Publicacion del mensaje
   jstate.position = q
   pub.publish(jstate)
   bmarker_deseado.xyz(xdes)
   bmarker_actual.xyz(x)
   t = t+dt
   # Esperar hasta la siguiente  iteracion
   rate.sleep()

print("\nFinal Error: \n")
print(qdes-q)
print('\nclosing ...\n')

fxcurrent.close()
fxdesired.close()
fq.close()