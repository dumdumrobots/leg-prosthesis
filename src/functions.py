import numpy as np
from copy import copy
import rbdl

pi = np.pi

class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('/home/stingray/project_ws/src/prosthesis/model/urdf/leg.urdf')

    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq


def dh(d, theta, a, alpha):
  sth = np.sin(theta)
  cth = np.cos(theta)
  sa  = np.sin(alpha)
  ca  = np.cos(alpha)
  T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                [sth,  ca*cth, -sa*cth, a*sth],
                [0.0,      sa,      ca,     d],
                [0.0,     0.0,     0.0,   1.0]])
  return T


def fkine(q):

  d     = np.array([        0,    -0.043,       0,         0,    0.1945,          0])
  th    = np.array([q[0]+pi/2,   q[1]+pi, q[2]+pi,      q[3],   q[4]+pi,  q[5]+pi/2])
  a     = np.array([        0,    0.6842,   -0.45,         0,      0.06,       0.05])
  alpha = np.array([     pi/2,      pi/2,       0,      pi/2,      pi/2,          0])

  T1 = dh(d[0], th[0], a[0], alpha[0])
  T2 = dh(d[1], th[1], a[1], alpha[1])
  T3 = dh(d[2], th[2], a[2], alpha[2])
  T4 = dh(d[3], th[3], a[3], alpha[3])
  T5 = dh(d[4], th[4], a[4], alpha[4])
  T6 = dh(d[5], th[5], a[5], alpha[5])

  T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6)
    
  return T


def jacobian_position(q, delta=0.0001):

    J = np.zeros((3,6))

    T = fkine(q)
    X = T[0:3,3]

    for i in xrange(6):

         dq = copy(q)

         dq[i]= dq[i] + delta

         Tnew = fkine(dq)
         Xnew = Tnew[0:3,3]

         J[0,i] = (Xnew[0] -  X[0])/delta
         J[1,i] = (Xnew[1] -  X[1])/delta
         J[2,i] = (Xnew[2] -  X[2])/delta

    return J

def jacobian_pose(q, delta=0.0001):

    J = np.zeros((7,6))

    T = fkine(q)
    X = T[0:3,3]
    quat = rot2quat(T)

    for i in xrange(6):
         dq = copy(q)

         dq[i]= dq[i] + delta

         Tnew = fkine(dq)
         Xnew = Tnew[0:3,3]

         quatNew = rot2quat(Tnew)

         J[0,i] = (Xnew[0] -  X[0])/delta
         J[1,i] = (Xnew[1] -  X[1])/delta
         J[2,i] = (Xnew[2] -  X[2])/delta
         J[3,i] = (quatNew[0] -  quat[0])/delta
         J[4,i] = (quatNew[1] -  quat[1])/delta
         J[5,i] = (quatNew[2] -  quat[2])/delta
         J[6,i] = (quatNew[3] -  quat[3])/delta
     
    return J




def rot2quat(R):

    dEpsilon = 1e-6
    quat = 4*[0.,]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

    return np.array(quat)


def TF2xyzquat(T):

    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)


def skew(w):

    R = np.zeros([3,3])
    R[0,1] = -w[2]; R[0,2] = w[1]
    R[1,0] = w[2];  R[1,2] = -w[0]
    R[2,0] = -w[1]; R[2,1] = w[0]
    return R

def ikine(xdes, q0):

    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001
 
    q  = copy(q0)
    J=jacobian_position(q,0.0001)
    
    for i in range(max_iter):

        f=fkine(q)
        e=xdes-f[0:3,3]  
        q=q+np.dot(np.linalg.pinv(J),e) 

        if(np.linalg.norm(e)<epsilon):
            break

    return q