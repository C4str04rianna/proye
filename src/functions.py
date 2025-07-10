import numpy as np
from copy import copy
import os

os.system('cls') # limpiamos terminal
cos=np.cos; sin=np.sin; pi=np.pi


def dh(d, theta, a, alpha):
    """
    Matriz de transformacion homogenea asociada a los parametros DH.
    Retorna una matriz 4x4
    """
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
 """
 Calcular la cinematica directa del brazo robotico dados sus valores articulares. 
 q es un vector numpy de la forma [q1, q2, q3, ..., qn]
 """
 # Longitudes (en metros)
 # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
 T0 = dh(0,-pi, 0, 0)
 T1 = dh(0.1, q[0],0, -pi/2)
 T2 = dh(0.3, q[1], 0, pi/2)
 T3 = dh(0.25, q[2], 0, -pi/2)
 T4 = dh(0.3, q[3], 0, pi/2)
 T5 = dh(0.3, q[4], 0, -pi/2)
 T6 = dh(0.14, q[5], 0, pi/2)
 # Efector final con respecto a la base
 T = T = T0 @ T1 @ T2 @ T3 @ T4 @ T5 @ T6 
 return T

def posicion(q):
 T = fkine(q)
 P = T[0:3,3]
 Pos = np.array([[P[0]],[P[1]],[P[2]]])
 return Pos

def jacobian_position(q, delta=0.0001):
 """
 Jacobiano analitico para la posicion de un brazo robotico de n grados de libertad. 
 Retorna una matriz de 3xn y toma como entrada el vector de configuracion articular 
 q=[q1, q2, q3, ..., qn]
 """
 q1 = q[0]
 q2 = q[1]
 q3 = q[2]
 q4 = q[3]
 q5 = q[4]
 q6 = q[5]

 col1 = 1/delta * (posicion([q1+delta, q2, q3, q4, q5, q6]) - posicion(q))
 col2 = 1/delta * (posicion([q1, q2+delta, q3, q4, q5, q6]) - posicion(q))
 col3 = 1/delta * (posicion([q1, q2, q3+delta, q4, q5, q6]) - posicion(q))
 col4 = 1/delta * (posicion([q1, q2, q3, q4+delta, q5, q6]) - posicion(q))
 col5 = 1/delta * (posicion([q1, q2, q3, q4, q5+delta, q6]) - posicion(q))
 col6 = 1/delta * (posicion([q1, q2, q3, q4, q5, q6+delta]) - posicion(q))

 J = np.hstack((col1, col2, col3, col4, col5, col6))
 return J

def ikine(xdes, q0):
 """
 Calcular la cinematica inversa de un brazo robotico numericamente a partir 
 de la configuracion articular inicial de q0. Emplear el metodo de newton.
 """
 epsilon  = 0.00001
 max_iter = 1000
 iter = 0
 q  = copy(q0)
 for i in range(max_iter):
  # Main loop
  J = jacobian_position(q)
  f = fkine(q)[0:3,3]
  e = xdes-f
  q = q + np.dot(np.linalg.pinv(J),e)
  iter = iter + 1
  if (np.linalg.norm(e) < epsilon):
    break
    
 return q,iter

def rot2quat(R):
 """
 Convertir una matriz de rotacion en un cuaternion

 Entrada:
  R -- Matriz de rotacion
 Salida:
  Q -- Cuaternion [ew, ex, ey, ez]

 """
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
 """
 Convert a homogeneous transformation matrix into the a vector containing the
 pose of the robot.

 Input:
  T -- A homogeneous transformation
 Output:
  X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
       is Cartesian coordinates and the last part is a quaternion
 """
 quat = rot2quat(T[0:3,0:3])
 res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
 return np.array(res)