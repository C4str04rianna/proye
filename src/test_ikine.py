#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from markers import *
from functions import *
import numpy as np

pi=np.pi

if __name__ == '__main__':

    rospy.init_node("testInverseKinematics")  # <- Nombre del nodo
    pub = rospy.Publisher('joint_states', JointState, queue_size=1)
    bmarker = BallMarker(color['GREEN'])
    marker = FrameMarker()

    # Nombres de las articulaciones del UR5
    jnames = ["motor1", "motor2", "motor3", "motor4", "motor5", "motor6", "Corredera9", "Corredera10"]
    
    # posicion deseada (en metros) del efector final:
    xdes = np.array([0.751, 0.26, 0.024])
    
    # Configuración articular inicial (puede ajustarse si quieres partir de una pose distinta)
    q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Cálculo de la solución de cinemática inversa
    q_sol,iter = ikine(xdes, q0)
    q_sol = (q_sol %(pi*2))
    print("Configuración inicial:\n",np.round(q0,3),"\n")
    print("Solución de IK:\n",np.round(q_sol,3),"\n")

    # Cinemática directa para verificar la posición alcanzada
    Tsol = fkine(q_sol)
    pos_final = Tsol[0:3,3]
    print("Posición alcanzada:\n",np.round(pos_final,3))
    cero = np.array([0])
    q_sol = np.concatenate((q_sol, cero, cero))

    # Publicar el marcador visual del efector final
    xsol = TF2xyzquat(Tsol)
    marker.setPose(xsol)

    # Mensaje tipo JointState
    jstate = JointState()
    jstate.header.stamp = rospy.Time.now()
    jstate.name = jnames
    jstate.position = q_sol

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        jstate.header.stamp = rospy.Time.now()
        pub.publish(jstate)
        marker.publish()
        rate.sleep()