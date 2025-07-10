#!/usr/bin/env python3
import time
import roslib
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from functions import fkine, ikine, TF2xyzquat

if __name__ == '__main__':
    
 # Iniciar un nodo
    rospy.init_node("comand_gazebo")

 # Declara la acción del tipo cliente
    srv = "/pos_joint_traj_controller/follow_joint_trajectory"
    client = actionlib.SimpleActionClient(srv, FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for trajectory action")

    print("Waiting for server...")

 # Metodo para espera conectarte al servidor de la acción
    client.wait_for_server()
    rospy.loginfo("Connected")

    print("Connected to server")

 # Declara las variables del brazo robotico
 # Lista de nombre
    joint_names = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint", "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
 # Lista de valores de la configuracion del robot
    Q0 = np.array([0, -1.57, 1.57, 0, 1, 0])
 
 # Definir el tipo de mensaje a utilizar
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
 
 # Definir los nombres de las articulaciones de g.trajectory
    g.trajectory.joint_names = joint_names
 # Definir la posicion inicial de g.trajectory

    g.trajectory.points = [JointTrajectoryPoint(positions = Q0, velocities = [0] * 6, time_from_start = rospy.Duration(2.0))]
 # Enviar el objetivo y esperar por el resultado
    client.send_goal(g)
    client.wait_for_result()
    print(client.get_result())
 # Dormir un segundo
    rospy.sleep(1)
    
 # Para probar crear un bucle para que el robot mueva quinta articulacion 0.05 rad/s
 # Usar el metodo "robot_client.cancel_goal()" cada vez antes de iniciar el bucle
 # Para asegurarnos de terminar la accion antes de empezar otra
    client.cancel_goal()
    xdes_ini = np.array([0.5, -0.4, 0.5])
    xdes_fin = np.array([0.5, 0.4, 0.5])

    pasos = 20   # cantidad de pasos para el movimiento
    rate = rospy.Rate(10)  # 10 Hz, puedes ajustar

    for i in range(pasos + 1):  
        if rospy.is_shutdown():
            break
    
        # interpolación lineal
        alpha = i / pasos
        xdes_actual = (1 - alpha) * xdes_ini + alpha * xdes_fin

    # calcular configuración articular para la posición deseada
        q_nuevo = ikine(xdes_actual, Q0)

    # construir el nuevo goal
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = joint_names
        p = JointTrajectoryPoint()
        p.positions = q_nuevo.tolist()  # aseguramos lista
        p.velocities = [0]*6
        p.time_from_start = rospy.Duration(0.2)
        g.trajectory.points = [p]

        client.send_goal(g)
        rate.sleep()

 # Despues de terminado el bucle, cancelar la ultima accion enviada
    client.cancel_goal()