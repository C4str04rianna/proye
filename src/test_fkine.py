#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

from markers import *
from functions import *

if __name__ == '__main__':

 rospy.init_node("testForwardKinematics")
 pub = rospy.Publisher('joint_states', JointState, queue_size=1)
 bmarker = BallMarker(color['GREEN'])
 marker = FrameMarker()
 
 # Joint names
 jnames = ["motor1", "motor2", "motor3", "motor4", "motor5", "motor6", "Corredera9", "Corredera10"]
 # Joint Configuration
 q = np.array([2.54, 1.25, 0.19, 0.58, -1.63, -0.54, 0.0, 0.0])

 # End effector with respect to the base
 T = fkine(q)
 print("Vector de configuración:\n", np.round(q[0:6], 3),"\n")
 print("Matriz de transformación final:\n", np.round(T, 3))
 bmarker.position(T)
 
 x0 = TF2xyzquat(T)
 marker.setPose(x0)

 # Object (message) whose type is JointState
 jstate = JointState()
 # Set values to the message
 jstate.header.stamp = rospy.Time.now()
 jstate.name = jnames
 # Add the head joint value (with value 0) to the joints
 jstate.position = q

 # Loop rate (in Hz)
 rate = rospy.Rate(20)
 # Continuous execution loop
 while not rospy.is_shutdown():
  # Current time (needed for ROS)
  jstate.header.stamp = rospy.Time.now()
  # Publish the message
  pub.publish(jstate)
  #bmarker.publish()
  marker.publish()
  # Wait for the next iteration
  rate.sleep()