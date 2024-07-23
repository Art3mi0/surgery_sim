#!/usr/bin/env python3

import rospy
import tf
import math
import numpy as np

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

if __name__ == '__main__':
	rospy.init_node('tf_a')
	listener = tf.TransformListener()

	rate = rospy.Rate(1.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/checkerboard', '/camera_color_optical_frame', rospy.Time(0))
			(r,p,y) = euler_from_quaternion(rot[0], rot[1], rot[2], rot[3])
			print("trans:", trans, "\nrot Quaternion:", rot, "\nrot RPY (radian):", r, p, y,
			"\nrot RPY (degree)", np.rad2deg(r), np.rad2deg(p), np.rad2deg(y), "\n") 
		except (tf.LookupException, tf.ConnectivityException):
			continue

		rate.sleep()
