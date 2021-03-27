#!/usr/bin/env python3

import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Vector3
from smartarm_msgs.msg import SonicArray
from smartarm_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from scipy.spatial.transform import Rotation as R


def orientation(acce, meg):
    east = np.cross(acce, meg)
    north = np.cross(east, acce)
    down = acce / np.linalg.norm(acce)
    east = east / np.linalg.norm(east)
    north = north / np.linalg.norm(north)

    rotation = np.zeros((3,3))
    print(down)
    rotation[:,0] = down
    rotation[:,1] = east
    rotation[:,2] = acce
    rotation = np.matrix(rotation)
    print(rotation)
    #rotation = rotation.I

    rot3 = R.from_dcm(rotation)
    q = rot3.as_quat()

    return q


def imu_callback(data):
    quat = orientation([data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z], \
        [data.magnetic_field.x,data.magnetic_field.y,data.magnetic_field.z])
    rospy.loginfo('Rotation quaternion: %f, %f, %f, %f', quat[0], quat[1], quat[2], quat[3])
    pub = rospy.Publisher("end_eff_pose", PoseStamped, queue_size=1)
    pose = PoseStamped()
    position = Point()
    quaternion = Quaternion()
    position.x = 0.5
    position.y = 0.5
    position.z = 0.5
    quaternion.x = quat[0]
    quaternion.y = quat[1]
    quaternion.z = quat[2]
    quaternion.w = quat[3]
    pose.header.frame_id = "base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position = position
    pose.pose.orientation = quaternion
    pub.publish(pose)
    pub.unregister


def main():
    rospy.init_node('imu_test_node')
    #pub = rospy.Publisher("end_eff_position", position, queue_size=100)
    rospy.Subscriber("imu_data", numpy_msg(Imu), imu_callback)
    #pub = rospy.Publisher("end_eff_position", position, queue_size=100)
    rospy.loginfo("node established!")
    rospy.spin()


if __name__ == '__main__':
    main()

