#!/usr/bin/env python
######### coded by Eungchang Mason Lee

import rospy
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
import tf

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class caster():
    def __init__(self):
        rospy.init_node('tf_broadcaster', anonymous=True)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.base_cb)
        self.rate = rospy.Rate(30)

        self.base_check=0
        self.br = tf.TransformBroadcaster()

    def base_cb(self, msg):
        self.base_check=1
        self.header = msg.header.stamp
        self.br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),\
(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),\
self.header,"base_link","map")
        self.br.sendTransform((0.0, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0), self.header, "os_lidar","base_link")
        self.br.sendTransform((0.1, 0.0, 0.0), (0.5,-0.5,0.5,-0.5), self.header, "d435i/depth_camera_link","base_link")
        return

if __name__ == '__main__':
    cas = caster()
    while 1:
        try:
            cas.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
