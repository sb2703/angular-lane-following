#!/usr/bin/env python
import rospy
from dynamic_reconfigure.server import Server
from lane_control.cfg import ConfigConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {cruise_control_speed}""".format(config))
    rospy.loginfo("""Reconfigure Request: {Kp}""".format(config))
    rospy.loginfo("""Reconfigure Request: {Ki}""".format(config))
    rospy.loginfo("""Reconfigure Request: {Kd}""".format(config))

    return config

if __name__ == '__main__':
    rospy.init_node("lane_control_rec", anonymous=False)
    srv = Server(ConfigConfig, callback)
    rospy.spin()
