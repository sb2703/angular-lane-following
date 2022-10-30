#!/usr/bin/env python
import rospy
from dynamic_reconfigure.server import Server
from line_detector.cfg import ConfigConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {h_yellow_min}""".format(config))
    rospy.loginfo("""Reconfigure Request: {s_yellow_min}""".format(config))
    rospy.loginfo("""Reconfigure Request: {v_yellow_min}""".format(config))

    rospy.loginfo("""Reconfigure Request: {h_white_min}""".format(config))
    rospy.loginfo("""Reconfigure Request: {s_white_min}""".format(config))
    rospy.loginfo("""Reconfigure Request: {v_white_min}""".format(config))

    rospy.loginfo("""Reconfigure Request: {h_red_min}""".format(config))
    rospy.loginfo("""Reconfigure Request: {s_red_min}""".format(config))
    rospy.loginfo("""Reconfigure Request: {v_red_min}""".format(config))

    rospy.loginfo("""Reconfigure Request: {h_yellow_max}""".format(config))
    rospy.loginfo("""Reconfigure Request: {s_yellow_max}""".format(config))
    rospy.loginfo("""Reconfigure Request: {v_yellow_max}""".format(config))

    rospy.loginfo("""Reconfigure Request: {h_white_max}""".format(config))
    rospy.loginfo("""Reconfigure Request: {s_white_max}""".format(config))
    rospy.loginfo("""Reconfigure Request: {v_white_max}""".format(config))

    rospy.loginfo("""Reconfigure Request: {h_red_max}""".format(config))
    rospy.loginfo("""Reconfigure Request: {s_red_max}""".format(config))
    rospy.loginfo("""Reconfigure Request: {v_red_max}""".format(**config))

    return config

if __name__ == '__main__':
    rospy.init_node("line_detector_rec", anonymous=False)
    srv = Server(ConfigConfig, callback)
    rospy.spin()
