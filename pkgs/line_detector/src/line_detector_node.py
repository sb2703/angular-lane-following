#!/usr/bin/env python3
import copy

import numpy as np
import cv2
import rospy
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from dynamic_reconfigure.server import Server
from line_detector.cfg import ConfigConfig
from std_msgs.msg import Bool, Float64


from duckietown.dtros import DTROS, NodeType, TopicType


class LineDetectorNode(DTROS):
    """
    This modified version of ``LineDetectorNode`` is responsible for detecting 
    the white and yellow lines in an image and is used for lane localization.

    Publishers:
    /lane following (:obj:`Image`): Debug image with detected lines
    /error (:obj:`Float64`): The error estimated by the line detector
    /found_white (:obj:`Bool`): True if white color is detected, False otherwise
    /found_yellow (:obj:`Bool`): True if yellow color is detected, False otherwise

    Subscriber:
    ~image/compressed (:obj:`CompressedImage`): The camera images

    """

    def __init__(self, node_name):
        
        # Initialize the DTROS parent class
        super(LineDetectorNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
	
        self.bridge = CvBridge()

        # Initialize variables from configuration file
        self.srv = Server(ConfigConfig, self.callback_config)

        # Publishers
        self.pub_img = rospy.Publisher('/lane_following', Image, queue_size=1, dt_topic_type=TopicType.PERCEPTION)
        self.pub_error = rospy.Publisher('/error', Float64, queue_size=1, dt_topic_type=TopicType.PERCEPTION)
        self.pub_white = rospy.Publisher('/found_white', Bool, queue_size=1, dt_topic_type=TopicType.PERCEPTION)
        self.pub_yellow = rospy.Publisher('/found_yellow', Bool, queue_size=1, dt_topic_type=TopicType.PERCEPTION)
        
        # Subscriber
        rospy.Subscriber('~image/compressed', CompressedImage, self.callback, queue_size=1, buff_size=2**24)
        
        self.log("Initialized!")

    def distance(self, x1, y1, x2, y2, x):
        # Function that computes distance of two points

        if (x2 - x1) == 0:
            #x = y
            y = x
        else:
            m = (y2 - y1) / (x2 - x1)
            q = (x2 * y1 - x1 * y2) / (x2 - x1)
            y = m * x + q
        return y

        return x
        
    
    def callback_config(self, config, level):
        # Callback for dynamic reconfigure

        global H_YELLOW_MIN
        H_YELLOW_MIN = config["h_yellow_min"]
        global S_YELLOW_MIN
        S_YELLOW_MIN = config["s_yellow_min"]
        global V_YELLOW_MIN
        V_YELLOW_MIN = config["v_yellow_min"]

        global H_WHITE_MIN
        H_WHITE_MIN = config["h_white_min"]
        global S_WHITE_MIN
        S_WHITE_MIN = config["s_white_min"]
        global V_WHITE_MIN
        V_WHITE_MIN = config["v_white_min"]

        global H_RED_MIN
        H_RED_MIN = config["h_red_min"]
        global S_RED_MIN
        S_RED_MIN = config["s_red_min"]
        global V_RED_MIN
        V_RED_MIN = config["v_red_min"]

        global H_YELLOW_MAX
        H_YELLOW_MAX = config["h_yellow_max"]
        global S_YELLOW_MAX
        S_YELLOW_MAX = config["s_yellow_max"]
        global V_YELLOW_MAX
        V_YELLOW_MAX = config["v_yellow_max"]

        global H_WHITE_MAX
        H_WHITE_MAX = config["h_white_max"]
        global S_WHITE_MAX
        S_WHITE_MAX = config["s_white_max"]
        global V_WHITE_MAX
        V_WHITE_MAX = config["v_white_max"]

        global H_RED_MAX
        H_RED_MAX = config["h_red_max"]
        global S_RED_MAX
        S_RED_MAX = config["s_red_max"]
        global V_RED_MAX
        V_RED_MAX = config["v_red_max"]
        
        return config
    

    def callback(self, data):
        """
        Processes the incoming image messages.

        In order to process an image and compute error, 
        the following operations are executed:
        - Image is cropped, then a HSV copy is made
        - For each color, a color mask is applied
        - For each color, Canny edge detector and Hough transform are used
        - Lines are drawn on the debug image
        - Angles are used to compute error

        """

        found_yellow, found_white, found_red = False, False, False
        
        cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        (h, w, c) = cv_image.shape

        # Inizialization
        error_yellow = 0
        error_white = 0

        theta_yellow = 45
        theta_white = 45

        lines_check_white = False
        lines_check_yellow = False

        # Crop image
        cropped_image = copy.deepcopy(cv_image)
        cropped_image[0:int(h/2), :] = 0

        # White line acquisition
        cv_image_white = copy.deepcopy(cropped_image)
        cv_image_white[:, 0:int(w/2)] = 0
        hsv = cv2.cvtColor(cv_image_white, cv2.COLOR_RGB2HSV)

        # Color range
        low_white = np.array([H_WHITE_MIN, S_WHITE_MIN, V_WHITE_MIN])
        up_white = np.array([H_WHITE_MAX, S_WHITE_MAX, V_WHITE_MAX])
        mask = cv2.inRange(hsv, low_white, up_white)
        edges_white = cv2.Canny(mask, 75, 150)

        lines = cv2.HoughLinesP(edges_white, 1, np.pi/180, 50, maxLineGap=50)

        # Line drawing
        if lines is not None:
            found_white = True
            for i in range(min(2, len(lines))):
                for line in lines[i]:
                    pt1 = (line[0], line[1])
                    pt2 = (line[2], line[3])

                    cv2.line(cv_image, pt1, pt2, (0, 255, 0), 2)  # (0, 255, 0) is green

                    # Theta
                    y_white = self.distance(line[0], line[1], line[2], line[3], (h * 75) / 100)  

                    yb_white = self.distance(line[0], line[1], line[2], line[3], (h * 20) / 100)  

                    if yb_white is not None:
                        theta_white = math.degrees(math.atan2((h * 75) / 100, y_white))  

                        error_white = theta_white - 45.0

        # Yellow line acquisition
        cv_image_yellow = copy.deepcopy(cropped_image)
        cv_image_yellow[:, int(w / 3*2):w] = 0
        hsv1 = cv2.cvtColor(cv_image_yellow, cv2.COLOR_RGB2HSV)

        # Color range
        low_yellow = np.array([H_YELLOW_MIN, S_YELLOW_MIN, V_YELLOW_MIN])
        up_yellow = np.array([H_YELLOW_MAX, S_YELLOW_MAX, V_YELLOW_MAX])
        mask1 = cv2.inRange(hsv1, low_yellow, up_yellow)
        edges_yellow = cv2.Canny(mask1, 75, 150)

        lines1 = cv2.HoughLinesP(edges_yellow, 1, np.pi/180, 50, maxLineGap=50)

        # Lines drawing
        if lines1 is not None:
            found_yellow = True
            for i in range(min(2, len(lines1))):
                for line in lines1[i]:
                    pt1 = (line[0], line[1])
                    pt2 = (line[2], line[3])
                    cv2.line(cv_image, pt1, pt2, (0, 0, 255), 2)  # (0, 0, 255) is blue

                    # Theta
                    y_yellow = self.distance(line[0], line[1], line[2], line[3], (h * 67) / 100)

                    theta_yellow = math.degrees(math.atan2((h * 67) / 100, y_yellow))

                    error_yellow = theta_yellow - 45.0

        # Red line acquisition (not used)
        cv_image_red = copy.deepcopy(cropped_image)
        cv_image_red[0:int(h/3*2), :] = 0
        hsv_r = cv2.cvtColor(cv_image_red, cv2.COLOR_RGB2HSV)

        # Color range
        low_red = np.array([H_RED_MIN, S_RED_MIN, V_RED_MIN])
        up_red = np.array([H_RED_MAX, S_RED_MAX, V_RED_MAX])
        mask_r = cv2.inRange(hsv_r, low_red, up_red)
        edges_red = cv2.Canny(mask_r, 75, 150)

        lines_r = cv2.HoughLinesP(edges_red, 1, np.pi / 180, 50, maxLineGap=50) 

        # Draw line
        if lines_r is not None:
            found_red = True
            for i in range(min(1, len(lines_r))):
                for line in lines_r[i]:
                    pt1 = (line[0], line[1])
                    pt2 = (line[2], line[3])
                    cv2.line(cv_image, pt1, pt2, (255, 0, 0), 2) # (0, 0, 255) is red

        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        # Computes error
        error_z = error_white-error_yellow
        self.pub_error.publish(error_z)
        self.pub_white.publish(found_white)
        self.pub_yellow.publish(found_yellow)

        # Publish debug image with lines
        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.pub_img.publish(msg)


if __name__ == "__main__":
    
    # Global variables for colors
    H_YELLOW_MIN = None
    S_YELLOW_MIN = None
    V_YELLOW_MIN = None

    H_WHITE_MIN = None
    S_WHITE_MIN = None
    V_WHITE_MIN = None

    H_RED_MIN = None
    S_RED_MIN = None
    V_RED_MIN = None

    H_YELLOW_MAX = None
    S_YELLOW_MAX = None
    V_YELLOW_MAX = None

    H_WHITE_MAX = None
    S_WHITE_MAX = None
    V_WHITE_MAX = None

    H_RED_MAX = None
    S_RED_MAX = None
    V_RED_MAX = None
    
    try:
        # Initialize the node
        line_detector_node = LineDetectorNode(node_name="line_detector_node")
        # Keep it spinning to keep the node alive
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
