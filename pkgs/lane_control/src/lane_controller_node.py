#!/usr/bin/env python3
import numpy as np
import rospy

#from geometry_msgs.msg import Twist
from duckietown_msgs.msg import Twist2DStamped
from simple_pid import PID
from dynamic_reconfigure.server import Server
from lane_control.cfg import ConfigConfig
from std_msgs.msg import Bool, Float64

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType

class LaneControllerNode(DTROS):
    """
    This modified version of ``LaneControllerNode`` computes control action.
    The node computes the commands in form of linear and angular velocities, 
    by processing the error obtained from ``LineDetectorNode``.

    Publisher:
    ~car_cmd (:obj:`Twist2DStamped`): The computed control action

    Subscriber:
    /error (:obj:`Float64`): The error estimated by the line detector
    /found_white (:obj:`Bool`): True if white color is detected, False otherwise
    /found_yellow (:obj:`Bool`): True if yellow color is detected, False otherwise
    
    """
    
    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LaneControllerNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        
        # Initialize variables
        self.found_white = None
        self.found_yellow = None

        # Global PID
        set_point_speed_x = 0.1
        self.pid_z = PID(1.0, 0.02, 0.0, setpoint=0)
        self.pid_z.output_limits = (-10.0, 10.0)
        
        # Initialize variables from configuration file
        self.srv = Server(ConfigConfig, self.callback_config)
        
        # Publisher
        self.pub_cmd_vel = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)
        
        # Subscribers
        rospy.Subscriber('/error', Float64, self.callback, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/found_white', Bool, self.cbWhite, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/found_yellow', Bool, self.cbYellow, queue_size=1, buff_size=2**24)
        
        rospy.on_shutdown(self.stop_motor)
	
        self.log("Initialized!")
    
    def cbWhite(self, data):
        # Callback that stores ``found_white``

        self.found_white = data.data
        
    def cbYellow(self, data):
        # Callback that stores ``found_yellow``

        self.found_yellow = data.data
       
    def callback_config(self, config, level):
        # Callback for dynamic reconfigure

        global CRUISE_CONTROL_SPEED
        CRUISE_CONTROL_SPEED = config["cruise_control_speed"]

        global pid_z
        self.pid_z.Kp = config["Kp"]
        self.pid_z.Ki = config["Ki"]
        self.pid_z.Kd = config["Kd"]

        return config
    
        
    def callback(self, data):
        # Callback that receives the error and computes the control action 

        # Control variable
        move_cmd = Twist2DStamped()  
    	
        # PID Control
        output_z = np.clip(-self.pid_z(int(data.data)), -2.0, 2.0)

        move_cmd.v = CRUISE_CONTROL_SPEED
        if self.found_white and self.found_yellow:
            move_cmd.omega = output_z
        elif self.found_white:
            move_cmd.omega = 2.0
        elif self.found_yellow:
            move_cmd.omega = -2.0

        self.log("Control Z: {}".format(output_z))

        self.pub_cmd_vel.publish(move_cmd)

    def stop_motor(self):
        # Function to stop motors on shutdown

        msg = Twist2DStamped()
        self.pub_cmd_vel.publish(msg)

if __name__ == "__main__":
    # Global for velocity
    CRUISE_CONTROL_SPEED = None
    
    try:
        # Initialize the node
        lane_controller_node = LaneControllerNode(node_name="lane_controller_node")
        # Keep it spinning
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
