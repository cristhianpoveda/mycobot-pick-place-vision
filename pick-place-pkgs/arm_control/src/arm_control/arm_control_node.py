#! /usr/bin/env python3

import time
import rospy
from pymycobot import MyCobot
from std_srvs.srv import Empty, EmptyReponse
from pick_place_msgs.srv import MoveArm, MoveArmResponse

class ArmControl():

    def __init__(self, node_name):

        self.mc = MyCobot("/dev/ttyACM0", 115200)

        self.MOVE_VEL = rospy.get_param("~move_vel")
        self.MODE = rospy.get_param("~mode")
        self.POSITION_TOLERANCE = rospy.get_param("~position_tolerance")
        self.RETURN_VEL = rospy.get_param("~return_vel")
        self.PUMP_STOP_TIME = rospy.get_param("~pump_stop_time")
        self.PUMP_VENT_DELAY = rospy.get_param("~pump_vent_delay")

        self.movement_srv = rospy.Service('~arm/move', MoveArm, self.movement_cb)

        self.pump_on_srv = rospy.Service('~pump/on', Empty, self.pump_on_cb)

        self.pump_off_srv = rospy.Service('~pump/off', Empty, self.pump_off_cb)

        rospy.loginfo(f"Initialized: {node_name}")

        rospy.spin()
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):

        rospy.loginfo(f"Finished: {node_name}")

    def movement_cb(self, pose):

        response = MoveArmResponse()
        response.status.data = True

        coordinate_list = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z]
        self.mc.send_coords(coordinate_list, self.MOVE_VEL, self.MODE)

        current_coords = self.mc.get_coords()

        for idx, coord in enumerate(current_coords):

            if abs(coord - coordinate_list[idx]) > self.POSITION_TOLERANCE:
                response.status.data = False
                self.mc.send_angles([0,0,0,0,0,0], self.RETURN_VEL)
                break
        
        return response
    
    def pump_on_cb(self, req=None):

        self.mc.set_basic_output(5, 0)
        rospy.sleep(self.PUMP_STOP_TIME)

        response = EmptyReponse()
        return response

    def pump_off_cb(self, req=None):
            
        self.mc.set_basic_output(5, 1)
        rospy.sleep(self.PUMP_STOP_TIME)
        self.mc.set_basic_output(2, 0)
        rospy.sleep(self.PUMP_VENT_DELAY)
        self.mc.set_basic_output(2, 1)
        rospy.sleep(self.PUMP_STOP_TIME)

        response = EmptyReponse()
        return response

if __name__ == '__main__':

    node_name = 'arm_control_node'

    rospy.init_node(node_name)

    try:

        node = ArmControl(node_name)

    except rospy.ROSInterruptException: pass