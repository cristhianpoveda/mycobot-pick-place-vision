#! /usr/bin/env python3

import rospy
from pymycobot import MyCobot
from std_srvs.srv import Empty, EmptyResponse
from pick_place_msgs.srv import SendCoords, SendCoordsResponse, SendCoord, SendCoordResponse, SendAngles, SendAnglesResponse

class ArmControl():

    def __init__(self, node_name):

        self.POSITION_TOLERANCE = rospy.get_param("~position_tolerance")
        self.CMD_STOP_TIME = rospy.get_param("~cmd_stop_time")
        self.PUMP_VENT_DELAY = rospy.get_param("~pump_vent_delay")
        
        self.mc = MyCobot("/dev/ttyACM0", 115200)

        rospy.sleep(self.CMD_STOP_TIME)

        self.arm_ready = self.mc.is_controller_connected()

        if self.arm_ready != 1: rospy.loginfo(f"Mycobot is not available")


        self.move_coords_srv = rospy.Service('~arm/coords', SendCoords, self.coords_cb)

        self.move_coord_srv = rospy.Service('~arm/coord', SendCoord, self.coord_cb)

        self.move_angles_srv = rospy.Service('~arm/angles', SendAngles, self.angles_cb)

        self.pump_on_srv = rospy.Service('~pump/on', Empty, self.pump_on_cb)

        self.pump_off_srv = rospy.Service('~pump/off', Empty, self.pump_off_cb)

        rospy.loginfo(f"Initialized: {node_name}")

        rospy.spin()
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):

        rospy.loginfo(f"Finished: {node_name}")

    def coords_cb(self, coords):

        response = SendCoordsResponse()
        response.status.data = True

        if self.mc.is_controller_connected() != 1:
            rospy.loginfo(f"Mycobot is not available")
            response.status.data = False
            return response

        coordinate_list = [coords.pose.position.x, coords.pose.position.y, coords.pose.position.z, coords.pose.orientation.x, coords.pose.orientation.y, coords.pose.orientation.z]
        self.mc.sync_send_coords(coordinate_list, coords.speed.data, coords.mode.data, coords.timeout.data)

        rospy.sleep(1) # 1s

        if self.mc.is_in_position(coordinate_list, 1) != 1:
            response.status.data = False
        
        return response
    
    def coord_cb(self, coord):

        response = SendCoordResponse()
        response.status.data = True

        if self.mc.is_controller_connected() != 1:
            rospy.loginfo(f"Mycobot is not available")
            response.status.data = False
            return response

        self.mc.send_coord(coord.id.data, coord.coord.data, coord.speed.data)

        rospy.sleep(coord.delay.data)

        current_coords = self.mc.get_coords()

        if abs(current_coords[coord.id.data] - coord.coord.data) > self.POSITION_TOLERANCE:
            response.status.data = False

        return response
    
    def angles_cb(self, angles):

        response = SendAnglesResponse()
        response.status.data = True

        if self.mc.is_controller_connected() != 1:
            rospy.loginfo(f"Mycobot is not available")
            response.status.data = False
            return response

        self.mc.sync_send_angles(angles.anlges.data, angles.speed.data, angles.timeout.data)

        rospy.sleep(1)

        if self.mc.is_in_position(angles.angles.data, 0) != 1:
            response.status.data = False
    
    def pump_on_cb(self, req=None):

        response = EmptyResponse()
        
        if self.mc.is_controller_connected() != 1:
            rospy.loginfo(f"Mycobot is not available")
            return response

        self.mc.set_basic_output(5, 0)
        rospy.sleep(self.CMD_STOP_TIME)

        return response

    def pump_off_cb(self, req=None):
            
        response = EmptyResponse()

        if self.mc.is_controller_connected() != 1:
            rospy.loginfo(f"Mycobot is not available")
            return response
        
        self.mc.set_basic_output(5, 1)
        rospy.sleep(self.CMD_STOP_TIME)
        self.mc.set_basic_output(2, 0)
        rospy.sleep(self.PUMP_VENT_DELAY)
        self.mc.set_basic_output(2, 1)
        rospy.sleep(self.CMD_STOP_TIME)

        return response

if __name__ == '__main__':

    node_name = 'arm_control_node'

    rospy.init_node(node_name)

    try:

        node = ArmControl(node_name)

    except rospy.ROSInterruptException: pass