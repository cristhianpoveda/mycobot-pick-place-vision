#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
from pick_place_msgs.srv import DetectBottles, DetectBottlesRequest, DetectBottlesResponse, SendCoords, SendCoordsRequest, SendCoordsResponse, SendCoord, SendCoordRequest, SendCoordResponse, SendAngles, SendAnglesRequest, SendAnglesResponse, SendAngle, SendAngleRequest, SendAngleResponse

class PickPlaceRoutine():

    def __init__(self, node_name):

        self.POSITION_TOLERANCE = rospy.get_param("~position_tolerance")
        self.CMD_STOP_TIME = rospy.get_param("~cmd_stop_time")
        self.PUMP_VENT_DELAY = rospy.get_param("~pump_vent_delay")

        self.pump_off_srv = rospy.Service('~start/routine', Empty, self.routine_cb)

        rospy.loginfo(f"Initialized: {node_name}")

        rospy.spin()
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):

        angles_req = SendAnglesRequest()
        angles_req.angles.data = [0, 0, 0, 0, 0, 0]
        angles_req.speed.data = 30
        angles_req.timeout.data = 7

        rospy.wait_for_service('/mycobot/arm_control_node/arm/angles')
        try:
            angles_srv = rospy.ServiceProxy('/mycobot/arm_control_node/arm/angles', SendAngles)
            angles_srv(angles_req)

        except rospy.ServiceException as e:
            rospy.loginfo("Request move angles failed: %s"%e)

        rospy.loginfo(f"Finished: {node_name}")

    def routine_cb(self, req=None):
            
        response = EmptyResponse()

        # INIT HOME POSITION

        angles_req = SendAnglesRequest()
        angles_req.angles.data = [0, 0, 0, 0, 0, 0]
        angles_req.speed.data = 30
        angles_req.timeout.data = 7

        rospy.wait_for_service('/mycobot/arm_control_node/arm/angles')
        try:
            angles_srv = rospy.ServiceProxy('/mycobot/arm_control_node/arm/angles', SendAngles)
            angles_srv(angles_req)

        except rospy.ServiceException as e:
            rospy.loginfo("Request move angles failed: %s"%e)
            return response
        

        # SELECT BOTTLE TO PICK

        detected_valid = False

        for i in range(3):

            rospy.wait_for_service('/mycobot/detection_node/locate/bottle')
            try:
                detection_srv = rospy.ServiceProxy('/mycobot/detection_node/locate/bottle', DetectBottles)
                selected_bottle = detection_srv()

                if selected_bottle.result.data == 'success':
                    detected_valid = True
                    break

            except rospy.ServiceException as e:
                rospy.loginfo("Request move angles failed: %s"%e)
                return response
        
        if not detected_valid:
            rospy.loginfo(f'Detection failed: {selected_bottle.result.data}')
            return response
        

        # BOX CENTRE
        
        coords_req = SendCoordsRequest()
        coords_req.pose.position.x = -10.0
        coords_req.pose.position.y = -220.0
        coords_req.pose.position.z = 220.0
        coords_req.pose.orientation.x = -180.0
        coords_req.pose.orientation.y = 0.0
        coords_req.pose.orientation.z = -90.0
        coords_req.speed.data = 5
        coords_req.mode.data = 0
        coords_req.timeout.data = 7
        
        rospy.wait_for_service('/mycobot/arm_control_node/arm/coords')
        try:
            coords_srv = rospy.ServiceProxy('/mycobot/arm_control_node/arm/coords', SendCoords)
            coords_srv(coords_req)

        except rospy.ServiceException as e:
            rospy.loginfo("Move to home position failed: %s"%e)
            return response
        

        # PICKING X, Y, Z = 200

        coords_req = SendCoordsRequest()
        coords_req.pose.position.x = 1000 * selected_bottle.pose.position.x
        coords_req.pose.position.y = 1000 * selected_bottle.pose.position.y
        coords_req.pose.position.z = 200
        coords_req.pose.orientation.x = -180.0
        coords_req.pose.orientation.y = 0.0
        coords_req.pose.orientation.z = -90.0
        coords_req.speed.data = 5
        coords_req.mode.data = 0
        coords_req.timeout.data = 7
        
        rospy.wait_for_service('/mycobot/arm_control_node/arm/coords')
        try:
            coords_srv(coords_req)

        except rospy.ServiceException as e:
            rospy.loginfo("Move to home position failed: %s"%e)
            return response
        

        # # PUMP HOSE TO 0°

        # angle_req = SendAngleRequest()
        # angle_req.id.data = 6
        # angle_req.angle.data = 0
        # angle_req.speed = 30

        # rospy.wait_for_service('/mycobot/arm_control_node/arm/angle')
        # try:
        #     angle_srv = rospy.ServiceProxy('/mycobot/arm_control_node/arm/angle', SendAngle)
        #     angle_srv(angle_req)

        # except rospy.ServiceException as e:
        #     rospy.loginfo("Request move angles failed: %s"%e)
        #     return response
        

        
        
        # coords_req.pose.position.x = -10.0
        # coords_req.pose.position.y = -220.0
        # coords_req.pose.position.z = 200.0
        # coords_req.pose.orientation.x = -180.0
        # coords_req.pose.orientation.y = 0.0
        # coords_req.pose.orientation.z = -90.0

        # rospy.wait_for_service('/mycobot/arm_control_node/arm/coords')
        # try:
        #     coords_srv = rospy.ServiceProxy('/mycobot/arm_control_node/arm/coords', SendCoords)
        #     coords_srv(coords_req)

        # except rospy.ServiceException as e:
        #     rospy.loginfo("Request move angles failed: %s"%e)
        #     return response
        
        return response

if __name__ == '__main__':

    node_name = 'routine_node'

    rospy.init_node(node_name)

    try:

        node = PickPlaceRoutine(node_name)

    except rospy.ROSInterruptException: pass