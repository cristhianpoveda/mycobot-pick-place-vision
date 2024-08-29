#! /usr/bin/env python3

import rospy
from pick_place_msgs.srv import DetectBottles, DetectBottlesResponse, SendCoords, SendCoordsRequest, SendCoordsResponse, SendCoord, SendCoordRequest, SendCoordResponse, SendAngles, SendAnglesRequest, SendAnglesResponse, SendAngle, SendAngleRequest, SendAngleResponse
import actionlib
from pick_place_msgs.msg import PickPlaceAction, PickPlaceResult, PickPlaceFeedback

class PickPlaceRoutine():

    def __init__(self, node_name):

        self._feedback = PickPlaceFeedback()
        self._result = PickPlaceResult()

        self._as = actionlib.SimpleActionServer('pick_place', PickPlaceAction, execute_cb=self.routine_cb, auto_start = False)
        self._as.start()

        rospy.loginfo(f"Initialized: {node_name}")

        rospy.on_shutdown(self.shutdown_hook)

        rospy.spin()
        

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

    def fail_msg(self):
        self._result.result.data = False
        rospy.loginfo('Pick and place: Failed')
        self._as.set_aborted(self._result)

    def feedback_msg(self, msg):

        self._feedback.feedback.data = msg
        self._as.publish_feedback(self._feedback)

    def routine_cb(self, req=None):

        rospy.loginfo('Executing action: Picking selected bottles')

        # INIT HOME POSITION

        self.feedback_msg(f'Moving to home position')

        angles_req = SendAnglesRequest()
        angles_req.angles.data = [0, 0, 0, 0, 0, 0]
        angles_req.speed.data = 30
        angles_req.timeout.data = 7

        rospy.wait_for_service('/mycobot/arm_control_node/arm/angles')
        try:
            angles_srv = rospy.ServiceProxy('/mycobot/arm_control_node/arm/angles', SendAngles)
            angles_response = angles_srv(angles_req)

        except rospy.ServiceException as e:
            rospy.loginfo("Request move angles failed: %s"%e)
            self.fail_msg()
        

        # SELECT BOTTLE TO PICK

        self.feedback_msg(f'Requesting bottle selection')

        detected_valid = False

        for i in range(3):

            rospy.wait_for_service('/mycobot/detection_node/locate/bottle')
            try:
                detection_srv = rospy.ServiceProxy('/mycobot/detection_node/locate/bottle', DetectBottles)
                selected_bottle = detection_srv()

                if selected_bottle.result.data == 'success':
                    rospy.loginfo(f"Picking coordinates: x: {selected_bottle.pose.position.x}, y: {selected_bottle.pose.position.y}")
                    detected_valid = True
                    break

            except rospy.ServiceException as e:
                rospy.loginfo("Request detection failed: %s"%e)
                self.fail_msg()
        
        if not detected_valid:
            rospy.loginfo(f'No viable bottle to pick: {selected_bottle.result.data}')
            self.fail_msg()
        

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
            coords_response = coords_srv(coords_req)
            rospy.loginfo(f"Current pos: {coords_response.current_pos.data}")

        except rospy.ServiceException as e:
            rospy.loginfo("Move to box centre failed: %s"%e)
            self.fail_msg()
        

        # PICKING X, Y, Z = 200

        coords_req = SendCoordsRequest()
        picking_x = 1000 * selected_bottle.pose.position.x - 10
        picking_y = 1000 * selected_bottle.pose.position.y
        rospy.loginfo(f'Picking coords: x: {picking_x}, y: {picking_y}')
        coords_req.pose.position.x = picking_x
        coords_req.pose.position.y = picking_y
        coords_req.pose.position.z = 200
        coords_req.pose.orientation.x = -180.0
        coords_req.pose.orientation.y = 0.0
        coords_req.pose.orientation.z = -90.0
        coords_req.speed.data = 5
        coords_req.mode.data = 1
        coords_req.timeout.data = 7
        
        rospy.wait_for_service('/mycobot/arm_control_node/arm/coords')
        try:
            coords_response = coords_srv(coords_req)
            rospy.loginfo(f"Current pos: {coords_response.current_pos.data}")

        except rospy.ServiceException as e:
            rospy.loginfo("Move picking position failed: %s"%e)
            self.fail_msg()
        

        self._result.result.data = True
        rospy.loginfo('Pick and place: Succeeded')
        self._as.set_succeeded(self._result)

if __name__ == '__main__':

    node_name = 'routine_node'

    rospy.init_node(node_name)

    try:

        node = PickPlaceRoutine(node_name)

    except rospy.ROSInterruptException: pass