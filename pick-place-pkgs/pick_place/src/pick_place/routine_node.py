#! /usr/bin/env python3

import rospy
import math
from pick_place_msgs.srv import VerifyPicking, VerifyPickingRequest, GetJoints, GetJointsRequest, DetectBottles, SendCoords, SendCoordsRequest, SendCoord, SendCoordRequest, SendAngles, SendAnglesRequest, SendAngle, SendAngleRequest
import actionlib
from pick_place_msgs.msg import PickPlaceAction, PickPlaceResult, PickPlaceFeedback
from std_srvs.srv import Empty, EmptyRequest

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

        pump_off_req = EmptyRequest()

        try:
            rospy.wait_for_service('/mycobot/arm_control_node/pump/off', timeout=1)

            try:
                pump_off_srv = rospy.ServiceProxy("/mycobot/arm_control_node/pump/off", Empty)
                pump_off_srv(pump_off_req)

            except rospy.ServiceException as e:
                rospy.loginfo("Request suction pump off failed:\n%s"%e)
                self.fail_msg()
                return
            
            angles_req = SendAnglesRequest()
            angles_req.angles.data = [0, 0, 0, 0, 0, 0]
            angles_req.speed.data = 30
            angles_req.timeout.data = 7

            try:
                rospy.wait_for_service('/mycobot/arm_control_node/arm/angles')
                try:
                    angles_srv = rospy.ServiceProxy('/mycobot/arm_control_node/arm/angles', SendAngles)
                    angles_srv(angles_req)

                except rospy.ServiceException as e:
                    rospy.loginfo("Request move angles failed: %s"%e)
                    
            except rospy.ROSException as e:
                rospy.loginfo(f"Mycobot is not available:\n{e}")
            
        except rospy.ROSException as e:
            rospy.loginfo(f"Mycobot is not available:\n{e}")

        rospy.loginfo(f"Finished: {node_name}")

    def fail_msg(self):
        self._result.result.data = False
        rospy.loginfo('Pick and place: Failed')
        self._as.set_aborted(self._result)

    def feedback_msg(self, msg):

        self._feedback.feedback.data = msg
        self._as.publish_feedback(self._feedback)

    def get_picking_angle(self, bottle_a, J1):

        a_cam = bottle_a
        if a_cam == 0:
            a_arm = 180
        else:
            a_arm = - (a_cam / abs(a_cam)) * (180 - abs(a_cam))
        if abs(a_arm) > 90:
            a_perpendicular = (a_arm /abs(a_arm)) * (abs(a_arm) - 90)
        elif a_arm == 0:
            a_perpendicular = -90
        else:
            a_perpendicular = (a_arm /abs(a_arm)) * (abs(a_arm) + 90)

        if a_perpendicular > 0:

            pick_a = - (180 - a_perpendicular)

        else: pick_a = a_perpendicular

        a_j6 = J1 - pick_a

        if abs(a_j6) > 90:
            a_j6t = -(a_j6/abs(a_j6)) * (180 - abs(a_j6))
        else: a_j6t = a_j6

        if abs(a_j6t) > 90:

            rospy.loginfo(f"No feasible rotation found")
            a_j6t = 0

        return a_j6t
    

    def get_joint(self, bottle, a_J1):

        if bottle > 0:
            rot = - (180 - bottle)
        else:
            rot = bottle

        relative = abs(a_J1 - rot)

        if 45 < relative < 135:
            joint = 5
            angle = 1.2

        else:
            joint = 4
            angle = 1.8

        return joint, angle
    

    def routine_cb(self, req=None):

        rospy.loginfo('Executing action: Picking selected bottle')

        success = True

        # INIT HOME POSITION

        self.feedback_msg(f'Moving to home position')

        angles_req = SendAnglesRequest()
        angles_req.angles.data = [0, 0, 0, 0, 0, 0]
        angles_req.speed.data = 30
        angles_req.timeout.data = 7

        try:
            rospy.wait_for_service('/mycobot/arm_control_node/arm/angles', timeout=3)
        except rospy.ROSException as e:
            rospy.loginfo(f"Send angles service unavailable:\n{e}")
            self.fail_msg()
            return

        try:
            angles_srv = rospy.ServiceProxy('/mycobot/arm_control_node/arm/angles', SendAngles)
            angles_response = angles_srv(angles_req)

        except rospy.ServiceException as e:
            rospy.loginfo(f"Request move angles failed:\n{e}")
            self.fail_msg()
            return
        

        # SELECT BOTTLE TO PICK

        self.feedback_msg(f'Requesting bottle selection')

        detected_valid = False

        try:
            rospy.wait_for_service('/mycobot/detection_node/locate/bottle', timeout=3)
        except rospy.ROSException as e:
            rospy.loginfo(f"Detection service unavailable:\n{e}")
            self.fail_msg()
            return
        
        try:
            detection_srv = rospy.ServiceProxy('/mycobot/detection_node/locate/bottle', DetectBottles)
            selected_bottle = detection_srv()

            if selected_bottle.result.data == 'success':
                rospy.loginfo(f"Selected bottle coords [m]: x: {round(selected_bottle.pose.position.x,3)}, y: {round(selected_bottle.pose.position.y,2)}, z: {round(selected_bottle.pose.position.z,2)}")
                detected_valid = True

        except rospy.ServiceException as e:
            rospy.loginfo(f"Request detection failed:\n{e}")
            self.fail_msg()
            return
        
        if not detected_valid:
            rospy.loginfo(f'No viable bottle to pick: {selected_bottle.result.data}')
            self.fail_msg()
            return
        

        # PICKING X, Y, Z = 180

        coords_req = SendCoordsRequest()
        picking_x = 1000 * selected_bottle.pose.position.x - 10
        picking_y = 1000 * selected_bottle.pose.position.y

        picking_distance = math.sqrt(picking_x**2 + picking_y**2)

        if picking_distance > 240:

            rospy.loginfo(f"Picking coordinates outside of arm's workspace")
            self.fail_msg()
            return

        rospy.loginfo(f'Moving to picking coords [mm]: x: {round(picking_x,0)}, y: {round(picking_y, 0)}')
        coords_req.pose.position.x = picking_x
        coords_req.pose.position.y = picking_y
        coords_req.pose.position.z = 180
        coords_req.pose.orientation.x = -180.0
        y_rot = int(7 * (picking_y + 220) / 100)
        coords_req.pose.orientation.y = y_rot
        rospy.loginfo(f'picking tilt angle: {y_rot}')
        coords_req.pose.orientation.z = -90.0
        coords_req.speed.data = 10
        coords_req.mode.data = 1
        coords_req.timeout.data = 7

        try:
            rospy.wait_for_service('/mycobot/arm_control_node/arm/coords', timeout=3)
        except rospy.ROSException as e:
            rospy.loginfo(f"Send coords service unavailable:\n{e}")
            self.fail_msg()
            return
        
        try:
            coords_srv = rospy.ServiceProxy('/mycobot/arm_control_node/arm/coords', SendCoords)
            coords_response = coords_srv(coords_req)
            rospy.loginfo(f"Current pos: {coords_response.current_pos.data}")

        except rospy.ServiceException as e:
            rospy.loginfo(f"Move picking position failed:\n{e}")
            self.fail_msg()
            return

        
        # ROTATE PUMP HEAD FOR PERPENDICULAR PICKING

        joints_req = GetJointsRequest()
        joints_req.type.data = 0

        try:
            rospy.wait_for_service('/mycobot/arm_control_node/arm/get/joints', timeout=3)
        except rospy.ROSException as e:
            rospy.loginfo(f"Get joints service unavailable:\n{e}")
            self.fail_msg()
            return

        try:
            joints_srv = rospy.ServiceProxy('/mycobot/arm_control_node/arm/get/joints', GetJoints)
            joints_response = joints_srv(joints_req)
            a_J1 = joints_response.values.data[0]
            rospy.loginfo(f"Current J1: {round(a_J1,1)}")

        except rospy.ServiceException as e:
            rospy.loginfo(f"Get Joint values failed:\n{e}")
            self.fail_msg()
            return

        a_j6t = self.get_picking_angle(selected_bottle.pose.orientation.z, a_J1)

        angle_req = SendAngleRequest()
        angle_req.id.data = 6
        angle_req.angle.data = a_j6t
        angle_req.speed.data = 20

        rospy.loginfo(f"Target tool angle: {round(a_j6t,1)}")

        try:
            rospy.wait_for_service('/mycobot/arm_control_node/arm/angle', timeout=3)
        except rospy.ROSException as e:
            rospy.loginfo(f"Send angle service unavailable:\n{e}")
            self.fail_msg()
            return

        try:
            angle_srv = rospy.ServiceProxy('/mycobot/arm_control_node/arm/angle', SendAngle)
            angle_response = angle_srv(angle_req)

        except rospy.ServiceException as e:
            rospy.loginfo(f"Request move angle failed:\n{e}")
            self.fail_msg()
            return

        # TURN ON PUMP

        pump_req = EmptyRequest()

        try:
            rospy.wait_for_service('/mycobot/arm_control_node/pump/on', timeout=3)
        except rospy.ROSException as e:
            rospy.loginfo(f"Turn on pump service unavailable:\n{e}")
            self.fail_msg()
            return

        try:
            pump_on_srv = rospy.ServiceProxy('/mycobot/arm_control_node/pump/on', Empty)
            pump_on_srv(pump_req)

        except rospy.ServiceException as e:
            rospy.loginfo(f"Request suction pump on failed:\n{e}")
            self.fail_msg()
            return


        # LOWER TOOL Z = Zd + 73.1

        coord_req = SendCoordRequest()
        coord_req.id.data = 3
        tool_height = 1000 * selected_bottle.pose.position.z + 73.1
        rospy.loginfo(f'Tool picking height [mm]: {round(tool_height,1)}')
        coord_req.coord.data = tool_height
        coord_req.speed.data = 10
        coord_req.delay.data = 4

        try:
            rospy.wait_for_service('/mycobot/arm_control_node/arm/coord', timeout=3)
        except rospy.ROSException as e:
            rospy.loginfo(f"Send coord service unavailable:\n{e}")
            self.fail_msg()
            return

        try:
            coord_srv = rospy.ServiceProxy('/mycobot/arm_control_node/arm/coord', SendCoord)
            coord_response = coord_srv(coord_req)

        except rospy.ServiceException as e:
            rospy.loginfo(f"Request lowering tool failed:\n{e}")
            self.fail_msg()
            return

        
        # MOVE TO VERIFICATION POSITION

        angles_req.angles.data = [-90, 0, -90, 90, 0, 0]
        angles_req.speed.data = 20

        try:
            rospy.wait_for_service('/mycobot/arm_control_node/arm/angles', timeout=3)
        except rospy.ROSException as e:
            rospy.loginfo(f"Send angles service unavailable:\n{e}")
            self.fail_msg()
            return
        
        try:
            angles_response = angles_srv(angles_req)

        except rospy.ServiceException as e:
            rospy.loginfo(f"Request moving to verification position failed:\n{e}")
            self.fail_msg()
            return
        
        # VERIFY BOTTLE W ORIENTATION
        # DETECTION (create srv)

        verify_req = VerifyPickingRequest()

        try:
            rospy.wait_for_service('/mycobot/detection_node/verify/picking', timeout=3)
        except rospy.ROSException as e:
            rospy.loginfo(f"Verify picking service unavailable:\n{e}")
            self.fail_msg()
            return
        
        try:
            verify_srv = rospy.ServiceProxy('/mycobot/detection_node/verify/picking', VerifyPicking)
            verify_response = verify_srv(verify_req)
            rospy.loginfo(f"Picked: {verify_response.pick.data}, Orientation: {verify_response.orientation.data}")

        except rospy.ServiceException as e:
            rospy.loginfo(f"Request bottle picking verification failed:\n{e}")
            self.fail_msg()
            return
        


        # MOVE TO PLACING POSITION. TOOL ROT (+-90)
        # ANGLES

        if verify_response.pick.data:

            if verify_response.orientation.data == 0:
                placing_rot = 90

            else:
                placing_rot = -90
            
            angles_req.angles.data = [-90, -70, -57, 127, 0, placing_rot]

            rospy.wait_for_service('/mycobot/arm_control_node/arm/angles')
            try:
                angles_response = angles_srv(angles_req)

            except rospy.ServiceException as e:
                rospy.loginfo(f"Request moving to verification position failed:\n{e}")
                self.fail_msg()
                return
            
        else:

            rospy.loginfo(f"Bottle not picked")
            self.fail_msg()
            success = False


        # RELEASE BOTTLE
        # PUMP OFF

        rospy.wait_for_service('/mycobot/arm_control_node/pump/off')
        try:
            pump_off_srv = rospy.ServiceProxy("/mycobot/arm_control_node/pump/off", Empty)
            pump_off_srv(pump_req)

        except rospy.ServiceException as e:
            rospy.loginfo("Request suction pump off failed:\n%s"%e)
            self.fail_msg()
            return
        

        # END ACTION

        if success:

            self._result.result.data = True
            rospy.loginfo('Pick and place: Succeeded')
            self._as.set_succeeded(self._result)

if __name__ == '__main__':

    node_name = 'routine_node'

    rospy.init_node(node_name)

    try:

        node = PickPlaceRoutine(node_name)

    except rospy.ROSInterruptException: pass