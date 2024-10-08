#! /usr/bin/env python3

import rospy
import actionlib
from pick_place_msgs.msg import PickPlaceAction, PickPlaceGoal
from pick_place_msgs.srv import SendAngles, SendAnglesRequest
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest

class SimulationClient():

    def __init__(self, node_name):

        self.work = False

        self.failure = False

        self.pick_place_request = PickPlaceGoal()

        self.routine_client = actionlib.SimpleActionClient('/mycobot/pick_place', PickPlaceAction)

        self.routine_client.wait_for_server()

        self.goal = PickPlaceGoal()

        self.work_srv = rospy.Service('~start/stop', Empty, self.work_state_cb)
        self.abort_srv = rospy.Service('~abort', Empty, self.abort_cb)
        self.empty_response = EmptyResponse()

        rospy.loginfo("Initialized: %s", node_name)

        while not rospy.is_shutdown():

            self.action_request()

    def stop_operation(self, msg):

        rospy.loginfo(f"ABORTING OPERATION: {msg}")

        self.routine_client.cancel_all_goals()

        pump_req = EmptyRequest()

        try:
            rospy.wait_for_service('/mycobot/arm_control_node/pump/off', timeout=3)
        except rospy.ROSException as e:
            rospy.loginfo(f"Turn off pump service unavailable:\n{e}")
            return

        try:
            pump_off_srv = rospy.ServiceProxy('/mycobot/arm_control_node/pump/off', Empty)
            pump_off_srv(pump_req)

        except rospy.ServiceException as e:
            rospy.loginfo(f"Request suction pump off failed:\n{e}")
            return
        
    def abort_cb(self, req=None):
        
        self.failure = True
        self.stop_operation("Requested by user")
        return self.empty_response

    def action_request(self):

        if self.work and not self.failure:

            self.routine_client.send_goal(self.goal)

            self.routine_client.wait_for_result()

            action_result = self.routine_client.get_result()

            rospy.loginfo(f"Pick place action result: {action_result.result.data}")

            if action_result.failure.data: 

                self.failure = True
                self.stop_operation("Undefined failure")

        else:
            pass

    def work_state_cb(self, req=None):

        if self.work:
            self.work = False

        else:
            self.work = True

        return self.empty_response


if __name__ == '__main__':

    node_name = 'routine_client_node'

    rospy.init_node(node_name)

    try:

        node = SimulationClient(node_name)

    except rospy.ROSInterruptException: pass