#! /usr/bin/env python3

import rospy
import actionlib
from pick_place_msgs.msg import PickPlaceAction, PickPlaceGoal
from std_srvs.srv import Empty, EmptyResponse

class SimulationClient():

    def __init__(self, node_name):

        self.work = False

        self.pick_place_request = PickPlaceGoal()

        self.routine_client = actionlib.SimpleActionClient('/mycobot/pick_place', PickPlaceAction)

        self.routine_client.wait_for_server()

        self.goal = PickPlaceGoal()

        self.work_srv = rospy.Service('~start/stop', Empty, self.work_state_cb)
        self.empty_response = EmptyResponse()

        rospy.loginfo("Initialized: %s", node_name)

        while not rospy.is_shutdown():

            self.action_request()

    def action_request(self):

        if self.work:

            self.routine_client.send_goal(self.goal)

            self.routine_client.wait_for_result()

            action_result = self.routine_client.get_result()

            rospy.loginfo(f"Pick place action result: {action_result.result.data}")

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