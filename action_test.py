#!/usr/bin/env python3

import time


import actionlib
import actionlib.action_server
import actionlib.action_client
from actionlib.server_goal_handle import ServerGoalHandle
import rospy


from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult
from control_msgs.msg import FollowJointTrajectoryFeedback, FollowJointTrajectoryResult, FollowJointTrajectoryGoal

class ActionTest():
    def __init__(self):

        self.action_test_srv = actionlib.action_server.ActionServer("actiontest", FollowJointTrajectoryAction, self.action_test_goal_cb, self.action_test_cancel_cb, auto_start=False)
        self.action_fb_msg = FollowJointTrajectoryFeedback()
        self.action_test_srv.start()
        self.action_active = False
        self.active_goal:ServerGoalHandle = None
        # I'm caching this so that the active goal object isnt cleaned up before it finishes posting results
        # It wasnt during testing, but I'm paranoid.
        self.previous_goal:ServerGoalHandle = None  
        print("server up")


    def action_test_goal_cb(self, goal:ServerGoalHandle):
        """
            accepts waypoints in js, 
            plans and executes in joint space through them
        """
        print("goal callback")
        print(type(goal))
        print(type(goal.goal))
        print(type(goal.goal.goal))
        # print("====")
        # print(dir(goal))
        # print("====")
        # print(dir(goal.goal))
        # print("====")
        # print(dir(goal.goal.goal))
        
        # to rejhect an incoming goal because of whatever, populate result with reason and set rejected
        # goal.set_rejected(FollowJointTrajectoryResult(), "I reject this goal")
        # return

        self.active_goal = goal
        self.previous_goal = None
        print(goal.goal.goal_id)
        traj = goal.goal.goal.trajectory
        self.action_active = True
        self.active_goal.set_accepted("accepted by goal_cb")
        print("goal accepted")


    def action_test_cancel_cb(self, msg:ServerGoalHandle):
        """
            receives an action cancel request and cancels execution of the current trajectory
        """
        print("cancel CB")
        self.action_active = False
        self.active_goal.set_cancel_requested()
        # print(type(msg))
        # print(dir(msg))


        if self.active_goal.get_goal_id() == msg.get_goal_id():
            print("incoming cancel request matches current goal id")

        result = FollowJointTrajectoryResult()
        result.error_code = result.INVALID_GOAL
        self.active_goal.set_canceled(result, "cancel was called")

        self.previous_goal = self.active_goal
        self.active_goal = None
        self.action_active = False
        print("cancelled")


    def send_action_feedback(self):
        
        self.action_fb_msg.header.stamp = rospy.Time.now()
        self.action_fb_msg.header.seq += 1
        self.active_goal.publish_feedback(self.action_fb_msg)


    def send_action_result(self, res:str):
        " this terminates the action"

        result = FollowJointTrajectoryResult()
        

        if res == "aborted":
            result.error_code = result.INVALID_JOINTS
            self.active_goal.set_aborted(result, "set aborted")
        elif res== "succeeded":
            result.error_code = result.SUCCESSFUL
            self.active_goal.set_succeeded(result, "yay!")
        else:
            print(f"cannot handle {res}")

        self.previous_goal = self.active_goal
        self.active_goal = None



from actionlib.action_client import ClientGoalHandle

def fb_cb(msg:ClientGoalHandle, msg2:FollowJointTrajectoryFeedback):
    print('fb cb')
    # print(type(msg2))
    # print(dir(msg2))

client_side_goal_done = False

def trans_cb(msg:ClientGoalHandle):
    global client_side_goal_done
    """  
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5
    PREEMPTING = 6
    RECALLING = 7
    RECALLED = 8
    LOST = 9
    """

    print('transition cb')
    status = msg.get_goal_status()
    print(f"goal status transition to {status}")
    if status not in [0, 1]:
        client_side_goal_done = True


if __name__ == "__main__":

    rospy.init_node("action_test", anonymous=True)

    action_server_tester = ActionTest()

    ac = actionlib.action_client.ActionClient("actiontest", FollowJointTrajectoryAction)
    print("waiting for server")
    ac.wait_for_action_server_to_start()
    print("server seems up")
    newgoal = FollowJointTrajectoryGoal()
    newgoal.trajectory.header.stamp = rospy.Time.now()

    print("sending goal")
    # callbacks will not be called if the client_gh returned object gets cleaned up
    client_gh = ac.send_goal(newgoal, trans_cb, fb_cb)

    rate = rospy.Rate(5)
    loops=0
    while(not rospy.is_shutdown()):
        
        if action_server_tester.active_goal:
            action_server_tester.send_action_feedback()

        if loops>10 and action_server_tester.active_goal:
            print("setting result")
            action_server_tester.send_action_result("succeeded")
            # print("requesting cancel")
            # ac.cancel_all_goals()

        if client_side_goal_done:
            print("no goal remains")
            break

        rate.sleep()
        loops+=1
    time.sleep(1)