#! /usr/bin/env python

import rospy
import actionlib

import actionlib_tutorials.msg

class FibonacciActionServer():

    _feedback = actionlib_tutorials.msg.FibonacciFeedback()
    _result = actionlib_tutorials.msg.FibonacciResult()

    def __init__(self):
        self._action_name = '/fibonacci'
        self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        rospy.loginfo("Action Server Started Successfully...You can send Goals to me...")

    def execute_cb(self, goal):

        r = rospy.Rate(1)
        flag = True

        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)

        rospy.loginfo("Execution started for order {} with {}, {}".format(goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))

        for i in range(1, goal.order):

            if self._as.is_preempt_requested():
                rospy.loginfo('Preempt Requested for {}'.format(self._action_name))
                self._as.set_preempted()
                flag = False

            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])

            self._as.publish_feedback(self._feedback)

            r.sleep()

        if flag:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('Goal succeeded for {}'.format(self._action_name))
            self._as.set_succeeded(self._result)

def main():
    rospy.init_node('node_simple_fibonacci_action_server')
    server = FibonacciActionServer()
    rospy.spin()

if __name__ == '__main__':
    main()