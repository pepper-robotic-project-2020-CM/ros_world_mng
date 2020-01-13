#!/usr/bin/env python
__author__ = 'jsaraydaryan'
import rospy
from rospy_message_converter import message_converter, json_message_converter
from robocup_msgs.msg import InterestPoint #, Order, OrderInterest
from geometry_msgs.msg import Pose,PoseStamped
from map_manager.srv import *
from navigation_manager.msg import NavMngGoal, NavMngAction
import actionlib
from actionlib_msgs.msg import GoalStatus


class Mt:
    # CONFIG_PATH="/home/astro/ros_ws/src/laptop-youbot/cpe_robotcup/config/interest-points/"
    CONFIG_PATH=""
    _index_label=0


    def __init__(self,conf_path):
        self.CONFIG_PATH=conf_path
        self.configure()
        print

    ####
    #  Set the initial configuration of the current Node
    ####
    def configure(self):
        self._simpleGoal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.simpleGoalcallback)
        self._simpleGoal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self._doorGoal_sub = rospy.Subscriber("/door_goal", PoseStamped, self.doorGoalcallback)

        rospy.loginfo("Connecting to navigation_manager action server ... ")
        self._actionNavMng_server = actionlib.SimpleActionClient('navigation_manager', NavMngAction)
        finished1 =self._actionNavMng_server.wait_for_server(timeout = rospy.Duration(20.0))
        if finished1:
            rospy.loginfo("navigation_manager Connected")
        else:
            rospy.logwarn("Unable to connect to navigation_manager action server")

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def sendNavOrderAction(self,action,mode,itP,timeout):
        try:
            goal = NavMngGoal()
            goal.action=action
            goal.itP=itP
            goal.navstrategy=mode
            rospy.loginfo("### NAV ACTION PENDING : %s",str(goal).replace('\n',', '))
            self._actionNavMng_server.send_goal(goal)
            self._actionNavMng_server.wait_for_result(rospy.Duration.from_sec(timeout))
            state=self._actionNavMng_server.get_state()
            if state == GoalStatus.ABORTED:
                rospy.logwarn("###### NAV ACTION END , State: %s",self.action_status_to_string(state))
            else:
                rospy.loginfo("###### NAV ACTION END , State: %s",self.action_status_to_string(state))
            return state
        except Exception as e:
             rospy.logwarn("###### NAV ACTION FAILURE , State: %s",str(e))
        return GoalStatus.ABORTED

    def simpleGoalcallback(self, data):
        self.saveInterestPoint(data)

    def doorGoalcallback(self, data):
        self.saveInterestPoint(data, type="door_goal")
        #self._simpleGoal_pub.publish(data)

    def saveInterestPoint(self, data, type="simple_goal"):
        itPoint = InterestPoint()
        itPoint.label = "It"+str(self._index_label)
        itPoint.type = type
        itPoint.pose = data.pose
        itPoint.arm_position = 0

        self.sendNavOrderAction('NP', 'CleanRetryReplay', str(self._index_label), 30)

        self._index_label+=1

        f = open(self.CONFIG_PATH + str(itPoint.label) + '.coord', 'w+')
        json_str = json_message_converter.convert_ros_message_to_json(itPoint)
        f.write(json_str)
        f.close()
        rospy.loginfo('Successfully saved the interestPoint:' + str(json_str))


if __name__ == '__main__':


    ## COMMAND SAMPLE ##
    #
    #  rosrun map_manager MapManager.py _confPath:="/home/astrostudent/evers_ws/conf/ITs"
    #
    ####################
    default_value="/home/xia0ben/pepper_ws/data/world_mng/interest_points/"

    rospy.init_node('map_tools_server')
    config_directory_param=rospy.get_param("~confPath",default_value)
    mt = Mt(config_directory_param)
