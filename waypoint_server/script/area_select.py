#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Trigger, TriggerResponse
import rospy


class AreaSelectNode():
    def __init__(self):
        rospy.init_node('area_select', anonymous=True)
        self.detect_box_flag =False
        self.area_select_sucess = False
        self.select_exec = False
        self.area = ""
        self.A_waypoint = rospy.get_param("/area_select/bigining_area_A", 6)
        self.B_waypoint = rospy.get_param("/area_select/bigining_area_B", 3)
        self.C_waypoint = rospy.get_param("/area_select/bigining_area_C", 1)
        self.skip_waypoint_num = rospy.Publisher("/waypoint_manager/waypoint_next_num", UInt8, queue_size=10)
        self.vel_pub = rospy.Publisher("/area_select/rotate_vel", Twist, queue_size=10)
        self.tag_sub = rospy.Subscriber("/label_string", String, self.label_getter_cb)
        self.detect_box = rospy.Service('/area_select/detect_box', SetBool, self.detect_box_server)
        self.select_exec_sv = rospy.Service('/area_select/select_exec', Trigger, self.select_exec_server)
        self.skip_area= rospy.ServiceProxy('/waypoint_manager/waypoint_server/next_waypoint', Trigger)

    def label_getter_cb(self, area_str):
        if(self.select_exec):
            self.area = area_str.data
            rospy.loginfo("get area label %s" % self.area)
        else:
            rospy.logwarn("NOT EXECUTE select task")


    def detect_box_server(self, request):
        if(self.select_exec):
            if request.data:
                self.detect_box_flag = True
                return SetBoolResponse (True, 'start selecting area')
            else:
                return SetBoolResponse(False, 'not start selecting area task')
        else:
            return SetBoolResponse(False, 'NOT EXECUTE')
                           
                           
    def select_exec_server(self, request):
        if(self.select_exec):
            self.select_exec = True
            return TriggerResponse(True, 'select area task execute')
        

    def loop(self):
        while self.select_exec:
            rospy.loginfo("wait detected result...")

            if self.detect_box_flag:
                rospy.loginfo("wait area tag...")

                if self.area == 'tag_a':
                    rospy.loginfo("select area A")
                    self.skip_waypoint_num.publish(self.B_waypoint - self.C_waypoint)
                    rospy.wait_for_service('waypoint_manager/waypoint_server/next_waypoint')

                    try:
                        self.skip_area()
                    except rospy.ServiceException as e:
                        rospy.loginfo("Service call failed: %s" %e)

                    self.area = ""

                elif self.area == 'tag_b':
                    rospy.loginfo("select area B")
                    self.skip_waypoint_num.publish(self.B_waypoint - self.C_waypoint)
                    rospy.wait_for_service('waypoint_manager/waypoint_server/next_waypoint')

                    try:
                        self.skip_area()
                    except rospy.ServiceException as e:
                        rospy.loginfo("Service call failed: %s" %e)
                    
                    self.area = ""
                    
                elif self.area == 'tag_c':
                    rospy.loginfo("select area C")
                
                else:
                    rospy.loginfo("not detected")
                    self.select_exec = False
                    break

            else:
                rospy.logwarn("box not detected")
                self.select_exec = False
                break


if __name__ == '__main__':
    rospy.loginfo('area select started')
    node = AreaSelectNode()
    DURATION = 1
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        node.loop()
        r.sleep()