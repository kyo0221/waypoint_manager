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
        self.time = 0
        self.detect_point_cnt = 0
        self.detect_box_flag =False
        self.area_select_flag = False
        self.select_exec = False
        self.goal_flag = False
        self.detect_box_way = False
        self.search_box_next = False
        self.area_select_seq = False
        self.deli_finish_flag = False
        self.sw_waypoint_label = ""
        self.area = ""
        self.area_skip_seq = ""
        self.A_waypoint = rospy.get_param("/area_select/bigining_area_A", 2)
        self.B_waypoint = rospy.get_param("/area_select/bigining_area_B", 2)
        self.C_waypoint = rospy.get_param("/area_select/bigining_area_C", 2)
        self.skip_waypoint_num = rospy.Publisher("/waypoint_manager/waypoint_next_num", UInt8, queue_size=10)
        self.tag_sub = rospy.Subscriber("/label_string", String, self.label_getter_cb)
        self.goal_sub = rospy.Subscriber("/waypoint_manager/waypoint/is_reached", Bool, self.goal_cb)
        self.area_point = rospy.Service('/area_waypoint', SetBool, self.area_point_server)
        self.result_sv = rospy.Service('/detect_result', SetBool, self.detect_result_server)
        self.area_select_sv = rospy.Service('/area_select', SetBool, self.area_select_server)
        self.delivery_finish_sv = rospy.Service('/finish', SetBool,  self.delivery_finish_server)
        self.skip_waypoint= rospy.ServiceProxy('/waypoint_manager/waypoint_server/next_waypoint', Trigger)
        self.sw_waypoint = rospy.Service('/switch_segmentation', SetBool, self.switch_waypoint_server)




    def label_getter_cb(self, area_str):
        if self.select_exec:
            self.area = area_str.data
            rospy.loginfo("get area label %s" % self.area)
        else:
            rospy.logwarn_once("NOT EXECUTED select task yet")


    def goal_cb(self, goal):
        if self.select_exec:
            self.goal_flag = goal.data
            #rospy.loginfo("goal reached")
        else:
            rospy.logwarn_once("NOT EXECUTED select task yet")


    # def detect_box_server(self, request):
    #     if self.select_exec:
    #         self.detect_box_way = True
    #         rospy.loginfo("box detected")
    #         return SetBoolResponse (True, 'start detect box')
    #     else:
    #         rospy.logwarn("NOT EXECUTED select task yet")
    #         return SetBoolResponse(False, 'NOT EXECUTE')


    def area_point_server(self, request):
        if request.data:
            self.select_exec = True
            rospy.loginfo("start delivery area")
            return SetBoolResponse (True, 'start delivery area')
        else:
            self.select_exec = False
            rospy.logwarn("end delivery area")
            return SetBoolResponse(False, 'end delivery area')


    def detect_result_server(self, request):
        if self.select_exec:
            self.search_box_next = True
            if request.data:
                self.area_select_flag = True
                rospy.loginfo("result -> box detected!")
                return SetBoolResponse (True, 'detect success')
            else:
                self.area_select_flag = False
                rospy.logwarn("result -> box not detected")
                return SetBoolResponse(False, 'detect failed')
        else:
            rospy.logwarn("NOT EXECUTED select task yet")
            return SetBoolResponse(False, 'NOT EXECUTE')


    def area_select_server(self, request):
        if self.select_exec and request.data:
            if self.area_select_flag:
                rospy.loginfo("getting area tag...")
                self.area_select_seq = True
                self.area_select_flag = False
                return SetBoolResponse(False, 'area select')

            else:
                rospy.logwarn("box not detected")
                self.area_select_seq = True
                self.area_select_flag = False
                return SetBoolResponse(False, 'box detection failed. start return navigation')
        else:
            rospy.logwarn("NOT EXECUTED select task yet")
            return SetBoolResponse(False, 'NOT EXECUTE')
        

    def delivery_finish_server(self, finish_status):
        if finish_status.data:
            self.deli_finish_flag = True
            rospy.loginfo("delivery success")
            return SetBoolResponse (True, ' delivery success')
        else:
            self.select_exec = False
            rospy.logwarn("delivery failed")
            return SetBoolResponse(False, 'delivery failed')


    def switch_waypoint_server(self, point_status):
        if point_status.data:
            self.sw_waypoint_label = "skip_b"
            rospy.loginfo("waypoint sw 1")
            return SetBoolResponse(True, 'switch 1')
        else:
            if self.area == 'a':
                rospy.loginfo("not skip area A")
                return SetBoolResponse(False, 'not skip')
            else:
                self.sw_waypoint_label = "skip_a"
                rospy.loginfo("waypoint sw 0")
                return SetBoolResponse(False, 'switch 0')


    def area_selector(self):
        if self.area == 'tag_a':
            rospy.loginfo("select area A")
            self.skip_waypoint_num.publish(self.C_waypoint + self.B_waypoint - 1)

            rospy.wait_for_service('waypoint_manager/waypoint_server/next_waypoint')
            try:
                self.skip_waypoint()
            except rospy.ServiceException as err:
                rospy.logfatal("Service call failed: %s" %err)

            self.area = ""
            return "a"

        elif self.area == 'tag_b':
            rospy.loginfo("select area B")
            self.skip_waypoint_num.publish(self.C_waypoint - 1)

            rospy.wait_for_service('waypoint_manager/waypoint_server/next_waypoint')
            try:
                self.skip_waypoint()
            except rospy.ServiceException as err:
                rospy.logfatal("Service call failed: %s" %err)
            
            self.area = ""
            return "b"
            
        elif self.area == 'tag_c':
            rospy.loginfo("select area C")
            return "c"
        
        else:
            rospy.logwarn("detected failed. return navigation")
            self.skip_waypoint_num.publish(self.C_waypoint + self.B_waypoint + self.A_waypoint - 1)

            rospy.wait_for_service('waypoint_manager/waypoint_server/next_waypoint')
            try:
                self.skip_waypoint()
            except rospy.ServiceException as err:
                rospy.logfatal("Service call failed: %s" %err)
            return "failed"


    def skip_area(self, skip_label):
        rospy.loginfo("area skipping")
        if skip_label == 'skip_a':
            rospy.loginfo("skip area A")
            self.skip_waypoint_num.publish(self.A_waypoint - 1)
        
            rospy.wait_for_service('waypoint_manager/waypoint_server/next_waypoint')
            try:
                self.skip_waypoint()
            except rospy.ServiceException as err:
                rospy.logfatal("Service call failed: %s" %err)

            return 

        elif skip_label == 'skip_b':
            rospy.loginfo("skip area B")
            self.skip_waypoint_num.publish(self.B_waypoint - 1)
        
            rospy.wait_for_service('waypoint_manager/waypoint_server/next_waypoint')
            try:
                self.skip_waypoint()
            except rospy.ServiceException as err:
                rospy.logfatal("Service call failed: %s" %err)

            return
        
        elif skip_label == 'a':
            rospy.loginfo("skip outside area A")
            self.skip_waypoint_num.publish(self.detect_point_cnt)
        
            rospy.wait_for_service('waypoint_manager/waypoint_server/next_waypoint')
            try:
                self.skip_waypoint()
            except rospy.ServiceException as err:
                rospy.logfatal("Service call failed: %s" %err)

            return


    def loop(self):
        if self.select_exec:
            rospy.loginfo_once("---- area select process execute ----")
            if self.time % 5 == 0:
                rospy.loginfo("job progress")

            if self.search_box_next:
                rospy.loginfo("next waypoint 1")
                rospy.wait_for_service('/waypoint_manager/waypoint_server/next_waypoint')
                req = self.skip_waypoint()
                self.search_box_next = False

            if self.area_select_seq:
                self.area_skip_seq = self.area_selector()
                rospy.loginfo("%s" % self.area_skip_seq)
                if (self.area_skip_seq == "a") or (self.area_skip_seq == "b") or (self.area_skip_seq == "c"): #処理を追加する用（消しても良い）
                    rospy.loginfo("area selected!")
                    self.area_select_seq = False
                else:
                    rospy.logfatal("box detect failed")
                    self.area_select_seq = False

            if self.deli_finish_flag:
                rospy.loginfo("delivery finish")
                if self.area_skip_seq == "a":
                    rospy.loginfo("skip outside A")
                    self.skip_area(self.area_skip_seq)
                    self.deli_finish_flag = False
                else:
                    rospy.loginfo("skip rest area waypoint")
                    self.skip_area("detect_waypoint_count")
                    self.deli_finish_flag = False

            if self.sw_waypoint_label == "skip_a":
                self.skip_area(self.sw_waypoint_label)
                self.sw_waypoint_label = ""

            if self.sw_waypoint_label == "skip_b":
                self.skip_area(self.sw_waypoint_label)
                self.sw_waypoint_label = ""


        self.time += DURATION

                


if __name__ == '__main__':
    rospy.loginfo("area select started")
    node = AreaSelectNode()
    DURATION = 1
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        node.loop()
        r.sleep()