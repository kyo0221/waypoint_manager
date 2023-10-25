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
        self.serch_waypoint_cnt = 0
        self.search_loop_cnt = 1
        self.rest_point = 0
        self.detect_box_flag =False
        self.area_select_flag = False
        self.select_exec = False
        self.goal_flag = False
        self.prev_goal_data = False
        self.detect_box_way = False
        self.search_box_next = False
        self.area_select_seq = False
        self.detect_result = False
        self.deli_finish_flag = False
        self.area_search_flag = False
        self.prev_stop_data = False
        self.stop_flag = False
        self.area_select_success = False
        self.search_finish = False
        self.loop_break = False
        self.sw_forward_dir = True
        self.toggle = True
        self.sw_waypoint_label = ""
        self.area = ""
        self.area_skip_seq = ""
        self.A_outside_waypoint = rospy.get_param("/area_select/outside_area_A", 3)
        self.A_waypoint = rospy.get_param("/area_select/area_A", 2)
        self.B_waypoint = rospy.get_param("/area_select/area_B", 4) - 1
        self.C_waypoint = rospy.get_param("/area_select/area_C", 3)
        self.A_search_waypoint = rospy.get_param("/area_select/search_waypoint_A", 3)
        self.B_search_waypoint = rospy.get_param("/area_select/search_waypoint_B", 3)
        self.C_search_waypoint = rospy.get_param("/area_select/search_waypoint_C", 3)
        self.skip_waypoint_num = rospy.Publisher("/waypoint_manager/waypoint_next_num", UInt8, queue_size=10)
        self.tag_sub = rospy.Subscriber("/label_string", String, self.label_getter_cb)
        self.goal_sub = rospy.Subscriber("/waypoint_manager/waypoint/is_reached", Bool, self.goal_cb)
        self.area_point = rospy.Service('/area_waypoint', SetBool, self.area_point_server)
        self.result_sv = rospy.Service('/detect_result', SetBool, self.detect_result_server)
        self.area_select_sv = rospy.Service('/area_select', SetBool, self.area_select_server)
        self.stop_waypoint_sv = rospy.Service('/stop_service', SetBool, self.stop_waypoint_server)
        self.delivery_finish_sv = rospy.Service('/finish', SetBool,  self.delivery_finish_server)
        self.sw_waypoint = rospy.Service('/switch_segmentation', SetBool, self.switch_waypoint_server)
        self.skip_waypoint= rospy.ServiceProxy('/waypoint_manager/waypoint_server/next_waypoint', Trigger)
        self.back_waypoint= rospy.ServiceProxy('/waypoint_manager/waypoint_server/prev_waypoint', Trigger)




    def label_getter_cb(self, area_str):
        if self.select_exec:
            self.area = area_str.data
            rospy.loginfo("get area label %s" % self.area)
        else:
            rospy.logwarn("NOT EXECUTED select task yet")


    def goal_cb(self, goal):
        if self.select_exec:
            self.prev_goal_data = goal.data
            
            if self.prev_goal_data != self.goal_flag:
                self.goal_flag = not self.goal_flag
                rospy.loginfo("goal flag %s" % self.goal_flag)
                self.toggle = True

            else:
                if self.toggle:
                    rospy.logwarn("same goal flag")
                    self.toggle = False
        else:
            rospy.logwarn("NOT EXECUTED select task yet from goal_cb")


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
                self.detect_result = True
                rospy.loginfo("result -> box detected!")
                return SetBoolResponse (True, 'detect success')
            else:
                self.area_select_flag = False
                self.detect_result = False
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
                self.area_search_flag = True
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
        if self.select_exec:
            if finish_status.data:
                self.deli_finish_flag = True
                rospy.loginfo("delivery success")
                return SetBoolResponse (True, ' delivery success')
            else:
                self.select_exec = False
                rospy.logwarn("delivery failed")
                return SetBoolResponse(False, 'delivery failed')
        else:
            rospy.logwarn("NOT EXECUTED select task yet")
            return SetBoolResponse(False, 'NOT EXECUTE')



    def switch_waypoint_server(self, point_status):
        if self.select_exec:
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
        else:
            rospy.logwarn("NOT EXECUTED select task yet")
            return SetBoolResponse(False, 'NOT EXECUTE')
        

    def stop_waypoint_server(self, status):
        if self.select_exec:
            self.prev_stop_data = status.data
            if self.prev_stop_data != self.stop_flag:
                if (not self.area_select_flag) and self.area_select_success:
                    if status.data:
                        rospy.loginfo("stop point reached")
                        self.stop_flag = status.data
                        self.serch_waypoint_cnt += 1
                        return SetBoolResponse(True, 'true')
                    else:
                        self.stop_flag = status.data
                        rospy.loginfo("stop point false")
                        return SetBoolResponse(False, 'false')
                else:
                    rospy.loginfo("searching box")
                    return SetBoolResponse(False, 'false')
            else:
                return SetBoolResponse(False, 'same stop point flag')
        else:
            rospy.logwarn("NOT EXECUTED select task yet")
            return SetBoolResponse(False, 'NOT EXECUTE')


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
            self.rest_point = self.A_search_waypoint
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
            self.rest_point = self.B_search_waypoint
            return "b"

        elif self.area == 'tag_c':
            rospy.loginfo("select area C")
            self.area = ""
            self.rest_point = self.C_search_waypoint
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
            self.skip_waypoint_num.publish(self.A_outside_waypoint + 1)

            rospy.wait_for_service('waypoint_manager/waypoint_server/next_waypoint')
            try:
                self.skip_waypoint()
            except rospy.ServiceException as err:
                rospy.logfatal("Service call failed: %s" %err)

            return
        
        else:
            rospy.logfatal("not define skip label")
            return


    def search_forward_seq(self, rest_point_cnt):
        if self.sw_forward_dir:
            rospy.loginfo("next search point")
            rest_point_cnt -= 1
            self.next_waypoint(1)
            rospy.loginfo("rest search point %d" % rest_point_cnt)
        else:
            rospy.loginfo("prev search point")
            rest_point_cnt += 1
            self.prev_waypoint(1)
            rospy.loginfo("rest search point %d" % rest_point_cnt)
        
        return rest_point_cnt


    def search_seq(self):
        rospy.loginfo("search waypoint trans %d" % self.serch_waypoint_cnt)
        if self.area_skip_seq == 'a':
            rospy.loginfo("at area A")
            if self.serch_waypoint_cnt < self.A_search_waypoint:
                rospy.loginfo("next search")
                self.rest_point = self.search_forward_seq(self.rest_point)
                return 
            else:
                rospy.loginfo("switch direction")
                self.sw_forward_dir = not self.sw_forward_dir
                self.serch_waypoint_cnt = 1
                self.search_loop_cnt += 1
                if self.search_loop_cnt <= 3:
                    rospy.loginfo("continue loop")
                    self.rest_point = self.search_forward_seq(self.rest_point)
                    return
                else:
                    rospy.loginfo("loop break")
                    self.loop_break = True
                    return

        elif self.area_skip_seq == 'b':
            rospy.loginfo("at area B")
            if self.serch_waypoint_cnt < self.B_search_waypoint:
                rospy.loginfo("next search")
                self.rest_point = self.search_forward_seq(self.rest_point)
                return 
            else:
                rospy.loginfo("switch direction")
                self.sw_forward_dir = not self.sw_forward_dir
                self.serch_waypoint_cnt = 1
                self.search_loop_cnt += 1
                if self.search_loop_cnt <= 3:
                    rospy.loginfo("continue loop")
                    self.rest_point = self.search_forward_seq(self.rest_point)
                    return
                else:
                    rospy.loginfo("loop break")
                    self.loop_break = True
                    return

        elif self.area_skip_seq == 'c':
            rospy.loginfo("at area C")
            if self.serch_waypoint_cnt < self.C_search_waypoint:
                rospy.loginfo("next search")
                self.rest_point = self.search_forward_seq(self.rest_point)
                return 
            else:
                rospy.loginfo("switch direction")
                self.sw_forward_dir = not self.sw_forward_dir
                self.serch_waypoint_cnt = 1
                self.search_loop_cnt += 1
                if self.search_loop_cnt <= 3:
                    rospy.loginfo("continue loop")
                    self.rest_point = self.search_forward_seq(self.rest_point)
                    return
                else:
                    rospy.loginfo("loop break")
                    self.loop_break = True
                    return


    def next_waypoint(self, skip_num):
        self.skip_waypoint_num.publish(skip_num)
        
        rospy.wait_for_service('waypoint_manager/waypoint_server/next_waypoint')
        try:
            self.skip_waypoint()
        except rospy.ServiceException as err:
            rospy.logfatal("Service call failed: %s" %err)

        return


    def prev_waypoint(self, skip_num):
        self.skip_waypoint_num.publish(skip_num)
        
        rospy.wait_for_service('waypoint_manager/waypoint_server/prev_waypoint')
        try:
            self.back_waypoint()
        except rospy.ServiceException as err:
            rospy.logfatal("Service call failed: %s" %err)

        return



    def loop(self):
        if self.select_exec:
            rospy.loginfo_once("---- area select process execute ----")
            if self.time % 5 == 0:
                rospy.loginfo("job progress")

            if self.search_box_next and (not self.area_search_flag) and (not self.search_finish):
                rospy.loginfo("next waypoint 1")
                self.next_waypoint(1)
                self.search_box_next = False

            if self.area_select_seq:
                self.area_skip_seq = self.area_selector()
                rospy.loginfo("%s" % self.area_skip_seq)
                if (self.area_skip_seq == "a") or (self.area_skip_seq == "b") or (self.area_skip_seq == "c"):
                    rospy.loginfo("area selected!")
                    self.area_select_success = True
                    self.area_select_seq = False
                else:
                    rospy.logfatal("box detect failed")
                    self.area_select_seq = False

            if (self.deli_finish_flag or self.loop_break) and (not self.search_finish):
                self.sw_forward_dir = True
                rospy.loginfo("delivery finish")
                if self.stop_flag:
                    if (self.rest_point-1) > 0:
                        rospy.loginfo("skip rest search waypoint")
                        self.rest_point = self.search_forward_seq(self.rest_point)
                    else:
                        if self.area_skip_seq == "a":
                            rospy.loginfo("skip outside A")
                            self.skip_area(self.area_skip_seq)
                        else:
                            self.rest_point = self.search_forward_seq(self.rest_point)
                            rospy.loginfo("skip rest search waypoint end!")
                            self.deli_finish_flag = False
                            self.area_search_flag = False
                            self.search_finish = True
                else:
                    rospy.loginfo("wait stop waypoint")


            if self.area_search_flag and self.stop_flag and (not self.deli_finish_flag) and (not self.loop_break):
                if self.search_box_next:
                    rospy.loginfo("area search loop %d" % self.search_loop_cnt)
                    self.search_seq()
                    self.stop_flag = False
                    self.search_box_next = False
                else:
                    rospy.loginfo("delivery-box searching")

            if self.sw_waypoint_label == "skip_a":
                if self.area_skip_seq == "a":
                    rospy.loginfo("not skip area A")
                else:
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