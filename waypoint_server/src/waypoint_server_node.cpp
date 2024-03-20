#include <iostream>
#include <string>
#include <functional>
#include <thread>
#include <stdio.h>
#include <time.h>

using namespace std;

#include <Eigen/Dense>

#include <ros/ros.h>
#include <dynamic_reconfigure/client.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <topic_tools/MuxSelect.h>
#include <dynamic_reconfigure/Reconfigure.h>

#include <obstacle_detector/Obstacles.h>
#include <obstacle_detector/CircleObstacle.h>

#include <waypoint_manager_msgs/Waypoint.h>
#include <waypoint_manager_msgs/WaypointStamped.h>
#include <waypoint_manager_msgs/Waypoints.h>
#include <waypoint_manager_msgs/Route.h>

#include <waypoint_server/waypoint_server.h>

#define SIZE_OF_ARRAY(array) (sizeof(array) / sizeof(array[0]))

bool seg_flag = true;
bool traffic_flag = true;
bool skip_flag = false;
bool obstacle_flag = true;
bool detect_box_flag = true;
bool area_select_flag = true;
bool area_waypoint_flag = true;

// SKIP_WAYPOINT
// skipWaypointする際の秒数指定
double skip_sec = 0.0;
double to_skip_sec = 90.0;
// skipするためのカウントを行う上限速度
double skip_vel = 0.05;
// use_skipwaypoint
bool use_skip = false;
// radius for skip
double SKIP_RADIUS = 0.5;

time_t start_time, end_time;
time_t start_skip_time, end_skip_time;
double goal_X, goal_Y;

namespace waypoint_server
{
    struct NodeParameters
    {
        std::string goal_topic,
            waypoint_topic,
            is_reached_goal_topic,
            regist_goal_pose_topic,
            regist_goal_point_topic,
            erase_goal_topic,
            update_goal_topic,
            route_topic,
            append_route_topic,
            erase_route_topic,
            insert_route_topic,
            waypoints_topic;

        std::string regist_waypoint_prefix;

        std::string robot_base_frame,
            global_frame;

        std::string waypoints_file,
            route_file;

        bool debug,
            latch,
            enable_2d,
            enable_3d,
            enable_loop;

        int publish_queue_size,
            subscribe_queue_size;

        float wait_publish_waypoints_time;

        double goal_publish_frequency;
    };

    class Node
    {

    public:
        Node();

        void spin();

    private:
        ros::NodeHandle private_nh,
            nh;

        ros::Publisher waypoint_publisher,
            waypoints_publisher,
            route_publisher;

        ros::Subscriber is_reached_goal_subscriber,
            regist_goal_pose_subscriber,
            regist_goal_point_subscriber,
            erase_goal_subscriber,
            update_goal_subscriber,
            append_route_subscriber,
            erase_route_subscriber,
            insert_route_subscriber,
            cmd_vel_subscriber,
            obstacle_pos_subscriber,
            waypoint_forward_index_subscriber;

        ros::ServiceServer save_service,
            save_waypoints_service,
            save_route_service,
            reset_route_service,
            switch_cancel_service,
            next_waypoint_service,
            prev_waypoint_service,
            resume_waypoint_service;

        ros::ServiceClient white_line_service,
            detect_start_service,
            switch_segmentation_service,
            stop_service,
            traffic_service,
            clear_costmap_service,
            detect_box_service, 
            area_select_service, 
            area_waypoint_service;
        // config_service;

        NodeParameters param;

        Map waypoint_map;
        Route router;

        std::atomic<unsigned int> regist_goal_id;
        std::atomic_bool is_cancel;

        // TODO Change message type
        void isReachedGoal(const std_msgs::Bool::ConstPtr &),
            registGoalPose(const geometry_msgs::PoseStamped::ConstPtr &),
            registGoalPoint(const geometry_msgs::PointStamped::ConstPtr &),
            eraseGoal(const std_msgs::String::ConstPtr &),
            updateGoalPose(const waypoint_manager_msgs::WaypointStamped::ConstPtr &),
            appendRoute(const std_msgs::String::ConstPtr &),
            eraseRoute(const std_msgs::String::ConstPtr &),
            insertRoute(const std_msgs::String::ConstPtr &),
            cmd_vel_(const geometry_msgs::Twist::ConstPtr &),
            obstacle_pos(const obstacle_detector::Obstacles::ConstPtr &),
            waypointForwardIndex(const std_msgs::UInt8::ConstPtr &);

        bool save(
            std_srvs::TriggerRequest &request,
            std_srvs::TriggerResponse &response);
        bool saveWaypoints(
            std_srvs::TriggerRequest &request,
            std_srvs::TriggerResponse &response);
        bool saveRoute(
            std_srvs::TriggerRequest &request,
            std_srvs::TriggerResponse &response);
        bool resetRoute(
            std_srvs::TriggerRequest &request,
            std_srvs::TriggerResponse &response);
        bool switchCancel(
            std_srvs::TriggerRequest &request,
            std_srvs::TriggerResponse &response);
        bool nextWaypoint(
            std_srvs::TriggerRequest &request,
            std_srvs::TriggerResponse &response);
        bool prevWaypoint(
            std_srvs::TriggerRequest &request,
            std_srvs::TriggerResponse &response);
        bool resumeWaypoint(
            std_srvs::TriggerRequest &request,
            std_srvs::TriggerResponse &response);
        void Muxselect();
        void detectstart();
        void switch_segmentation();
        void TrafficSignService();
        void StopService();
        void ClearCostmapService();
        void skipWaypoint();
        void detect_box();
        void area_select();
        void area_waypoint();

        void publishGoal(),
            publishWaypoints(),
            publishRoute();

        void publishLatchedData();

        void exchangeCancelState();
    };

    Node::Node() : private_nh("~"),
                   nh()
    {

        private_nh.param(
            "goal_topic",
            param.goal_topic,
            std::string("move_base_simple/goal"));
        private_nh.param(
            "waypoint_topic",
            param.waypoint_topic,
            std::string("waypoint"));
        private_nh.param(
            "is_reached_goal_topic",
            param.is_reached_goal_topic,
            std::string("waypoint/is_reached"));
        private_nh.param(
            "regist_goal_pose_topic",
            param.regist_goal_pose_topic,
            std::string("waypoint/regist_pose"));
        private_nh.param(
            "regist_goal_point_topic",
            param.regist_goal_point_topic,
            std::string("waypoint/regist_point"));
        private_nh.param(
            "erase_goal_topic",
            param.erase_goal_topic,
            std::string("waypoint/erase"));
        private_nh.param(
            "update_goal_topic",
            param.update_goal_topic,
            std::string("waypoint/update"));
        private_nh.param(
            "route_topic",
            param.route_topic,
            std::string("route"));
        private_nh.param(
            "append_route_topic",
            param.append_route_topic,
            std::string("route/append"));
        private_nh.param(
            "erase_route_topic",
            param.erase_route_topic,
            std::string("route/erase"));
        private_nh.param(
            "insert_route_topic",
            param.insert_route_topic,
            std::string("route/insert"));
        private_nh.param(
            "waypoints_topic",
            param.waypoints_topic,
            std::string("waypoints"));
        private_nh.param(
            "regist_waypoint_prefix",
            param.regist_waypoint_prefix,
            std::string("registed_"));
        private_nh.param(
            "robot_base_frame",
            param.robot_base_frame,
            std::string("base_link"));
        private_nh.param(
            "global_frame",
            param.global_frame,
            std::string("map"));
        private_nh.param(
            "waypoints_file",
            param.waypoints_file,
            std::string(""));
        private_nh.param(
            "route_file",
            param.route_file,
            std::string(""));
        private_nh.param(
            "debug",
            param.debug,
            false);
        private_nh.param(
            "latch",
            param.latch,
            false);
        private_nh.param(
            "enable_loop",
            param.enable_loop,
            false);
        private_nh.param(
            "publish_queue_size",
            param.publish_queue_size,
            1);
        private_nh.param(
            "subscribe_queue_size",
            param.subscribe_queue_size,
            1);
        private_nh.param(
            "wait_publish_waypoints_time",
            param.wait_publish_waypoints_time,
            static_cast<float>(5e-3));
        private_nh.param(
            "goal_publish_frequency",
            param.goal_publish_frequency,
            0.5);

        waypoint_publisher = nh.advertise<waypoint_manager_msgs::Waypoint>(
            param.waypoint_topic,
            param.publish_queue_size,
            param.latch);
        waypoints_publisher = nh.advertise<waypoint_manager_msgs::Waypoints>(
            param.waypoints_topic,
            param.publish_queue_size,
            true);
        route_publisher = nh.advertise<waypoint_manager_msgs::Route>(
            param.route_topic,
            param.publish_queue_size,
            true);

        is_reached_goal_subscriber = nh.subscribe<std_msgs::Bool>(
            param.is_reached_goal_topic,
            param.subscribe_queue_size,
            &Node::isReachedGoal,
            this);
        regist_goal_pose_subscriber = nh.subscribe<geometry_msgs::PoseStamped>(
            param.regist_goal_pose_topic,
            param.subscribe_queue_size,
            &Node::registGoalPose,
            this);
        regist_goal_point_subscriber = nh.subscribe<geometry_msgs::PointStamped>(
            param.regist_goal_point_topic,
            param.subscribe_queue_size,
            &Node::registGoalPoint,
            this);
        erase_goal_subscriber = nh.subscribe<std_msgs::String>(
            param.erase_goal_topic,
            param.subscribe_queue_size,
            &Node::eraseGoal,
            this);
        update_goal_subscriber = nh.subscribe<waypoint_manager_msgs::WaypointStamped>(
            param.update_goal_topic,
            param.subscribe_queue_size,
            &Node::updateGoalPose,
            this);
        append_route_subscriber = nh.subscribe<std_msgs::String>(
            param.append_route_topic,
            param.subscribe_queue_size,
            &Node::appendRoute,
            this);
        erase_route_subscriber = nh.subscribe<std_msgs::String>(
            param.erase_route_topic,
            param.subscribe_queue_size,
            &Node::eraseRoute,
            this);
        insert_route_subscriber = nh.subscribe<std_msgs::String>(
            param.insert_route_topic,
            param.subscribe_queue_size,
            &Node::insertRoute,
            this);
        cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>(
            "/cmd_vel",
            10,
            &Node::cmd_vel_,
            this);
        obstacle_pos_subscriber = nh.subscribe<obstacle_detector::Obstacles>(
            "/obstacles",
            10,
            &Node::obstacle_pos,
            this);
        waypoint_forward_index_subscriber = nh.subscribe<std_msgs::UInt8>(
            "/waypoint_manager/waypoint_next_num",
            10,
            &Node::waypointForwardIndex,
            this);

        save_service = private_nh.advertiseService(
            "save",
            &Node::save,
            this);
        save_waypoints_service = private_nh.advertiseService(
            "save_waypoints",
            &Node::saveWaypoints,
            this);
        save_route_service = private_nh.advertiseService(
            "save_route",
            &Node::saveRoute,
            this);
        reset_route_service = private_nh.advertiseService(
            "reset_route",
            &Node::resetRoute,
            this);
        switch_cancel_service = private_nh.advertiseService(
            "switch_cancel",
            &Node::switchCancel,
            this);
        next_waypoint_service = private_nh.advertiseService(
            "next_waypoint",
            &Node::nextWaypoint,
            this);
        prev_waypoint_service = private_nh.advertiseService(
            "prev_waypoint",
            &Node::prevWaypoint,
            this);
        resume_waypoint_service = private_nh.advertiseService(
            "resume_waypoint",
            &Node::resumeWaypoint,
            this);
        white_line_service = private_nh.serviceClient<topic_tools::MuxSelect>(
            "/mux/select");
        detect_start_service = private_nh.serviceClient<std_srvs::Trigger>(
            "/start_detect");
        switch_segmentation_service = private_nh.serviceClient<std_srvs::SetBool>(
            "/switch_segmentation");
        stop_service = private_nh.serviceClient<std_srvs::SetBool>("/stop_service");
        traffic_service = private_nh.serviceClient<std_srvs::SetBool>("/traffic_service");
        clear_costmap_service = private_nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        detect_box_service = private_nh.serviceClient<std_srvs::SetBool>(
            "/detect_box");
        area_select_service = private_nh.serviceClient<std_srvs::SetBool>(
            "/area_select");
        area_waypoint_service = private_nh.serviceClient<std_srvs::SetBool>(
            "/area_waypoint");

        is_cancel.store(true);
        regist_goal_id.store(0);

        ROS_INFO("Loading waypoints_file %s", param.waypoints_file.c_str());
        waypoint_map.load(param.waypoints_file);
        ROS_INFO("Success load waypoints_file");

        ROS_INFO("Loading route_file: %s", param.route_file.c_str());
        router.load(param.route_file);
        router.loop(param.enable_loop);
        ROS_INFO("Success load route_file");
        ROS_INFO("Count of skip ids %d", router.getSkipIds());
    }

    void Node::spin()
    {
        std::thread subscribers_thread{
            [this]()
            {
                ros::spin();
            }};

        ros::Rate rate{param.goal_publish_frequency};

        rate.sleep();
        publishLatchedData();

        while (ros::ok())
        {
            publishGoal();
            if (use_skip)
            {
                if (!is_cancel.load())
                {
                    end_time = time(NULL);
                    ROS_INFO("time:%ld, to_skip time:%ld\n", end_time - start_time, end_skip_time - start_skip_time);
                    if ((end_time - start_time >= skip_sec) && (skip_flag))
                    {
                        if (!(waypoint_map[router.getIndex()].properties["stop"] == "true"))
                            skipWaypoint();
                        start_time = time(NULL);
                        start_skip_time = time(NULL);
                        skip_flag = false;
                    }
                }
                else
                {
                    start_time = time(NULL);
                    start_skip_time = time(NULL);
                    printf("wait!\n");
                }
            }
            rate.sleep();
        }
        ros::shutdown();
        subscribers_thread.join();
    }

    void Node::isReachedGoal(const std_msgs::Bool::ConstPtr &msg)
    {
        if (router.isEmpty())
        {
            ROS_WARN("Route size of zero");
            return;
        }
        if (is_cancel.load())
        {
            return;
        }
        if (!msg->data)
        {
            return;
        }
        ROS_INFO(
            "Goal reached %s from waypoint_server_node",
            router.getIndex().c_str());
        if (router.forward_index != 1)
        {
            router.forward_index = 1;
        }
        if (waypoint_map[router.getIndex()].properties["detect_box_ON"] == "true")
        {
            ROS_INFO("Current waypoint properties detect_box_ON is true");
            detect_box_flag = true;
            detect_box();
        }
        if (waypoint_map[router.getIndex()].properties["detect_box_OFF"] == "true")
        {
            ROS_INFO("Current waypoint properties detect_box_OFF is true");
            detect_box_flag = false;
            detect_box();
        }
        if (waypoint_map[router.getIndex()].properties["stop"] == "true")
        {
            ROS_INFO("Current waypoint properties stop is true");
            ROS_INFO("Please call the ~/next_waypoint service");
            StopService();
            start_time = time(NULL);
            start_skip_time = time(NULL);
            return;
        }
        if (waypoint_map[router.getIndex()].properties["standby_mode"] == "true")
        {
            ROS_INFO("Current waypoint properties standby_mode is true");
            start_skip_time = time(NULL);
            if (obstacle_flag)
                return;
        }
        if (waypoint_map[router.getIndex()].properties["white"] == "true")
        {
            ROS_INFO("Current waypoint properties white is true");
            ROS_INFO("Please call the ~/resume_waypoint service");
            detectstart();
        }
        if (waypoint_map[router.getIndex()].properties["traffic_sign_ON"] == "true")
        {
            ROS_INFO("Current waypoint properties traffic_sign_ON is true");
            // ROS_INFO("Please call the ~/resume_waypoint service");
            traffic_flag = true;
            TrafficSignService();
        }
        if (waypoint_map[router.getIndex()].properties["traffic_sign_OFF"] == "true")
        {
            ROS_INFO("Current waypoint properties traffic_sign_OFF is true");
            // ROS_INFO("Please call the ~/resume_waypoint service");
            traffic_flag = false;
            TrafficSignService();
        }
        if (waypoint_map[router.getIndex()].properties["switch_segmentation_ON"] == "true")
        {
            ROS_INFO("Current waypoint properties switch_segmentation_ON is true");
            // ROS_INFO("Please call the ~/resume_waypoint service");
            seg_flag = true;
            switch_segmentation();
        }
        if (waypoint_map[router.getIndex()].properties["switch_segmentation_OFF"] == "true")
        {
            ROS_INFO("Current waypoint properties switch_segmentation_OFF is true");
            // ROS_INFO("Please call the ~/resume_waypoint service");
            seg_flag = false;
            switch_segmentation();
        }
        if (waypoint_map[router.getIndex()].properties["area_select_ON"] == "true")
        {
            ROS_INFO("Current waypoint properties area_select_ON is true");
            area_select_flag = true;
            area_select();
        }
        if (waypoint_map[router.getIndex()].properties["area_select_OFF"] == "true")
        {
            ROS_INFO("Current waypoint properties area_select_OFF is true");
            area_select_flag = false;
            area_select();
        }
        if (waypoint_map[router.getIndex()].properties["area_waypoint_ON"] == "true")
        {
            ROS_INFO("Current waypoint properties area_waypoint_ON is true");
            area_waypoint_flag = true;
            area_waypoint();
        }
        if (waypoint_map[router.getIndex()].properties["area_waypoint_OFF"] == "true")
        {
            ROS_INFO("Current waypoint properties area_waypoint_OFF is true");
            area_waypoint_flag = false;
            area_waypoint();
        }
        if (!router.forwardIndex())
        {
            ROS_WARN("Not found next index for route");
            is_cancel.store(true);
        }
        else
        {
            publishGoal();
            // start_time = time(NULL);
        }
    }

    Map::Key generateKey(const std::atomic<unsigned int> &id, const std::string &prefix)
    {
        return prefix + std::to_string(id.load()) + "_" + std::to_string(ros::Time::now().toNSec());
    }

    void Node::registGoalPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        const auto name = generateKey(regist_goal_id, param.regist_waypoint_prefix);

        waypoint_map[name].goal.x() = msg->pose.position.x;
        waypoint_map[name].goal.y() = msg->pose.position.y;
        waypoint_map[name].goal.z() = msg->pose.position.z;
        waypoint_map[name].quaternion.x() = msg->pose.orientation.x;
        waypoint_map[name].quaternion.y() = msg->pose.orientation.y;
        waypoint_map[name].quaternion.z() = msg->pose.orientation.z;
        waypoint_map[name].quaternion.w() = msg->pose.orientation.w;
        waypoint_map.setQuaternion(name);

        ROS_INFO("Add waypoint %s from pose", name.c_str());

        regist_goal_id.store(regist_goal_id.load() + 1);

        publishLatchedData();
    }

    void Node::registGoalPoint(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        const auto name = generateKey(regist_goal_id, param.regist_waypoint_prefix);

        waypoint_map[name].goal.x() = msg->point.x;
        waypoint_map[name].goal.y() = msg->point.y;
        waypoint_map[name].goal.z() = msg->point.z;
        waypoint_map[name].quaternion.x() = 0;
        waypoint_map[name].quaternion.y() = 0;
        waypoint_map[name].quaternion.z() = 0;
        waypoint_map[name].quaternion.w() = 1;
        waypoint_map.setQuaternion(name);

        ROS_INFO("Add waypoint %s from point", name.c_str());

        regist_goal_id.store(regist_goal_id.load() + 1);

        publishLatchedData();
    }

    void Node::eraseGoal(const std_msgs::String::ConstPtr &msg)
    {
        if (!waypoint_map.hasKey(msg->data))
        {
            ROS_INFO("Do not have waypoint id %s", msg->data.c_str());
            return;
        }
        waypoint_map.erase(msg->data);
        router.erase(msg->data);

        ROS_INFO("Removed waypoint id %s", msg->data.c_str());

        publishLatchedData();
    }

    void Node::updateGoalPose(const waypoint_manager_msgs::WaypointStamped::ConstPtr &msg)
    {
        if (param.global_frame != msg->header.frame_id)
        {
            ROS_WARN("The frame_id is different, so the goal pose is not updated");
            return;
        }
        decltype(auto) name = msg->waypoint.identity;

        waypoint_map[name].goal.x() = msg->waypoint.pose.position.x;
        waypoint_map[name].goal.y() = msg->waypoint.pose.position.y;
        waypoint_map[name].goal.z() = msg->waypoint.pose.position.z;
        waypoint_map[name].quaternion.x() = msg->waypoint.pose.orientation.x;
        waypoint_map[name].quaternion.y() = msg->waypoint.pose.orientation.y;
        waypoint_map[name].quaternion.z() = msg->waypoint.pose.orientation.z;
        waypoint_map[name].quaternion.w() = msg->waypoint.pose.orientation.w;

        for (const auto &[key, value] : msg->waypoint.properties)
        {
            waypoint_map[name].properties[key] = value;
        }
        waypoint_map.setQuaternion(name);
        publishLatchedData();
    }

    void Node::appendRoute(const std_msgs::String::ConstPtr &msg)
    {
        router.append(msg->data);

        publishLatchedData();
    }

    void Node::eraseRoute(const std_msgs::String::ConstPtr &msg)
    {
        router.erase(msg->data);

        publishLatchedData();
    }

    void Node::cmd_vel_(const geometry_msgs::Twist::ConstPtr &msg)
    {
        float vel_x;
        vel_x = msg->linear.x;
        if (use_skip)
        {
            end_skip_time = time(NULL);
            if (vel_x <= skip_vel)
            {
                if ((end_skip_time - start_skip_time) >= to_skip_sec)
                {
                    start_time = time(NULL);
                    // printf("skip count start!\n");
                    ROS_INFO("skip count start!");
                    start_skip_time = time(NULL);
                    skip_flag = true;
                }
            }
            else
            {
                start_skip_time = time(NULL);
            }
        }
    }

    void Node::obstacle_pos(const obstacle_detector::Obstacles::ConstPtr &msg)
    {
        int count = 0;
        Eigen::Vector2f distance_of_goal;
        double x, y;
        obstacle_detector::Obstacles obstacle;
        obstacle.circles = msg->circles;

        // std::cout << obstacle.circles.size() << std::endl;

        for (int i = 0; i < obstacle.circles.size(); i++)
        {
            x = obstacle.circles[i].center.x;
            y = obstacle.circles[i].center.y;

            //  judge
            distance_of_goal.x() = goal_X - x;
            distance_of_goal.y() = goal_Y - y;
            if (distance_of_goal.lpNorm<2>() < SKIP_RADIUS)
            {
                ROS_INFO("obstacle_detect!!!");
                obstacle_flag = true;
                break;
                count++;
            }
            else
            // {
            //     count++;
            // }
            // if (count >= obstacle.circles.size() - 1)
            {
                obstacle_flag = false;
                // printf("obstacle_flag false");
            }
        }
        count = 0;
    }

    void Node::waypointForwardIndex(const std_msgs::UInt8::ConstPtr &msg)
    {
        if(msg->data != router.forward_index){
            router.forward_index = msg->data;
            ROS_INFO("set nextwaypoit index: %d", router.forward_index);
        }
        else{
            ROS_WARN("nextwaypoint is already set %d", router.forward_index);
        }
    }

    void Node::insertRoute(const std_msgs::String::ConstPtr &msg)
    {
        if (!waypoint_map.hasKey(msg->data))
        {
            ROS_WARN("Do not have waypoint name");
            return;
        }
        const auto name = generateKey(regist_goal_id, param.regist_waypoint_prefix);
        waypoint_map[name].goal = waypoint_map[msg->data].goal;
        waypoint_map.setQuaternion(name);

        if (router.insertFromKey(msg->data, name, true))
        {
            ROS_INFO("Inserted route %s", name.c_str());
        }
        else
        {
            ROS_INFO("Failed insert %s", name.c_str());
        }
        publishLatchedData();
    }

    bool Node::save(
        std_srvs::TriggerRequest &request,
        std_srvs::TriggerResponse &response)
    {
        ROS_INFO("Called save()");
        waypoint_map.save(param.waypoints_file);
        router.save(param.route_file);
        return true;
    }

    bool Node::saveWaypoints(
        std_srvs::TriggerRequest &request,
        std_srvs::TriggerResponse &response)
    {
        ROS_INFO("Called saveWaypoints()");
        waypoint_map.save(param.waypoints_file);
        return true;
    }

    bool Node::saveRoute(
        std_srvs::TriggerRequest &request,
        std_srvs::TriggerResponse &response)
    {
        ROS_INFO("Called saveRoute()");
        router.save(param.route_file);
        return true;
    }

    bool Node::resetRoute(
        std_srvs::TriggerRequest &request,
        std_srvs::TriggerResponse &response)
    {
        ROS_INFO("Called resetRoute()");

        if (router.isEmpty())
        {
            ROS_WARN("Route size of zero");
            return false;
        }
        router.resetIndex();
        publishGoal();

        ROS_INFO("Reset of route current goal %s", router.getIndex().c_str());

        return true;
    };

    bool Node::switchCancel(
        std_srvs::TriggerRequest &request,
        std_srvs::TriggerResponse &response)
    {
        ROS_INFO("Called switchCancel()");
        exchangeCancelState();
        // start_clock = clock();
        start_time = time(NULL);
        return true;
    }

    bool Node::nextWaypoint(
        std_srvs::TriggerRequest &request,
        std_srvs::TriggerResponse &response)
    {
        ROS_INFO("Called nextWaypoint()");

        if (!router.forwardIndex())
        {
            ROS_WARN("Failed forward index for route");
        }
        publishGoal();

        // fall stop flag
        std_srvs::SetBool data;
        data.request.data = false;
        stop_service.call(data);

        // ClearCostmapService();

        // start_time = time(NULL);

        return true;
    }

    bool Node::prevWaypoint(
        std_srvs::TriggerRequest &request,
        std_srvs::TriggerResponse &response)
    {
        ROS_INFO("Called prevWaypoint()");

        if (!router.backIndex())
        {
            ROS_WARN("Failed back index for route");
        }
        publishGoal();

        return true;
    }

    void Node::skipWaypoint()
    {
        ROS_INFO("Called skipWaypoint()");

        if (!router.forwardIndex())
        {
            ROS_WARN("Failed forward index for route");
        }

        // start_time = time(NULL);

        publishGoal();

        std_srvs::SetBool data;
        data.request.data = false;
        stop_service.call(data);

        ClearCostmapService();
    }

    bool Node::resumeWaypoint(
        std_srvs::TriggerRequest &request,
        std_srvs::TriggerResponse &response)
    {
        ROS_INFO("Called resumeWaypoint()");
        Muxselect();

        return true;
    }

    void Node::ClearCostmapService()
    {
        ROS_INFO("Called clear_costmap_service()");
        std_srvs::Empty data;
        clear_costmap_service.call(data);
    }

    void Node::StopService()
    {
        ROS_INFO("Called stop_service()");
        std_srvs::SetBool data;
        data.request.data = true;
        stop_service.call(data);
    }

    void Node::TrafficSignService()
    {
        ROS_INFO("Called traffic_service()");

        std_srvs::SetBool data;
        if (traffic_flag)
        {
            data.request.data = true;
            traffic_service.call(data);
        }
        else
        {
            data.request.data = false;
            traffic_service.call(data);
        }
    }

    void Node::Muxselect()
    {
        ROS_INFO("Called Muxselect()");
        topic_tools::MuxSelect srv;
        std::string str;
        str = ("/nav_vel");
        srv.request.topic = str;
        white_line_service.call(srv);
    }

    void Node::detectstart()
    {
        ROS_INFO("Called detectstart()");

        std_srvs::Trigger trig;
        detect_start_service.call(trig);
    }

    void Node::switch_segmentation()
    {
        ROS_INFO("Called switch_segmentation()");

        std_srvs::SetBool data;
        if (seg_flag)
        {
            data.request.data = true;
            switch_segmentation_service.call(data);
        }
        else
        {
            data.request.data = false;
            switch_segmentation_service.call(data);
        }
    }

    void Node::detect_box()
    {
        ROS_INFO("Called detect_box()");

        std_srvs::SetBool data;
        if (detect_box_flag)
        {
            data.request.data = true;
            detect_box_service.call(data);
        }
        else
        {
            data.request.data = false;
            detect_box_service.call(data);
        }
    }

    void Node::area_select()
    {
        ROS_INFO("Called area_select()");

        std_srvs::SetBool data;
        if (area_select_flag)
        {
            data.request.data = true;
            area_select_service.call(data);
        }
        else
        {
            data.request.data = false;
            area_select_service.call(data);
        }
    }

    void Node::area_waypoint()
    {
        ROS_INFO("Called area_waypoint()");

        std_srvs::SetBool data;
        if (area_waypoint_flag)
        {
            data.request.data = true;
            area_waypoint_service.call(data);
        }
        else
        {
            data.request.data = false;
            area_waypoint_service.call(data);
        }
    }

    void Node::publishGoal()
    {
        if (is_cancel.load())
        {
            return;
        }
        if (router.isEmpty())
        {
            return;
        }
        const auto pose_vector = waypoint_map[router.getIndex()].goal;
        const auto orientation = waypoint_map[router.getIndex()].quaternion;

        const auto pose_vector_next = waypoint_map[router.getIndexNext()].goal;

        waypoint_manager_msgs::Waypoint waypoint;

        waypoint.identity = router.getIndex();
        waypoint.pose.position.x = pose_vector.x();
        waypoint.pose.position.y = pose_vector.y();
        waypoint.pose.position.z = pose_vector.z();
        waypoint.pose.orientation.x = orientation.x();
        waypoint.pose.orientation.y = orientation.y();
        waypoint.pose.orientation.z = orientation.z();
        waypoint.pose.orientation.w = orientation.w();

        goal_X = pose_vector_next.x();
        goal_Y = pose_vector_next.y();

        for (const auto &[name, value] : waypoint_map[router.getIndex()].properties)
        {
            waypoint_manager_msgs::Property property;

            property.name = name;
            property.data = value;

            waypoint.properties.push_back(property);
        }

        waypoint_publisher.publish(waypoint);
    }

    void Node::publishWaypoints()
    {
        waypoint_manager_msgs::Waypoints waypoints_msg;

        for (const auto &[key, waypoint] : waypoint_map.data())
        {
            waypoint_manager_msgs::Waypoint waypoint_msg;

            waypoint_msg.identity = key;
            waypoint_msg.pose.position.x = waypoint.goal.x();
            waypoint_msg.pose.position.y = waypoint.goal.y();
            waypoint_msg.pose.position.z = waypoint.goal.z();
            waypoint_msg.pose.orientation.x = waypoint.quaternion.x();
            waypoint_msg.pose.orientation.y = waypoint.quaternion.y();
            waypoint_msg.pose.orientation.z = waypoint.quaternion.z();
            waypoint_msg.pose.orientation.w = waypoint.quaternion.w();

            for (const auto &[name, data] : waypoint.properties)
            {
                waypoint_manager_msgs::Property property;

                property.name = name;
                property.data = data;

                waypoint_msg.properties.push_back(property);
            }
            waypoints_msg.waypoints.push_back(waypoint_msg);
        }
        waypoints_msg.info.header.frame_id = param.global_frame;
        waypoints_msg.info.header.stamp = ros::Time::now();

        waypoints_publisher.publish(waypoints_msg);
    }

    void Node::publishRoute()
    {
        waypoint_manager_msgs::Route route_msg;

        for (const auto &id : router.data())
        {
            route_msg.identities.push_back(id);
        }
        route_msg.header.frame_id = param.global_frame;
        route_msg.header.stamp = ros::Time::now();

        route_publisher.publish(route_msg);
    }

    void Node::publishLatchedData()
    {
        publishRoute();
        ros::Duration(param.wait_publish_waypoints_time).sleep();
        publishWaypoints();
    }

    void Node::exchangeCancelState()
    {
        is_cancel.store(!is_cancel.load());

        if (is_cancel.load())
        {
            ROS_INFO("Changed is_cancel true");
        }
        else
        {
            ROS_INFO("Changed is_cancel false");
        }
    }
}

auto main(int argc, char **argv) -> int
{
    ros::init(argc, argv, "waypoint_server_node");

    ROS_INFO("Start waypoint_server_node");

    waypoint_server::Node node{};
    node.spin();

    ROS_INFO("Finish waypoint_server_node");
    return 0;
}
