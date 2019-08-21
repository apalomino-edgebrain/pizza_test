#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include "pizza_msgs/spread_tomato.h"
#include "pizza_msgs/scatter_cheese.h"
#include "pizza_msgs/move_to_oven.h"
#include "pizza_msgs/change_tool.h"

// Status
#define WAITING         0
#define TOMATO      1
#define FINISH_TOMATO      11
#define CHEESE     2
#define FINISH_CHEESE     22
#define OVEN     3
#define FINISH_OVEN     33
#define TOOL     4
#define FINISH_TOOL    44
#define TIME_TOMATO 10
#define TIME_CHEESE 10
#define TIME_OVEN 5
#define TIME_TOOL 5

std_msgs::Int8 robot_state;
std::string log_r = "[R2]";

bool spread_tomato_sauce(pizza_msgs::spread_tomato::Request  &req,  pizza_msgs::spread_tomato::Response &res)
{
    bool data;
    data = req.request;
    robot_state.data = TOMATO;
    res.ack = 1;
    return true;
}

bool scatter_cheese(pizza_msgs::scatter_cheese::Request  &req, pizza_msgs::scatter_cheese::Response &res)
{
    bool data;
    data = req.request;
    robot_state.data = CHEESE;
    res.ack = 1;
    return true;
}

bool move_to_oven(pizza_msgs::move_to_oven::Request  &req, pizza_msgs::move_to_oven::Response &res)
{
    bool data;
    data = req.request;
    robot_state.data = OVEN;
    res.ack = 1;
    return true;
}

bool change_tool(pizza_msgs::change_tool::Request  &req, pizza_msgs::change_tool::Response &res)
{
    bool data;
    data = req.request;
    robot_state.data = TOOL;
    res.ack = 1;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "R2");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(2);

    // Service server
    ros::ServiceServer spread_tomato_server = n.advertiseService("spread_tomato", spread_tomato_sauce);
    ros::ServiceServer scatter_cheese_server = n.advertiseService("scatter_cheese", scatter_cheese);
    ros::ServiceServer move_to_oven_server = n.advertiseService("move_to_oven", move_to_oven);
    ros::ServiceServer change_tool_server = n.advertiseService("change_tool", change_tool);

    // Publisher
    ros::Publisher state_publisher = n.advertise<std_msgs::Int8>("state", 1000);
    robot_state.data = WAITING;
    int process_time = 0;

    while (ros::ok())
    {
        // Publish robot state
        state_publisher.publish(robot_state);

        switch(robot_state.data)
        {
        case WAITING:
            break;
        case TOMATO:
            if (process_time < TIME_TOMATO)
            {
                process_time++;
                ROS_INFO_STREAM(log_r << " Spreading tomato sauce " << process_time);
            }
            else
            {
                process_time = 0;
                robot_state.data = FINISH_TOMATO;
            }
            break;
        case CHEESE:
            if (process_time < TIME_CHEESE)
            {
                process_time++;
                ROS_INFO_STREAM(log_r <<" Scattering cheese " << process_time);
            }
            else
            {
                process_time = 0;
                robot_state.data = FINISH_CHEESE;
            }
            break;
        case TOOL:
            if (process_time < TIME_TOOL)
            {
                process_time++;
                ROS_INFO_STREAM(log_r <<" Changing tool " << process_time);
            }
            else
            {
                process_time = 0;
                robot_state.data = FINISH_TOOL;
            }
            break;
        case OVEN:
            if (process_time < TIME_OVEN)
            {
                process_time++;
                ROS_INFO_STREAM(log_r <<" Moving to oven " << process_time);
            }
            else
            {
                process_time = 0;
                robot_state.data = FINISH_OVEN;
            }
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();


    }
}
