#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include "pizza_msgs/place_in_box.h"
#include "pizza_msgs/cut.h"
#include "pizza_msgs/pack_pizza.h"
#include "pizza_msgs/change_tool.h"

// Status
#define WAITING         0
#define PLACE_IN_BOX      1
#define FINISH_PLACE_IN_BOX      11
#define CUT     2
#define FINISH_CUT     22
#define PACK_PIZZA     3
#define FINISH_PACK_PIZZA     33
#define TOOL     4
#define FINISH_TOOL    44
#define TIME_PLACE_IN_BOX 10
#define TIME_CUT 10
#define TIME_PACK_PIZZA 5
#define TIME_TOOL 5

std_msgs::Int8 robot_state;
std::string log_r = "[R3]";

bool place_in_box(pizza_msgs::place_in_box::Request  &req,  pizza_msgs::place_in_box::Response &res)
{
    bool data;
    data = req.request;
    robot_state.data = PLACE_IN_BOX;
    res.ack = 1;
    return true;
}

bool cut(pizza_msgs::cut::Request  &req, pizza_msgs::cut::Response &res)
{
    bool data;
    data = req.request;
    robot_state.data = CUT;
    res.ack = 1;
    return true;
}

bool pack_pizza(pizza_msgs::pack_pizza::Request  &req, pizza_msgs::pack_pizza::Response &res)
{
    bool data;
    data = req.request;
    robot_state.data = PACK_PIZZA;
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
    ros::init(argc, argv, "R3");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(2);

    // Service server
    ros::ServiceServer place_in_box_server = n.advertiseService("place_in_box", place_in_box);
    ros::ServiceServer cut_server = n.advertiseService("cut", cut);
    ros::ServiceServer pack_pizza_server = n.advertiseService("pack_pizza", pack_pizza);
    ros::ServiceServer change_tool_server = n.advertiseService("change_tool", change_tool);

    // Publisher
    ros::Publisher state_publisher = n.advertise<std_msgs::Int8>("state", 1000);
    robot_state.data = WAITING;
    int process_time = 0;

    while (ros::ok())
    {
        switch(robot_state.data)
        {
        case WAITING:
            break;
        case PLACE_IN_BOX:
            if (process_time < TIME_PLACE_IN_BOX)
            {
                process_time++;
                ROS_INFO_STREAM(log_r << " Placing pizza in box " << process_time);
            }
            else
            {
                process_time = 0;
                robot_state.data = FINISH_PLACE_IN_BOX;
            }
            break;
        case CUT:
            if (process_time < TIME_CUT)
            {
                process_time++;
                ROS_INFO_STREAM(log_r <<" Cutting pizza " << process_time);
            }
            else
            {
                process_time = 0;
                robot_state.data = FINISH_CUT;
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
        case PACK_PIZZA:
            if (process_time < TIME_PACK_PIZZA)
            {
                process_time++;
                ROS_INFO_STREAM(log_r <<" Packing pizza " << process_time);
            }
            else
            {
                process_time = 0;
                robot_state.data = FINISH_PACK_PIZZA;
            }
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();

        // Publish robot state
        state_publisher.publish(robot_state);
    }
}
