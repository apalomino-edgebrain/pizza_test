/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>

#include "../include/pizza_test/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace pizza_test {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

/**
* @brief Callbacks state
* @param state
*
******************************************************************/
void QNode::R1_state_callback(const std_msgs::Int8::ConstPtr& msg)
{
    R1_state.data = msg->data;
    system_status("R1", R1_state.data);
}
void QNode::R2_state_callback(const std_msgs::Int8::ConstPtr& msg)
{
    R2_state.data = msg->data;
    system_status("R2", R2_state.data);
}
void QNode::R3_state_callback(const std_msgs::Int8::ConstPtr& msg)
{
    R3_state.data = msg->data;
    system_status("R3", R3_state.data);
}
void QNode::R4_state_callback(const std_msgs::Int8::ConstPtr& msg)
{
    R4_state.data = msg->data;
    system_status("R4", R4_state.data);
}


bool QNode::init() {
	ros::init(init_argc,init_argv,"pizza_test");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
    // Add your ros communications here.7
    // Publishers
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);    

    // Service clients
    R1_spread_tomato_client = n.serviceClient<pizza_msgs::spread_tomato>("/R1/spread_tomato");
    R1_scatter_cheese_client = n.serviceClient<pizza_msgs::scatter_cheese>("/R1/scatter_cheese");
    R1_move_to_oven_client = n.serviceClient<pizza_msgs::move_to_oven>("/R1/move_to_oven");
    R1_change_tool_client = n.serviceClient<pizza_msgs::change_tool>("/R1/change_tool");

    R2_spread_tomato_client = n.serviceClient<pizza_msgs::spread_tomato>("/R2/spread_tomato");
    R2_scatter_cheese_client = n.serviceClient<pizza_msgs::scatter_cheese>("/R2/scatter_cheese");
    R2_move_to_oven_client = n.serviceClient<pizza_msgs::move_to_oven>("/R2/move_to_oven");
    R2_change_tool_client = n.serviceClient<pizza_msgs::change_tool>("/R2/change_tool");

    R3_place_in_box_client = n.serviceClient<pizza_msgs::place_in_box>("/R3/place_in_box");
    R3_slice_pizza_client = n.serviceClient<pizza_msgs::cut>("/R3/cut");
    R3_pack_pizza_client = n.serviceClient<pizza_msgs::pack_pizza>("/R3/pack_pizza");
    R3_change_tool_client = n.serviceClient<pizza_msgs::change_tool>("/R3/change_tool");

    R4_place_in_box_client = n.serviceClient<pizza_msgs::place_in_box>("/R4/place_in_box");
    R4_slice_pizza_client = n.serviceClient<pizza_msgs::cut>("/R4/cut");
    R4_pack_pizza_client = n.serviceClient<pizza_msgs::pack_pizza>("/R4/pack_pizza");
    R4_change_tool_client = n.serviceClient<pizza_msgs::change_tool>("/R4/change_tool");

    // Subscribers
    R1_state_subscriber = n.subscribe("/R1/state", 1, &QNode::R1_state_callback, this);
    R2_state_subscriber = n.subscribe("/R2/state", 1, &QNode::R2_state_callback, this);
    R3_state_subscriber = n.subscribe("/R3/state", 1, &QNode::R3_state_callback, this);
    R4_state_subscriber = n.subscribe("/R4/state", 1, &QNode::R4_state_callback, this);

    // Start node
	start();
	return true;
}

void QNode::process_control(std::vector<bool> pizza_ordered)
{
    pizza_queue = pizza_ordered;
    ROS_INFO_STREAM(" Number of pizzas " << pizza_queue.size());

}

void QNode::process_start(std::vector<bool> &pizza_ordered)
{
    spread_tomato_sauce(R1_spread_tomato_client, "R1");
    pizza_ordered.erase(pizza_ordered.begin(), pizza_ordered.end());
}

/**
* @brief Callbacks clients
*
******************************************************************/
bool QNode::spread_tomato_sauce(ros::ServiceClient robot_spread_tomato_client, std::string robot){
    pizza_msgs::spread_tomato srv;
    srv.request.request = true;

    if (robot_spread_tomato_client.call(srv))
    {
        ROS_INFO_STREAM(log_info  <<  " " << robot << "       Called: spread_tomato_sauce");
        return 1;
    }
    else
    {
        ROS_ERROR_STREAM(log_info  <<  " " << robot << "      Failed to call: spread_tomato_sauce");
        return 0;
    }
}

bool QNode::scatter_cheese(ros::ServiceClient robot_scatter_cheese_client, std::string robot){
    pizza_msgs::scatter_cheese srv;
    srv.request.request = true;

    if (robot_scatter_cheese_client.call(srv))
    {
        ROS_INFO_STREAM(log_info <<  " " << robot << "       Called: scatter_cheese");
        return 1;
    }
    else
    {
        ROS_ERROR_STREAM(log_info << " " << robot <<"      Failed to call: scatter_cheese");
        return 0;
    }
}

bool QNode::change_tool(ros::ServiceClient robot_change_tool_client, std::string robot){
    pizza_msgs::change_tool srv;
    srv.request.request = true;

    if (robot_change_tool_client.call(srv))
    {
        ROS_INFO_STREAM(log_info << " " << robot << "       Called: change_tool");
        return 1;
    }
    else
    {
        ROS_ERROR_STREAM(log_info << " " << robot << "      Failed to call: change_tool");
        return 0;
    }
}

bool QNode::move_to_oven(ros::ServiceClient robot_move_to_oven_client, std::string robot){
    pizza_msgs::move_to_oven srv;
    srv.request.request = true;

    if (robot_move_to_oven_client.call(srv))
    {
        ROS_INFO_STREAM(log_info << " " << robot << "       Called: move_to_oven");
        return 1;
    }
    else
    {
        ROS_ERROR_STREAM(log_info << " " << robot << "      Failed to call: move_to_oven");
        return 0;
    }
}

bool QNode::place_in_box(ros::ServiceClient robot_place_in_box_client, std::string robot){
    pizza_msgs::place_in_box srv;
    srv.request.request = true;

    if (robot_place_in_box_client.call(srv))
    {
        ROS_INFO_STREAM(log_info << " " << robot << "       Called: place_in_box");
        return 1;
    }
    else
    {
        ROS_ERROR_STREAM(log_info << " " << robot << "      Failed to call: place_in_box");
        return 0;
    }
}

bool QNode::slice_pizza(ros::ServiceClient robot_slice_pizza_client, std::string robot){
    pizza_msgs::cut srv;
    srv.request.request = true;

    if (robot_slice_pizza_client.call(srv))
    {
        ROS_INFO_STREAM(log_info << " " << robot << "       Called: slice_pizza");
        return 1;
    }
    else
    {
        ROS_ERROR_STREAM(log_info << " " << robot << "      Failed to call: slice_pizza");
        return 0;
    }
}

bool QNode::pack_pizza(ros::ServiceClient robot_pack_pizza_client, std::string robot){
    pizza_msgs::pack_pizza srv;
    srv.request.request = true;

    if (robot_pack_pizza_client.call(srv))
    {
        ROS_INFO_STREAM(log_info << " " << robot << "       Called: pack_pizza");
        return 1;
    }
    else
    {
        ROS_ERROR_STREAM(log_info << " " << robot << "      Failed to call: pack_pizza");
        return 0;
    }
}

/**
* @brief System control by robot states
*
******************************************************************/
void QNode::system(){

    switch(R1_state.data)
    {
    case WAITING:
        // Actions
        break;
    case TOMATO:
        // Actions
        called = false;
        break;
    case FINISH_TOMATO:
        // Actions
        if (!called)
        {
            change_tool(R1_change_tool_client, "R1");
            scatter_cheese(R2_scatter_cheese_client, "R2");
            called = true;
        }
        break;
    case CHEESE:        
        // Actions
        called = false;
        break;
    case FINISH_CHEESE:
        // Actions
        if (!called)
        {
            change_tool(R1_change_tool_client, "R1");
            move_to_oven(R2_move_to_oven_client, "R2");
            called = true;
        }
        break;
    case TOOL:
        // Actions
        called = false;
        break;
    case FINISH_TOOL:
        // Actions
        break;
    case OVEN:
        // Actions
        called = false;
        break;
    case FINISH_OVEN:
        // Actions
        if (!called)
        {
            change_tool(R1_change_tool_client, "R1");
            pizza_queue.erase(pizza_queue.begin());
            oven_queue.push_back(true);
            ROS_INFO_STREAM(" Pizza remained " << pizza_queue.size());
            if (pizza_queue.size() != 0)
            {
                spread_tomato_sauce(R2_spread_tomato_client, "R2");                
            }
            place_in_box(R3_place_in_box_client, "R3");
            called = true;
        }
        break;
    }

    //// ROBOT 2
    switch(R2_state.data)
    {
    case WAITING:
        // Actions
        break;
    case TOMATO:
        // Actions
        called_2 =false;
        break;
    case FINISH_TOMATO:
        // Actions
        if (!called_2)
        {
            change_tool(R2_change_tool_client, "R2");
            scatter_cheese(R1_scatter_cheese_client, "R1");
            called_2 = true;
        }
        break;
    case CHEESE:
        // Actions
        called_2 =false;
        break;
    case FINISH_CHEESE:        
        // Actions
        if (!called_2)
        {
            change_tool(R2_change_tool_client, "R2");
            move_to_oven(R1_move_to_oven_client, "R1");
            called_2 = true;
        }
        break;
    case TOOL:
        // Actions
        called_2 =false;
        break;
    case FINISH_TOOL:        
        // Actions
        break;
    case OVEN:        
        // Actions
        called_2 =false;
        break;
    case FINISH_OVEN:
        // Actions
        if (!called_2)
        {
            change_tool(R2_change_tool_client, "R2");
            pizza_queue.erase(pizza_queue.begin());
            oven_queue.push_back(true);
            ROS_INFO_STREAM(" Pizza remained " << pizza_queue.size());
            if (pizza_queue.size() != 0 )
            {
                spread_tomato_sauce(R1_spread_tomato_client, "R1");
            }
            called_2 = true;
        }
        break;
    }

    //// ROBOT 3
    switch(R3_state.data)
    {
    case WAITING:
        // Actions
        break;
    case PLACE_IN_BOX:
        // Actions
        called_3 = false;
        break;
    case FINISH_PLACE_IN_BOX:
        // Actions
        if (!called_3)
        {
            change_tool(R3_change_tool_client, "R3");
            slice_pizza(R4_slice_pizza_client, "R4");
            called_3 = true;
        }
        break;
    case CUT:
        // Actions
        called_3 = false;
        break;
    case FINISH_CUT:
        // Actions
        if (!called_3)
        {
            change_tool(R3_change_tool_client, "R3");
            pack_pizza(R4_pack_pizza_client, "R4");
            called_3 = true;
        }
        break;
    case TOOL:
        // Actions
        called_3 = false;
        break;
    case FINISH_TOOL:
        // Actions
        break;
    case PACK_PIZZA:
        // Actions
        called_3 =false;
        break;
    case FINISH_PACK_PIZZA:
        // Actions
        if (!called_3)
        {
            change_tool(R3_change_tool_client, "R3");
            oven_queue.erase(oven_queue.begin());
            packed_queue.push_back(true);
            ROS_INFO_STREAM(" Oven remained " << oven_queue.size());
            if (oven_queue.size() != 0 )
            {
                place_in_box(R4_place_in_box_client, "R4");
            }
            called_3 = true;
        }

        break;
    }

    //// ROBOT 4
    switch(R4_state.data)
    {
    case WAITING:
        // Actions
        break;
    case PLACE_IN_BOX:
        // Actions
        called_4 =false;
        break;
    case FINISH_PLACE_IN_BOX:
        // Actions
        if (!called_4)
        {
            change_tool(R4_change_tool_client, "R4");
            slice_pizza(R3_slice_pizza_client, "R3");
            called_4 = true;
        }
        break;
    case CUT:
        // Actions
        called_4 =false;
        break;
    case FINISH_CUT:
        // Actions
        if(!called_4)
        {
            change_tool(R4_change_tool_client, "R4");
            pack_pizza(R3_pack_pizza_client, "R3");
            called_4 = true;
        }
        break;
    case TOOL:
        // Actions
        called_4 =false;
        break;
    case FINISH_TOOL:
        // Actions
        break;
    case PACK_PIZZA:
        // Actions
        called_4 =false;
        break;
    case FINISH_PACK_PIZZA:
        // Actions        
        if (!called_4)
        {
            change_tool(R4_change_tool_client, "R4");
            oven_queue.erase(oven_queue.begin());
            packed_queue.push_back(true);
            ROS_INFO_STREAM(" Oven remained " << oven_queue.size());
            if (oven_queue.size() != 0 )
            {
                place_in_box(R3_place_in_box_client, "R3");
            }
            called_4 =true;
        }
        break;
    }

    pizza_number("Order",pizza_queue.size());
    pizza_number("Oven", oven_queue.size());
    pizza_number("Packed", packed_queue.size());
}

/**
* @brief Run threat
*
******************************************************************/
void QNode::run() {
    ros::Rate loop_rate(10);	

	while ( ros::ok() ) {

		ros::spinOnce();
		loop_rate.sleep();

        // System control
        system();
	}

	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


}  // namespace pizza_test
