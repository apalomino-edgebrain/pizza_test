/**
 * @file /include/pizza_test/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef pizza_test_QNODE_HPP_
#define pizza_test_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#include "pizza_msgs/spread_tomato.h"
#include "pizza_msgs/scatter_cheese.h"
#include "pizza_msgs/move_to_oven.h"
#include "pizza_msgs/change_tool.h"
#include "pizza_msgs/place_in_box.h"
#include "pizza_msgs/cut.h"
#include "pizza_msgs/pack_pizza.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace pizza_test {

/*****************************************************************************
** Class
*****************************************************************************/

// Status
#define WAITING         0
#define TOMATO      1
#define FINISH_TOMATO      11
#define CHEESE     2
#define FINISH_CHEESE     22
#define OVEN     3
#define FINISH_OVEN     33

#define PLACE_IN_BOX      1
#define FINISH_PLACE_IN_BOX      11
#define CUT     2
#define FINISH_CUT     22
#define PACK_PIZZA     3
#define FINISH_PACK_PIZZA     33

#define TOOL     4
#define FINISH_TOOL    44

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    void run();

    // pizza numbers
    std::vector<bool> pizza_queue;
    std::vector<bool> oven_queue;
    std::vector<bool> packed_queue;
    // Funtions
    void process_control(std::vector<bool> pizza_ordered);
    void process_start(std::vector<bool> &pizza_ordered);

Q_SIGNALS:    
    void rosShutdown();
    void system_status(QString robot, int state);
    void pizza_number(QString phase, int number);

private:
    int init_argc;
    char** init_argv;
    std::string log_info = "[Control] ";
    bool called = false;
    bool called_2 = false;
    bool called_3 = false;
    bool called_4 = false;
    std_msgs::Int8 R1_state;
    std_msgs::Int8 R2_state;
    std_msgs::Int8 R3_state;
    std_msgs::Int8 R4_state;

    /** Publishers */
    ros::Publisher chatter_publisher;
    ros::Publisher new_order_publisher;

    /** Subscribers */
    ros::Subscriber R1_state_subscriber;
    ros::Subscriber R2_state_subscriber;
    ros::Subscriber R3_state_subscriber;
    ros::Subscriber R4_state_subscriber;

    /** Service Clients */
    ros::ServiceClient R1_spread_tomato_client;
    ros::ServiceClient R1_scatter_cheese_client;
    ros::ServiceClient R1_move_to_oven_client;
    ros::ServiceClient R1_change_tool_client;

    ros::ServiceClient R2_spread_tomato_client;
    ros::ServiceClient R2_scatter_cheese_client;
    ros::ServiceClient R2_move_to_oven_client;
    ros::ServiceClient R2_change_tool_client;

    ros::ServiceClient R3_place_in_box_client;
    ros::ServiceClient R3_slice_pizza_client;
    ros::ServiceClient R3_pack_pizza_client;
    ros::ServiceClient R3_change_tool_client;

    ros::ServiceClient R4_place_in_box_client;
    ros::ServiceClient R4_slice_pizza_client;
    ros::ServiceClient R4_pack_pizza_client;
    ros::ServiceClient R4_change_tool_client;

    // System control
    void system();

    /** Callback functions */
    void R1_state_callback(const std_msgs::Int8::ConstPtr& msg);
    void R2_state_callback(const std_msgs::Int8::ConstPtr& msg);
    void R3_state_callback(const std_msgs::Int8::ConstPtr& msg);
    void R4_state_callback(const std_msgs::Int8::ConstPtr& msg);

    /** Service calls */
    bool spread_tomato_sauce(ros::ServiceClient robot_spread_tomato_client, std::string robot);
    bool scatter_cheese(ros::ServiceClient robot_scatter_cheese_client, std::string robot);
    bool move_to_oven(ros::ServiceClient robot_move_to_oven_client, std::string robot);
    bool change_tool(ros::ServiceClient robot_change_tool_client, std::string robot);
    bool place_in_box(ros::ServiceClient robot_place_in_box_client, std::string robot);
    bool slice_pizza(ros::ServiceClient robot_slice_pizza_client, std::string robot);
    bool pack_pizza(ros::ServiceClient robot_pack_pizza_client, std::string robot);


};

}  // namespace pizza_test

#endif /* pizza_test_QNODE_HPP_ */
