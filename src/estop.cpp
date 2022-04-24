#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "JetsonXavierGPIO/jetsonGPIO.h"
#include <signal.h>
#include <std_msgs/Int64.h>

bool estop_value = true; //default value for the Estop.
const jetsonXavierGPIONumber ESTOP_PIN = jetsonXavierGPIONumber::gpio428;

const jetsonXavierGPIONumber ESTOP_STATUS_PIN=jetsonXavierGPIONumber::gpio256;
unsigned int* value;
static ros::Publisher e_stop_status_pub;

void advertiseEstopStatus()
{
    std_msgs::Int64 msg;
    gpioGetValue(ESTOP_STATUS_PIN,value);
    msg.data = (*value);  
    e_stop_status_pub.publish(msg);

}

/**
 * Call back function for to toggle the Estop pin. This will change the logic level on the pin and enable the robot.
 */
bool toggleEstopEnable(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.message = "successfully toggle estop to on";
    gpioSetValue(ESTOP_PIN, 1);
    res.success = static_cast<unsigned char>(true);
    advertiseEstopStatus();
    return true;
}

/**
 * Call back function for to toggle the Estop pin. This will change the logic level on the pin and disable the robot drives.
 */
bool toggleEstopDisable(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.message = "successfully toggle estop to off";
static ros::Publisher e_stop_status_pub;
    gpioSetValue(ESTOP_PIN, 0);
    res.success = static_cast<unsigned char>(true);
    advertiseEstopStatus();
    return true;
}

/**
 * This function is called before the node is killed. It disables the estop and shutdown the node. 
*/
void sigintShutdownNode(int sig)
{
    gpioSetValue(ESTOP_PIN, 0);
    ros::shutdown();
}

/**
 * Initialize the configuration to control the GPIO pin.
 */
void initializeGPIO()
{
    gpioExport(ESTOP_PIN);
    gpioSetDirection(ESTOP_PIN, 1);
    gpioSetValue(ESTOP_PIN, 1);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "capra_estop", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    initializeGPIO();

    signal(SIGINT, sigintShutdownNode);
    e_stop_status_pub = nh.advertise<std_msgs::Int64>("e_stop_is_pressed",1000);

    ros::ServiceServer serviceEnable = nh.advertiseService("estop_enable", toggleEstopEnable);
    ros::ServiceServer serviceDisable = nh.advertiseService("estop_disable", toggleEstopDisable);
    
    ros::spin();

    return 0;
}