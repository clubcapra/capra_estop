#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "JetsonXavierGPIO/jetsonGPIO.h"
#include <signal.h>
#include <ros/xmlrpc_manager.h>

bool estop_value = true; //default value for the Estop.
const jetsonXavierGPIONumber ESTOP_PIN = jetsonXavierGPIONumber::gpio428;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

/**
 * Call back function for to toggle the Estop pin. This will change the electric output on the pin and enable or disable
 * the estop.
 */
bool toggleEstopEnable(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.message = "successfully toggle estop to on";
    gpioSetValue(ESTOP_PIN, 1);
    res.success = static_cast<unsigned char>(true);
    return true;
}

/**
 * Call back function for to toggle the Estop pin. This will change the electric output on the pin and enable or disable
 * the estop.
 */
bool toggleEstopDisable(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.message = "successfully toggle estop to off";
    gpioSetValue(ESTOP_PIN, 0);
    res.success = static_cast<unsigned char>(true);
    return true;
}

/**
 * This function is called before the node is killed. It disables the estop and shutdown the node. 
*/
void sigintShutdownNode(int sig)
{
    gpioSetValue(ESTOP_PIN, 0);
    ros::shutdown();
    g_request_shutdown = 1; // Set flag
}

/**
 * This function is called before the node is killed using rosnode kill. It disables the estop and shutdown the node. 
*/
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    gpioSetValue(ESTOP_PIN, 0);
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
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

    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

    signal(SIGINT, sigintShutdownNode);

    ros::ServiceServer serviceEnable = nh.advertiseService("estop_enable", toggleEstopEnable);
    ros::ServiceServer serviceDisable = nh.advertiseService("estop_disable", toggleEstopDisable);
    
    ros::Rate rate(100); // ROS Rate at 100Hz

    // Do our own spin loop
    while (!g_request_shutdown)
    {
      ros::spinOnce();
      rate.sleep();
    }

    return 0;
}