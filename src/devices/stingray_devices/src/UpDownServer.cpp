#include "../include/UpDownServer.h"
#include <stingray_devices_msgs/SetDeviceAction.h>


UpDownServer::UpDownServer(const std::string& actionName, const std::string& deviceActionService) :
        deviceActionService(deviceActionService),
        actionServer(nodeHandle, actionName, boost::bind(&UpDownServer::goalCallback, this, _1), false) {
    actionServer.start();
}


void UpDownServer::goalCallback(const stingray_devices_msgs::UpDownGoalConstPtr &goal) {
    //    TODO: load actual device ID from config. Current: dropper - 1, lifter - 2

    stingray_devices_msgs::SetDeviceAction SetDeviceAction;

    SetDeviceAction.request.device = goal->device;
    ROS_INFO("Activating %d device", goal->device);
    ROS_INFO("Force sent: %d", goal->velocity);
    SetDeviceAction.request.value = goal->velocity;
    ros::service::call(deviceActionService, SetDeviceAction);

    ros::Duration(goal->pause_common).sleep();

    if(goal->pause_optional > 0){
        ROS_INFO("Keeping lifter down for %f ms", goal->pause_optional);
        SetDeviceAction.request.value = 0;
        ros::service::call(deviceActionService, SetDeviceAction);
        ros::Duration(goal->pause_optional).sleep();
    }
    ROS_INFO("Lifting back...");
    SetDeviceAction.request.value = -goal->velocity;
    ros::service::call(deviceActionService, SetDeviceAction);
    ros::Duration(goal->pause_common).sleep();

    SetDeviceAction.request.value = 0;
    ros::service::call(deviceActionService, SetDeviceAction);

    ROS_INFO("Lifting done");

    actionServer.setSucceeded();
}
