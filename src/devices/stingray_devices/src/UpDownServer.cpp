#include "../include/UpDownServer.h"
#include <stingray_devices_msgs/SetDeviceAction.h>


UpDownServer::UpDownServer(const std::string& actionName, const std::string& deviceActionService) :
        deviceActionService(deviceActionService),
        actionServer(nodeHandle, actionName, boost::bind(&UpDownServer::goalCallback, this, _1), false) {
    actionServer.start();
}


void UpDownServer::goalCallback(const stingray_devices_msgs::UpDownGoalConstPtr &goal) {
    stingray_devices_msgs::SetDeviceAction SetDeviceAction;

    SetDeviceAction.request.device = goal->device;

    SetDeviceAction.request.value = -goal->velocity;
    ros::service::call(deviceActionService, SetDeviceAction);
    ros::Duration(goal->pause_common).sleep();

    if(goal->pause_optional > 0){
        SetDeviceAction.request.value = 0;
        ros::service::call(deviceActionService, SetDeviceAction);
        ros::Duration(goal->pause_optional).sleep();
    }

    SetDeviceAction.request.value = goal->velocity;
    ros::service::call(deviceActionService, SetDeviceAction);
    ros::Duration(goal->pause_common).sleep();

    SetDeviceAction.request.value = 0;
    ros::service::call(deviceActionService, SetDeviceAction);

    actionServer.setSucceeded();
}