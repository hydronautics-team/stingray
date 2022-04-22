#ifndef PROJECT_UPDOWNSERVER_H
#define PROJECT_UPDOWNSERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <devices_msgs/UpDownAction.h>

class UpDownServer {

private:

    std::string deviceActionService;

    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<devices_msgs::UpDownAction> actionServer;

    void goalCallback(const devices_msgs::UpDownGoalConstPtr &goal);

public:

    UpDownServer(const std::string& actionName, const std::string& deviceActionService);
    ~UpDownServer() = default;
};

#endif //PROJECT_UPDOWNSERVER_H
