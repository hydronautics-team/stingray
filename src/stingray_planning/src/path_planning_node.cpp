#include <Route.h>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "stingray_mapping/msg/map_object_array.hpp"


class PathPlannerNode : public rclcpp::Node{
    public:

        PathPlanner() = Node("path_planning"){
            this->declare_parameter("k_units",              20.0);
            this->declare_parameter("auv_length",            1.0);
            this->declare_parameter("auv_width",             0.4);
            this->declare_parameter("max_vel",               1.0);
            this->declare_parameter("min_vel",               0.1);
            this->declare_parameter("max_angle_vel",        60.0);
            this->declare_parameter("min_angle_vel",         5.0);

            k_units = this->get_parameter("k_units").as_double();
            auv_length = this->get_parameter("auv_length").as_double();
            auv_width = this->get_parameter("auv_width").as_double();
            max_vel = this->get_parameter("max_vel").as_double();
            min_vel = this->get_parameter("min_vel").as_double();
            max_angle_vel = this->get_parameter("max_angle_vel").as_double();
            min_angle_vel = this->get_parameter("min_angle_vel").as_double();


            sub_odometry = this -> create_subscription<nav_msgs::msg::Odometry>(
                "/core/state/odometry", 10, 
                std::bind(&PathPlannerNode::odometry_callback, this, std::placeholders::_1));
            sub_map = this -> create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map/occupancy_local", 10,
                std::bind(&PathPlannerNode::map_callback, this, std::placeholders::_1));
            sub_targets = this -> create_subscription<stingray_mapping::msg::MapObjectArray>(
                "/map/objects", 10,
                std::bind($PathPlannerNode::targets_callback, this, std::placeholders::_1));
            

            pub_cmd = this -> create_publisher<geometry_msgs::msg::Twist>("/core/cmd/velocity", 10);
            pub_path = this -> create_publisher<nav_msgs::msg::Pathe>("/planner/global_path", 10);
        
    private:
        
        double k_units;
        double auv_length;
        double auv_width;
        double max_vel;
        double min_vel;
        double max_angle_vel;
        double min_angle_vel;
        double resolution = 0.05;
        bool got_odom = false;
        bool got_map = false;
        bool got_objects = false;  
        Point start_point{0, 0};
        Point target{0, 0};
        std::unique_ptr<GRID> grid;

        void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            if (got_odom) return;

            const double x_m = msg->pose.pose.position.x;
            const double y_m = msg->pose.pose.position.y;
            start_point = {
                static_cast<int>(x_m * k_units),
                static_cast<int>(y_m * k_units)
            };

            got_odom = true;
            RCLCPP_INFO(this->get_logger(),
            "Odometry received: start=(%d, %d)", start_point_.x, start_point_.y);

            try_run();
        }

    }
}
