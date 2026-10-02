#include "Route.h"
#include <cmath>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "stingray_mapping/msg/map_object_array.hpp"


class PathPlannerNode : public rclcpp::Node{
    public:

        PathPlannerNode() : Node("path_planning"){
            this->declare_parameter("k_units",              20.0);
            this->declare_parameter("auv_length",            1.0);
            this->declare_parameter("auv_width",             0.4);
            this->declare_parameter("max_vel",               1.0);
            this->declare_parameter("min_vel",               0.1);
            this->declare_parameter("max_angle_vel",        60.0);
            this->declare_parameter("min_angle_vel",         5.0);
            this->declare_parameter("shutdown_on_complete", false);

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
                std::bind(&PathPlannerNode::targets_callback, this, std::placeholders::_1));
            

            pub_cmd = this -> create_publisher<geometry_msgs::msg::Twist>("/core/cmd/velocity", 10);
            pub_path = this -> create_publisher<nav_msgs::msg::Path>("/planner/global_path", 10);
        }
        
    private:

        double k_units;
        double auv_length;
        double auv_width;
        double max_vel;
        double min_vel;
        double max_angle_vel;
        double min_angle_vel;

        bool got_odom = false;
        bool got_map = false;
        bool got_target = false;
        bool executed = false;

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
            try_run();
        }

        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
            if (got_map) return;

            grid = std::make_unique<GRID>(static_cast<int>(msg->info.width),
                                          static_cast<int>(msg->info.height),
                                          std::vector<Point>{},
                                          std::vector<Point>{});
            for (int i = 0; i < msg->data.size(); i++){
                if (msg->data[i] > 50){
                    grid->field[i] = 1;
                }
                else{
                    grid->field[i] = 0;
                }
            }
            got_map = true;
            try_run();
        }

        void targets_callback(const stingray_mapping::msg::MapObjectArray::SharedPtr msg){
            if (got_target) return;

            const double x_m = msg->pose.position.x;
            const double y_m = msg->pose.position.y;
            target = {static_cast<int>(x_m * k_units), static_cast<int>(y_m * k_units)};
            got_target = true;
            try_run();
        }

        void try_run(){
            if (executed) return;
            if (!got_odom || !got_map || !got_target) return;
            executed = true;

            AUV VELT(start_point,
                     metres_to_grid_units(auv_length, k_units),
                     metres_to_grid_units(auv_width, k_units),
                     metres_to_grid_units(max_vel, k_units),
                     metres_to_grid_units(min_vel, k_units),
                     max_angle_vel,
                     min_angle_vel);
            std::vector<Point> targets = {target};
            std::vector<Point> path = VELT.build_full_route(targets, *grid);
            if (path.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Path not found");
                finish();
                return;
            }
            auto cmds = compute_commands(path, max_vel, min_vel, max_angle_vel, min_angle_vel, k_units);

             publish_path(path);
             publish_velocity(path, cmds);
             finish();
        }

        void finish(){
            if (this->get_parameter("shutdown_on_complete").as_bool()) {
                RCLCPP_INFO(this->get_logger(), "Shutting down (shutdown_on_complete=true)");
                rclcpp::shutdown();
            }
        }

        void publish_path(const std::vector<Point> & path){
            nav_msgs::msg::Path msg;
            msg.header.stamp    = this->now();
            msg.header.frame_id = "map";
            msg.poses.reserve(path.size());

            for (const auto & p : path) {
                geometry_msgs::msg::PoseStamped ps;
                ps.header = msg.header;
                ps.pose.position.x = static_cast<double>(p.x) / k_units;
                ps.pose.position.y = static_cast<double>(p.y) / k_units;
                ps.pose.position.z = 0.0;
                ps.pose.orientation.w = 1.0;
                msg.poses.push_back(ps);
            }
            pub_path->publish(msg);
        }

        void publish_velocity(const std::vector<Point> & path, const std::vector<WaypointCommand> & cmds){
            geometry_msgs::msg::Twist msg;
            if (cmds.empty() || path.size() < 2){
                pub_cmd->publish(msg);
                return;
            }
            const auto & c = cmds[0];
            msg.linear.x = c.x_vel;
            msg.linear.y = c.y_vel;
            msg.linear.z = 0.0;
            msg.angular.x = 0.0;
            msg.angular.y =  0.0;
            msg.angular.z = c.angle_vel * M_PI / 180.0;

            pub_cmd->publish(msg);
        }

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map;
        rclcpp::Subscription<stingray_mapping::msg::MapObjectArray>::SharedPtr sub_targets;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;


};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
