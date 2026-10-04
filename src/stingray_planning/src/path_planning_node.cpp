#include "stingray_planning/Route.hpp"
#include <cmath>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
//#include "stingray_mapping/msg/map_object_array.hpp"

namespace stingray::planning{
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

                k_units_ = this->get_parameter("k_units").as_double();
                auv_length_ = this->get_parameter("auv_length").as_double();
                auv_width_ = this->get_parameter("auv_width").as_double();
                max_vel_ = this->get_parameter("max_vel").as_double();
                min_vel_ = this->get_parameter("min_vel").as_double();
                max_angle_vel_ = this->get_parameter("max_angle_vel").as_double();
                min_angle_vel_ = this->get_parameter("min_angle_vel").as_double();


                sub_odometry = this -> create_subscription<nav_msgs::msg::Odometry>(
                    "/core/state/odometry", 10, 
                    std::bind(&PathPlannerNode::odometry_callback, this, std::placeholders::_1));
                sub_map = this -> create_subscription<nav_msgs::msg::OccupancyGrid>(
                    "/map/occupancy_local", 10,
                    std::bind(&PathPlannerNode::map_callback, this, std::placeholders::_1));
                // sub_targets = this -> create_subscription<stingray_mapping::msg::MapObjectArray>(
                //     "/map/objects", 10,
                //     std::bind(&PathPlannerNode::targets_callback, this, std::placeholders::_1));
                

                pub_cmd = this -> create_publisher<geometry_msgs::msg::Twist>("/core/cmd/velocity", 10);
                pub_path = this -> create_publisher<nav_msgs::msg::Path>("/planner/global_path", 10);
            }
            
        private:

            double k_units_;
            double auv_length_;
            double auv_width_;
            double max_vel_;
            double min_vel_;
            double max_angle_vel_;
            double min_angle_vel_;

            bool got_odom_ = false;
            bool got_map_ = false;
            bool got_target_ = false;
            bool executed_ = false;

            Point start_point_{0, 0};
            Point target_{0, 0};
            std::unique_ptr<GRID> grid_;

            void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
                if (got_odom_) return;

                const double x_m = msg->pose.pose.position.x;
                const double y_m = msg->pose.pose.position.y;
                start_point_ = {
                    static_cast<int>(x_m * k_units_),
                    static_cast<int>(y_m * k_units_)
                };

                got_odom_ = true;
                try_run();
            }

            void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
                if (got_map_) return;

                grid_ = std::make_unique<GRID>(static_cast<int>(msg->info.width),
                                            static_cast<int>(msg->info.height),
                                            std::vector<Point>{},
                                            std::vector<Point>{});
                for (size_t i = 0; i < msg->data.size(); i++){
                    if (msg->data[i] > 50){
                        grid_->field[i] = 1;
                    }
                    else{
                        grid_->field[i] = 0;
                    }
                }
                got_map_ = true;
                try_run();
            }

            // void targets_callback(const stingray_mapping::msg::MapObjectArray::SharedPtr msg){
            //     if (got_target_) return;

            //     const double x_m = msg->pose.position.x;
            //     const double y_m = msg->pose.position.y;
            //     target_ = {static_cast<int>(x_m * k_units_), static_cast<int>(y_m * k_units_)};
            //     got_target_ = true;
            //     try_run();
            // }

            void try_run(){
                if (executed_) return;
                if (!got_odom_ || !got_map_ || !got_target_) return;
                executed_ = true;

                AUV VELT(start_point_,
                        metres_to_grid_units(auv_length_, k_units_),
                        metres_to_grid_units(auv_width_, k_units_),
                        metres_to_grid_units(max_vel_, k_units_),
                        metres_to_grid_units(min_vel_, k_units_),
                        max_angle_vel_,
                        min_angle_vel_);
                std::vector<Point> targets = {target_};
                std::vector<Point> path = VELT.build_full_route(targets, *grid_);
                if (path.empty()) {
                    RCLCPP_ERROR(this->get_logger(), "Path not found");
                    finish();
                    return;
                }
                auto cmds = compute_commands(path, max_vel_, min_vel_, max_angle_vel_, min_angle_vel_, k_units_);

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
                    ps.pose.position.x = static_cast<double>(p.x) / k_units_;
                    ps.pose.position.y = static_cast<double>(p.y) / k_units_;
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
            //rclcpp::Subscription<stingray_mapping::msg::MapObjectArray>::SharedPtr sub_targets;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;


    };
}

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<stingray::planning::PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
