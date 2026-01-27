#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


namespace dijkstra{

    struct  GraphNode{
        int x;
        int y;
        int cost;
        std::shared_ptr<GraphNode>prev;
        
        GraphNode() : GraphNode(0,0){}
        GraphNode(int in_x, int in_y) : x(in_x), y(in_y), cost(0) {};

        bool operator > (const GraphNode &other) const {
            return cost > other.cost;
        }
        bool operator == (const GraphNode &other) const {
            return cost == other.cost;
        }

        GraphNode operator + (std::pair<int, int> const &other){
            GraphNode res(x + other.first, y + other.second);
            return res;
        }
    };

    class DijkstraPlanner : public rclcpp::Node {

        public:
            DijkstraPlanner();
        
        private:
            void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
            void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_);
            
            nav_msgs::msg::Path plan (const geometry_msgs::msg::Pose &start_pose ,
                                      const geometry_msgs::msg::Pose &goal_pose);
            

            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_; 
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; //publishes the planned path
            rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_; //publishes 

            //for storing data
            nav_msgs::msg::OccupancyGrid::SharedPtr map_; //for read purposes only
            nav_msgs::msg::OccupancyGrid visited_map_;// for r/w purposes

            //for robot position
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;      
    };

} //namespace

#endif