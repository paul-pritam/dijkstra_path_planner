#include "../include/dijkstra_planning/dijkstra.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rmw/qos_profiles.h>

namespace dijkstra{

    DijkstraPlanner::DijkstraPlanner() : Node("dijkstra_planning_node"){

        tf_buffer_= std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_= std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rclcpp::QoS map_qos(10);

        map_qos.transient_local();
        map_qos.reliable();
        map_qos.keep_last(1);

        // subscribers
        map_sub_ = this -> create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            map_qos,
            std::bind(&DijkstraPlanner::map_callback,this,std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",  
            10,
            std::bind(&DijkstraPlanner::goal_callback, this, std::placeholders::_1)
        );

        //publishers
        path_pub_ = this-> create_publisher<nav_msgs::msg::Path>("/dijkstra/path",10);
        map_pub_ = this-> create_publisher<nav_msgs::msg::OccupancyGrid>("/dijkstra/visited_map",10);



    }



    void DijkstraPlanner::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map){
        
        map_ = map;
        visited_map_.header.frame_id=map->header.frame_id;
        visited_map_.info = map->info;
        visited_map_.data = std::vector<int8_t>(visited_map_.info.height*visited_map_.info.width,-1); //init visited_map_ || -1 means unvisited
    }

    void DijkstraPlanner::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_){
        
        if (!map_){
            RCLCPP_ERROR(get_logger(), "No map received");
            return;
        }

        visited_map_.data = std::vector<int8_t>(visited_map_.info.height * visited_map_.info.width, -1); //reset visited_map_ before each goal planning run || clears results from previous goals

        geometry_msgs::msg::TransformStamped map_to_base_tf;

        try{
            map_to_base_tf = tf_buffer_->lookupTransform(map_->header.frame_id, "base_link", tf2::TimePointZero);
        }catch(const tf2::TransformException &ex){
            RCLCPP_INFO(get_logger(), "couldnt transform map to base_link");
            return;
        }

        geometry_msgs::msg::Pose map_to_base_pose;
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
        map_to_base_pose.orientation = map_to_base_tf.transform.rotation;

        auto path = plan(map_to_base_pose, pose_->pose); //start->goal

        if 
        
    }

}