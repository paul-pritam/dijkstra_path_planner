#include "../include/dijkstra_planning/dijkstra.hpp"
#include <rmw/qos_profiles.h>
#include <queue>
#include <algorithm> 

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
        path_pub_ = this-> create_publisher<nav_msgs::msg::Path>("/dijkstra/path",map_qos);
        map_pub_ = this-> create_publisher<nav_msgs::msg::OccupancyGrid>("/dijkstra/visited_map",map_qos);
    }

    void DijkstraPlanner::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map){
        map_ = map;
        visited_map_.header.frame_id=map->header.frame_id;
        visited_map_.info = map->info;
        visited_map_.data = std::vector<int8_t>(visited_map_.info.height*visited_map_.info.width,-1); 
    }

    void DijkstraPlanner::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_){
        
        if (!map_){
            RCLCPP_ERROR(get_logger(), "No map received");
            return;
        }

       
        visited_map_.data = std::vector<int8_t>(visited_map_.info.height * visited_map_.info.width, -1); //init visited_map_ || -1 means unvisited 

        geometry_msgs::msg::TransformStamped map_to_base_tf;
        try{
            map_to_base_tf = tf_buffer_->lookupTransform(map_->header.frame_id, "base_link", tf2::TimePointZero);
        }catch(const tf2::TransformException &ex){
            RCLCPP_INFO(get_logger(), "couldnt transform map to base_link");
            return;
        }

        geometry_msgs::msg::Pose map_to_base_pose;
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
        map_to_base_pose.position.y = map_to_base_tf.transform.translation.y;
        map_to_base_pose.orientation = map_to_base_tf.transform.rotation;

        auto path = plan(map_to_base_pose, pose_->pose); //start->goal

        if (!path.poses.empty()){
            RCLCPP_INFO(this->get_logger(),"shortest path found");
            path_pub_->publish(path);
        }
        else{
            RCLCPP_WARN(this->get_logger(), "no path to goal");
        }
    }

    nav_msgs::msg::Path DijkstraPlanner::plan(const geometry_msgs::msg::Pose &start_pose ,const geometry_msgs::msg::Pose &goal_pose) {

        std::vector<std::pair<int, int>> explore_dir={{-1,0},{1,0},{0,-1},{0,1}};

        std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
        
        // Use boolean grid for O(1) 
        std::vector<bool> visited_grid(map_->info.width * map_->info.height, false);

        GraphNode start_node = worldToGrid(start_pose);
        GraphNode goal_node = worldToGrid(goal_pose);

        // Basic bounds check
        if (!poseOnMap(start_node) || !poseOnMap(goal_node)) {
            return nav_msgs::msg::Path();
        }

        // Add start node
        start_node.cost = 0;
        pending_nodes.push(start_node);
        visited_grid[poseToCell(start_node)] = true;

        GraphNode active_node; //current node
        bool found = false;

        while (!pending_nodes.empty() && rclcpp::ok())
        {
            active_node = pending_nodes.top(); //best cost in the pending nodes
            pending_nodes.pop(); //pop the best cost 

            // Check goal
            if(active_node.x == goal_node.x && active_node.y == goal_node.y){
                found = true;
                break; 
            }

            //explore the 4 neighbors from the active_node

            for (const auto & dir : explore_dir){
                GraphNode new_node = active_node + dir;
                
                // Bound Check
                if(!poseOnMap(new_node)) continue;

                unsigned int idx = poseToCell(new_node);

                // Check visited using boolean grid
                if(visited_grid[idx]) continue;

                // Obstacle Check (Allows 0, -1, and < 50)
                int8_t val = map_->data[idx];
                bool is_obstacle = (val == 100 || val >= 50);

                if (!is_obstacle) {
                    new_node.cost = active_node.cost + 1;
                    new_node.prev = std::make_shared<GraphNode>(active_node);
                    
                    pending_nodes.push(new_node);
                    visited_grid[idx] = true; // Mark visited immediately

                    visited_map_.data[idx]= 10;//blue
                }
            }
        }
        
        // Publish visited map ONCE at the end
        map_pub_->publish(visited_map_);

        nav_msgs::msg::Path path;
        path.header.frame_id = map_->header.frame_id;
        
        if (found) {
            GraphNode* current_ptr = &active_node;
            while (current_ptr != nullptr && rclcpp::ok()){
                geometry_msgs::msg::Pose last_pose = gridToWorld(*current_ptr);
                geometry_msgs::msg::PoseStamped last_pose_stamped;
                last_pose_stamped.header.frame_id = map_->header.frame_id;
                last_pose_stamped.pose = last_pose;
                path.poses.push_back(last_pose_stamped);
                
                if(current_ptr->prev) 
                    current_ptr = current_ptr->prev.get();
                else 
                    break;
            }
            std::reverse(path.poses.begin(), path.poses.end());
        }
        return path;
    }

    GraphNode DijkstraPlanner::worldToGrid (const geometry_msgs::msg::Pose &pose){
        int grid_x = static_cast<int>((pose.position.x - map_->info.origin.position.x) / map_->info.resolution);
        int grid_y = static_cast<int>((pose.position.y - map_->info.origin.position.y) / map_->info.resolution);       
        return GraphNode(grid_x,grid_y); 
    }

    bool DijkstraPlanner::poseOnMap (const GraphNode &node){
        return node.x < static_cast<int>(map_->info.width) && node.x >= 0 && node.y < static_cast<int>(map_->info.height) && node.y >= 0;
    }

    unsigned int DijkstraPlanner::poseToCell(const GraphNode &node){
        return map_ -> info.width * node.y + node.x;
    }

    geometry_msgs::msg::Pose DijkstraPlanner::gridToWorld(const GraphNode &node){
        geometry_msgs::msg::Pose pose;
        pose.position.x = node.x * map_->info.resolution + map_->info.origin.position.x;
        pose.position.y = node.y * map_->info.resolution + map_->info.origin.position.y;
        return pose;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dijkstra::DijkstraPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}