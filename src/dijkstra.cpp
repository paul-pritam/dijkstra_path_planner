#include "../include/dijkstra_planning/dijkstra.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rmw/qos_profiles.h>
#include <queue>


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

        if (!path.poses.empty()){
            RCLCPP_INFO(this->get_logger(),"shortest path found");
            path_pub_->publish(path);
        }
        else{
            RCLCPP_WARN(this->get_logger(), "no path to goal");
        }
        
    }

    nav_msgs::msg::Path DijkstraPlanner::plan(const geometry_msgs::msg::Pose &start_pose ,const geometry_msgs::msg::Pose &goal_pose) {

        std::vector<std::pair<int, int>> explore_dir={
            {-1,0},{1,0},{0,-1},{0,1}
        };

        std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
        std::vector <GraphNode> visited_nodes;

        pending_nodes.push(worldToGrid(start_pose));

        GraphNode active_node; //current node
        while (!pending_nodes.empty() && rclcpp::ok())
        {
            active_node = pending_nodes.top(); //best cost in the pending nodes
            pending_nodes.pop(); //pop the best cost

            if(worldToGrid(goal_pose) == active_node){
                break; //check if goal reached
            }

            //explore the 4 neighbors from the active_node
            for (const auto &dir : explore_dir){
                GraphNode new_node;
                if (std::find(visited_nodes.begin(), visited_nodes.end(),new_node) == visited_nodes.end() && poseOnMap(new_node) && map_->data.at(poseToCell(new_node)) == 0){
                    new_node.cost = active_node.cost + 1;
                    new_node.prev = std::make_shared<GraphNode>(active_node);
                    pending_nodes.push(new_node);
                    visited_nodes.push_back(new_node);
                }
            }

            visited_map_.data.at(poseToCell(active_node)) = 10; //bluw
            map_pub_->publish(visited_map_);
        }
        
        nav_msgs::msg::Path path;
        path.header.frame_id = map_->header.frame_id;
        while (active_node.prev && rclcpp::ok()){
            geometry_msgs::msg::Pose  last_pose = gridToWorld(active_node);
            geometry_msgs::msg::PoseStamped last_pose_stamped;
            last_pose_stamped.header.frame_id = map_->header.frame_id;
            last_pose_stamped.pose = last_pose;
            path.poses.push_back(last_pose_stamped);
            active_node = *active_node.prev;
        }
        std::reverse(path.poses.begin(), path.poses.end());
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

            

