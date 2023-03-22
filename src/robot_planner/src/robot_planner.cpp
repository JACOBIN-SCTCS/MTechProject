#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_planner/costmap_client.h"
#include "tf2_ros/transform_listener.h"
#include "robot_planner/path_finder.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "builtin_interfaces/msg/duration.hpp"

namespace robot_planner
{
  class RobotPlanner : public rclcpp::Node
  {
    public:

      // /using navigate_to_pose = nav2_msgs::action::NavigateToPose;
      using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

      RobotPlanner()
      : Node("robot_planner"),
        _tf_buffer(this->get_clock()),
        tf_listener_(_tf_buffer),
        _costmap_client(*this, &_tf_buffer)

      {
        RCLCPP_INFO(this->get_logger(), "Robot Planner is running");
   
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_rep_markers", 10);
        frontier_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier_marker", 10);
        paths_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("paths", 10);
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        timer_ = this->create_wall_timer(
          std::chrono::milliseconds(500), std::bind(&RobotPlanner::timer_callback, this));

        path_utils = robot_planner::  Utils(_costmap_client.getCostmap());
      }

     
      void timer_callback()
      {
          timer_->cancel();
          geometry_msgs::msg::PoseStamped pose = _costmap_client.getRobotPose();
          path_utils.findPath(pose);
          
          std::vector<Obstacle> obstacles = path_utils.getObstacles();

          visualize_obstacle_markers(obstacles);
          RCLCPP_INFO(this->get_logger(), "Number of obstacles: %lu", obstacles.size());
          RCLCPP_INFO(this->get_logger(), "Reached inside callback");
          auto frontier = path_utils.getFrontier();              
          RCLCPP_INFO(this->get_logger(), "Frontier point index = %u" , frontier.frontier_point);
          struct WorldCoord world_coord = _costmap_client.convert_index_to_world(frontier.frontier_point);
          visualize_frontier(world_coord.x,world_coord.y);
          std::vector<std::vector<geometry_msgs::msg::Point>> paths = path_utils.getPaths();
          RCLCPP_INFO(this->get_logger(), "Number of paths: %lu", paths.size());
          RCLCPP_INFO(this->get_logger(), "Number of points in first path: %lu", paths[0].size());
          RCLCPP_INFO(this->get_logger(), "Number of points in second path: %lu", paths[1].size());
          visualize_paths(paths);
          return;


          if(!this->action_client_->wait_for_action_server())
          {
            RCLCPP_INFO(this->get_logger(), "Reached inside error section");
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
          }

          auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
          goal_msg.pose.header.frame_id = "map";
          goal_msg.pose.pose.position.x = 1.0;
          goal_msg.pose.pose.position.y = 0.0;
          goal_msg.pose.pose.orientation.w = 1.0;
         
         
          RCLCPP_INFO(this->get_logger(), "Sending goal");

          auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
          send_goal_options.goal_response_callback =
            std::bind(&RobotPlanner::goal_response_callback, this, std::placeholders::_1);
          send_goal_options.feedback_callback =
            std::bind(&RobotPlanner::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
          send_goal_options.result_callback =
            std::bind(&RobotPlanner::result_callback, this, std::placeholders::_1);
          
          RCLCPP_INFO(this->get_logger(), "Sending goal");
          action_client_->async_send_goal(goal_msg, send_goal_options);
      }

      void visualize_obstacle_markers(std::vector<Obstacle> obstacles)
      { 
         auto obstacle_marker_message = visualization_msgs::msg::MarkerArray();

          for(long unsigned int i=0;i<obstacles.size();++i)
          {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = obstacles[i].rep_world;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            builtin_interfaces::msg::Duration lifetime;
            lifetime.sec = 0;
            marker.lifetime= lifetime;
            obstacle_marker_message.markers.push_back(marker);
          }

          marker_pub_->publish(obstacle_marker_message);

      }

      void visualize_paths(std::vector<std::vector<geometry_msgs::msg::Point>> paths)
      { 
          auto paths_marker_message = visualization_msgs::msg::MarkerArray();
          srand( (unsigned)time( NULL ) );
          for(long unsigned int i=0;i<paths.size();++i)
          {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            for(long unsigned int j=0;j<paths[i].size();++j)
            {
              marker.points.push_back(paths[i][j]);
            }
            marker.color.r = (float) rand()/RAND_MAX;
            marker.color.g = (float) rand()/RAND_MAX;
            marker.color.b = (float) rand()/RAND_MAX;
            marker.color.a = 1.0;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;

            builtin_interfaces::msg::Duration lifetime;
            lifetime.sec = 0;
            marker.lifetime= lifetime;
            paths_marker_message.markers.push_back(marker);
          }

          paths_pub_->publish(paths_marker_message);
          RCLCPP_INFO(this->get_logger(), "Published paths");

      }

      void visualize_frontier(double frontier_x,double frontier_y)
      {
          auto frontier_marker = visualization_msgs::msg::Marker();
          frontier_marker.header.frame_id = "map";
          frontier_marker.header.stamp = this->now();
          frontier_marker.ns = "";
          frontier_marker.id = 0;
          frontier_marker.type = visualization_msgs::msg::Marker::SPHERE;
          frontier_marker.action = visualization_msgs::msg::Marker::ADD;
          frontier_marker.pose.position.x = frontier_x;
          frontier_marker.pose.position.y = frontier_y;
          frontier_marker.pose.orientation.z = 1.0;
          frontier_marker.color.r = 1.0f;
          frontier_marker.color.g = 0.0f;
          frontier_marker.color.b = 0.0f;
          frontier_marker.color.a = 1.0;
          frontier_marker.scale.x = 0.1;
          frontier_marker.scale.y = 0.1;
          frontier_marker.scale.z = 0.1;
          builtin_interfaces::msg::Duration lifetime;
          lifetime.sec = 0;
          frontier_marker.lifetime= lifetime;
          frontier_pub_->publish(frontier_marker);
      }
    
    protected:
      tf2_ros::Buffer _tf_buffer;
      tf2_ros::TransformListener tf_listener_;
      Costmap2DClient _costmap_client;
      robot_planner::Utils path_utils;

    
    private:
      
     
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr paths_pub_;
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontier_pub_;
      rclcpp::TimerBase::SharedPtr timer_;
     

      void goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
      {
        auto goal_handle = future.get();
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
      }

      void feedback_callback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
      {
        auto time_elapsed = feedback->navigation_time;
        std::stringstream ss;
        ss << time_elapsed.sec << " seconds elapsed";
        RCLCPP_INFO(this->get_logger(),ss.str().c_str());
        geometry_msgs::msg::PoseStamped pose = _costmap_client.getRobotPose();

      }

      void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
      {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
      }

  
  };
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_planner::RobotPlanner>());
  rclcpp::shutdown();
  return 0;
 }
