#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_planner/costmap_client.h"
#include "tf2_ros/transform_listener.h"
#include "robot_planner/obstacle_reps.h"
#include "visualization_msgs/msg/marker_array.hpp"
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
        /*sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "map", 10, std::bind(&RobotPlanner::map_callback, this, std::placeholders::_1));
        */
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_rep_markers", 10);
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        timer_ = this->create_wall_timer(
          std::chrono::milliseconds(500), std::bind(&RobotPlanner::timer_callback, this));

        obstacle_utils = robot_planner::ObstacleUtils(_costmap_client.getCostmap());
      }
     
      void timer_callback()
      {
          timer_->cancel();
          _costmap_client.printRobotPose();
          obstacle_utils.searchObstacles();
          std::vector<Obstacle> obstacles = obstacle_utils.getObstacles();

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
          RCLCPP_INFO(this->get_logger(), "Number of obstacles: %lu", obstacles.size());
          RCLCPP_INFO(this->get_logger(), "Reached inside callback");
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

      void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) const
      {
      
        RCLCPP_INFO(this->get_logger(), "Received a %d X %d map @ %.3f m/pix",
          msg->info.width,
          msg->info.height,
          msg->info.resolution);
        //_costmap_client.printRobotPose();
      }
    
    protected:
      tf2_ros::Buffer _tf_buffer;
      tf2_ros::TransformListener tf_listener_;
      Costmap2DClient _costmap_client;
      robot_planner::ObstacleUtils obstacle_utils;

    
    private:
      
     
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
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
        _costmap_client.printRobotPose();

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
