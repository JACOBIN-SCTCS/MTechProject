#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "robot_topological_explore/robot.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

const std::string bt_dir = ament_index_cpp::get_package_share_directory("robot_topological_explore") + "/bt_xml";


class TopologicalExploreNode : public rclcpp::Node
{
public:

  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
  
  TopologicalExploreNode() : Node("topological_explorer"),
                             _tf_buffer(this->get_clock()),
                             tf_listener_(_tf_buffer),
                             costmap_client(*this, &_tf_buffer),
                             robot(*this,&costmap_client)
  {
    // follow_waypoints
    costmap_client.updateObstacles();
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_rep_markers", 10);
    locations_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("locations",10);
    paths_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("sample_path",10);
    navigation_client_ = rclcpp_action::create_client< nav2_msgs::action::FollowWaypoints>(this,"follow_waypoints");
    visualize_obstacle_markers(costmap_client.obstacles_);
  }
  
  class ObstacleFinderBTNode : public BT::SyncActionNode
  {
    public:
      ObstacleFinderBTNode(const std::string &name, const BT::NodeConfiguration &config, TopologicalExploreNode *node) : BT::SyncActionNode(name, config),
                                                                                                                        node_(node)
      {
      }

      BT::NodeStatus tick() override
      {
        std::cout << "ObstacleFinderBTNode: " << this->name() << std::endl;
        node_->costmap_client.updateObstacles();
        node_->visualize_obstacle_markers(node_->costmap_client.obstacles_);
        node_->visualize_positions({node_->robot.global_start_point, node_->robot.global_goal_pose});
        return BT::NodeStatus::SUCCESS;
      }

      TopologicalExploreNode *node_;
  };

  class PathFinderBTNode : public BT::SyncActionNode
  {
    public:
      PathFinderBTNode(const std::string &name, const BT::NodeConfiguration &config, TopologicalExploreNode *node) : BT::SyncActionNode(name, config),
                                                                                                                    node_(node)
      {
      }

      BT::NodeStatus tick() override
      {
        std::cout << "PathFinderBTNode: " << this->name() << std::endl;
        node_->robot.get_exploration_path();
        node_->visualize_path(node_->robot.current_path);
        return BT::NodeStatus::SUCCESS;
      }

      TopologicalExploreNode *node_;
  };

  class PathFollowerBTNode : public BT::StatefulActionNode
  {
    public:
      PathFollowerBTNode(const std::string& name, const BT::NodeConfiguration& config,TopologicalExploreNode *node): BT::StatefulActionNode(name,config), node_(node)
      {
        
      }
    

      BT::NodeStatus onStart() override;
      BT::NodeStatus onRunning() override;
      void onHalted() override;

      void path_goal_response_callback(std::shared_future<GoalHandleFollowWaypoints::SharedPtr> future)
      {  
        auto goal_handle = future.get();
        if (!goal_handle) 
        {
          RCLCPP_ERROR(node_->get_logger(), "The Navigation Path was rejected by server");
          path_following_failed = true;
          return;
        }
        RCLCPP_INFO(node_->get_logger(), "The navigation path was accepted by server, waiting for result");
      }

      void path_feedback_callback(GoalHandleFollowWaypoints::SharedPtr ,const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback> feedback)
      {
        current_waypoint = feedback->current_waypoint;
        std::stringstream ss;
        ss <<  "Current Pose : " << current_waypoint;
        RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
      }

      void path_result_callback(const GoalHandleFollowWaypoints::WrappedResult &result)
      {
        switch (result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
          {
            RCLCPP_INFO(node_->get_logger(), "Waypoint following succeeded!"); 
            goal_succeeded = true;
            return;
          }
          case rclcpp_action::ResultCode::ABORTED:
          {
            RCLCPP_ERROR(node_->get_logger(), "Path following aborted");
            path_following_failed = true;
            return;
          }

          case rclcpp_action::ResultCode::CANCELED:
          {
            RCLCPP_ERROR(node_->get_logger(), "Path following canceled");
            path_following_failed = true;
            return;
          }
          default:
          {
            RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
            path_following_failed = true;
            return;
          }
        }
      }

      TopologicalExploreNode *node_;
      bool goal_succeeded = false;
      bool path_following_failed = false;
      int current_waypoint;
  };




  void setup()
  {
    createBT();
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&TopologicalExploreNode::updateBT, this));
  }

  void createBT()
  {
    BT::BehaviorTreeFactory bt_factory;
    std::cout << bt_dir << std::endl;

    auto obstacle_builder = [&](const std::string &name, const BT::NodeConfiguration &config)
    {
      return std::make_unique<ObstacleFinderBTNode>(name, config, this);
    };

    auto pathfinder_builder = [&](const std::string &name, const BT::NodeConfiguration &config)
    {
      return std::make_unique<PathFinderBTNode>(name, config, this);
    };
    
    auto pathfollower_builder = [&](const std::string &name, const BT::NodeConfiguration &config)
    {
      return std::make_unique<PathFollowerBTNode>(name, config, this);
    };

    bt_factory.registerBuilder<ObstacleFinderBTNode>("ObstacleFinderBTNode", obstacle_builder);
    bt_factory.registerBuilder<PathFinderBTNode>("PathFinderBTNode", pathfinder_builder);
    bt_factory.registerBuilder<PathFollowerBTNode>("PathFollowerBTNode",pathfollower_builder);
    tree = bt_factory.createTreeFromFile(bt_dir + "/tree.xml");
    
  }

  void updateBT()
  {
    tree.tickRoot();
    RCLCPP_INFO(this->get_logger(), "Reached here");
  }

  void visualize_obstacle_markers(std::vector<robot_topological_explore::Obstacle> obstacles)
  {
    auto obstacle_marker_message = visualization_msgs::msg::MarkerArray();

    for (long unsigned int i = 0; i < obstacles.size(); ++i)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = obstacles[i].obstacle_pose;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      builtin_interfaces::msg::Duration lifetime;
      lifetime.sec = 0;
      marker.lifetime = lifetime;
      obstacle_marker_message.markers.push_back(marker);
    }

    marker_pub_->publish(obstacle_marker_message);
  }

  void visualize_positions(std::vector<geometry_msgs::msg::Point> positions)
  {
    auto locations_marker_message = visualization_msgs::msg::MarkerArray();
    for (long unsigned int i = 0; i < positions.size(); ++i)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = positions[i].x;
      marker.pose.position.y = positions[i].y;
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      builtin_interfaces::msg::Duration lifetime;
      lifetime.sec = 0;
      marker.lifetime = lifetime;
      locations_marker_message.markers.push_back(marker);
    }
    locations_pub_->publish(locations_marker_message);
  }

  void visualize_path(std::vector<geometry_msgs::msg::Point> path)
  {
      auto path_marker_message = visualization_msgs::msg::Marker();
      path_marker_message.header.frame_id = "map";
      path_marker_message.header.stamp = this->now();
      path_marker_message.ns = "";
      path_marker_message.id = static_cast<int>(0);
      path_marker_message.type = visualization_msgs::msg::Marker::CUBE_LIST;
      path_marker_message.action = visualization_msgs::msg::Marker::ADD;
      
      for(long unsigned int i = 0; i < path.size(); ++i)
      {
     
        path_marker_message.points.push_back(path[i]);
        
      }

      path_marker_message.color.r = 0.0f;
      path_marker_message.color.g = 0.0f;
      path_marker_message.color.b = 1.0f;
      path_marker_message.color.a = 1.0;
      path_marker_message.scale.x = 0.1;
      path_marker_message.scale.y = 0.1;
      path_marker_message.scale.z = 0.1;
      builtin_interfaces::msg::Duration lifetime;
      lifetime.sec = 0;
      path_marker_message.lifetime = lifetime;
      paths_pub_->publish(path_marker_message);
  }

  BT::Tree tree;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener tf_listener_;
  robot_topological_explore::Costmap2DClient costmap_client;
  robot_topological_explore::Robot robot;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr locations_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr paths_pub_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr navigation_client_;

};
  BT::NodeStatus TopologicalExploreNode::PathFollowerBTNode::onStart()
  {
        if(!node_->navigation_client_->wait_for_action_server())
        {
          RCLCPP_INFO(node_->get_logger(),"Waiting for navigation action server");
        }
        auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();
        for(long unsigned int i= node_->robot.current_path_index; i < node_->robot.current_path.size(); ++i)
        { 
          geometry_msgs::msg::PoseStamped pose;
          pose.header.frame_id = "map";
          pose.pose.position.x = node_->robot.current_path[i].x;  //paths[0][i].x;
          pose.pose.position.y = node_->robot.current_path[i].y;
          pose.pose.orientation.w = 1.0;
          goal_msg.poses.push_back(pose);

          RCLCPP_INFO(node_->get_logger(),"( %f, %f )" , pose.pose.position.x, pose.pose.position.y);
        }
        auto goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
       
        goal_options.goal_response_callback = std::bind(&PathFollowerBTNode::path_goal_response_callback, this, std::placeholders::_1);
        goal_options.feedback_callback =
          std::bind(&PathFollowerBTNode::path_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        goal_options.result_callback =
          std::bind(&PathFollowerBTNode::path_result_callback, this, std::placeholders::_1);
     
        node_->navigation_client_->async_send_goal(goal_msg, goal_options);
        return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus  TopologicalExploreNode::PathFollowerBTNode::onRunning()
  {
    if(goal_succeeded)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else if(path_following_failed)
    {
      return BT::NodeStatus::FAILURE;
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }  
  }

  void TopologicalExploreNode::PathFollowerBTNode::onHalted()
  {
    ;
  }


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TopologicalExploreNode>();
  node->setup();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}