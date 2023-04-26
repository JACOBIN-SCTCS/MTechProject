#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "robot_topological_explore/robot.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

const std::string bt_dir = ament_index_cpp::get_package_share_directory("robot_topological_explore") + "/bt_xml";


class TopologicalExploreNode : public rclcpp::Node
{
public:
  TopologicalExploreNode() : Node("topological_explorer"),
                             _tf_buffer(this->get_clock()),
                             tf_listener_(_tf_buffer),
                             costmap_client(*this, &_tf_buffer),
                             robot(*this,&costmap_client)
  {
    costmap_client.updateObstacles();
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_rep_markers", 10);
    locations_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("locations",10);
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
        node_->robot.get_exploration_path();
        return BT::NodeStatus::SUCCESS;
      }

      TopologicalExploreNode *node_;
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

    bt_factory.registerBuilder<ObstacleFinderBTNode>("ObstacleFinderBTNode", obstacle_builder);
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

  BT::Tree tree;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener tf_listener_;
  robot_topological_explore::Costmap2DClient costmap_client;
  robot_topological_explore::Robot robot;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr locations_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TopologicalExploreNode>();
  node->setup();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}