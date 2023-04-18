#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "robot_topological_explore/costmap_client.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

const std::string bt_dir = ament_index_cpp::get_package_share_directory("robot_topological_explore") + "/bt_xml";


class TopologicalExploreNode : public rclcpp::Node
{
public:
  TopologicalExploreNode() : Node("topological_explorer"),
                             _tf_buffer(this->get_clock()),
                             tf_listener_(_tf_buffer),
                             _costmap_client(*this, &_tf_buffer)
  {
    _costmap_client.updateObstacles();
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_rep_markers", 10);
    visualize_obstacle_markers(_costmap_client.obstacles_);
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
        node_->_costmap_client.updateObstacles();
        node_->visualize_obstacle_markers(node_->_costmap_client.obstacles_);
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

  BT::Tree tree;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener tf_listener_;
  robot_topological_explore::Costmap2DClient _costmap_client;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
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