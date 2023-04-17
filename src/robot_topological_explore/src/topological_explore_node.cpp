#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;


const std::string bt_dir = ament_index_cpp::get_package_share_directory("robot_topological_explore") + "/bt_xml";

BT::NodeStatus CheckBattery(std::string message)
{
  std::cout << "[ Battery: OK ]" <<"---" << message<<std::endl;
  return BT::NodeStatus::SUCCESS;
}

class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string& name) :
      BT::SyncActionNode(name, {})
  {}

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

class TopologicalExploreNode : public rclcpp::Node
{
  public:
    TopologicalExploreNode() : Node("topological_explorer")
    {
      
    }

    void setup()
    {
       createBT();
       timer_ = this->create_wall_timer(
          1000ms, std::bind(&TopologicalExploreNode::updateBT, this));
    } 
    

    void createBT()
    {
      BT::BehaviorTreeFactory bt_factory;
      std::cout << bt_dir<<std::endl;
      bt_factory.registerNodeType<ApproachObject>("ApproachObject");
      bt_factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery,"hello"));

      tree = bt_factory.createTreeFromFile(bt_dir+"/tree.xml");

    }

    void updateBT()
    {
      tree.tickRoot();
      // RCLCPP_INFO(this->get_logger(),"Reached here");
    }

    BT::Tree tree;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TopologicalExploreNode>();
  node -> setup();
  rclcpp::spin(node);


  rclcpp::shutdown();
  return 0;
}