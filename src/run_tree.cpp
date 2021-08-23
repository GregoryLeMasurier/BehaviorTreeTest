#include <ros/ros.h>
#include <ros/package.h>

#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"

#include "behavior_tree_test/dummy_nodes.h"

using namespace DummyNodes;
using namespace BT;

int main(int argc, char** argv) {
  ros::init(argc, argv, "TestRunBehaviorTree");
  ros::NodeHandle n;

  BehaviorTreeFactory factory;
  RegisterNodes(factory);
  Tree bt_tree = factory.createTreeFromFile(ros::package::getPath("behavior_tree_test") + "/trees/coroutine_test.xml");

  printTreeRecursively(bt_tree.root_node);

  StdCoutLogger logger_cout(bt_tree);

  NodeStatus status = NodeStatus::RUNNING;
  while( status == NodeStatus::RUNNING)
  {
      status = bt_tree.root_node->executeTick();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return 0;
}
