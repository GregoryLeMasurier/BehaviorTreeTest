#include "behavior_tree_test/dummy_nodes.h"

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
    DummyNodes::RegisterNodes(factory);
}

namespace DummyNodes
{

BT::NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}
}
