#ifndef SIMPLE_BT_NODES_H
#define SIMPLE_BT_NODES_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/action_node.h>

using Timepoint = std::chrono::time_point<std::chrono::steady_clock>;
using namespace BT;

namespace DummyNodes
{

BT::NodeStatus CheckBattery();

class GripperInterface
{
  public:
    GripperInterface() : _opened(true)
    {
    }

    BT::NodeStatus open();

    BT::NodeStatus close();

  private:
    bool _opened;
};

//--------------------------------------

// Example of custom SyncActionNode (synchronous action)
// without ports.
class ApproachObject : public BT::CoroActionNode
{
  public:
    ApproachObject(const std::string& name) :
        BT::CoroActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
};

// Example of custom SyncActionNode (synchronous action)
// with an input port.
class SaySomething : public BT::CoroActionNode
{
  public:
    SaySomething(const std::string& name, const BT::NodeConfiguration& config)
      : BT::CoroActionNode(name, config)
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("message") };
    }
};

//Same as class SaySomething, but to be registered with SimpleActionNode
BT::NodeStatus SaySomethingSimple(BT::TreeNode& self);

class MyAsyncAction: public CoroActionNode
{
  public:
    MyAsyncAction(const std::string& name):
        CoroActionNode(name, {})
    {}

  private:
    // This is the ideal skeleton/template of an async action:
    //  - A request to a remote service provider.
    //  - A loop where we check if the reply has been received.
    //  - You may call setStatusRunningAndYield() to "pause".
    //  - Code to execute after the reply.
    //  - A simple way to handle halt().

    NodeStatus tick() override

    {
        std::cout << name() <<": Started. Send Request to server." << std::endl;

        auto Now = [](){ return std::chrono::high_resolution_clock::now(); };

        TimePoint initial_time = Now();
        TimePoint time_before_reply = initial_time + std::chrono::milliseconds(100);

        int count = 0;
        bool reply_received = false;

        while( !reply_received )
        {
            if( count++ == 0)
            {
                // call this only once
                std::cout << name() <<": Waiting Reply..." << std::endl;
            }
            // pretend that we received a reply
            if( Now() >= time_before_reply )
            {
                reply_received = true;
            }

            if( !reply_received )
            {
                // set status to RUNNING and "pause/sleep"
                // If halt() is called, we will not resume execution (stack destroyed)
                setStatusRunningAndYield();
            }
        }

        // This part of the code is never reached if halt() is invoked,
        // only if reply_received == true;
        std::cout << name() <<": Done. 'Waiting Reply' loop repeated "
                  << count << " times" << std::endl;
        cleanup(false);
        if(name() == "action_B")
            return NodeStatus::FAILURE;
        return NodeStatus::SUCCESS;
    }

    // you might want to cleanup differently if it was halted or successful
    void cleanup(bool halted)
    {
        if( halted )
        {
            std::cout << name() <<": cleaning up after an halt()\n" << std::endl;
        }
        else{
            std::cout << name() <<": cleaning up after SUCCESS\n" << std::endl;
        }
    }
    void halt() override
    {
        std::cout << name() <<": Halted." << std::endl;
        cleanup(true);
        // Do not forget to call this at the end.
        CoroActionNode::halt();
    }
};

// Action_A has a different constructor than the default one.
class Action_A: public SyncActionNode
{

public:
    // additional arguments passed to the constructor
    Action_A(const std::string& name, const NodeConfiguration& config,
             int arg1, double arg2, std::string arg3 ):
        SyncActionNode(name, config),
        _arg1(arg1),
        _arg2(arg2),
        _arg3(arg3) {}

    NodeStatus tick() override
    {
        std::cout << "Action_A: " << _arg1 << " / " << _arg2 << " / " << _arg3 << std::endl;
        return NodeStatus::SUCCESS;
    }
    static PortsList providedPorts() { return {}; }

private:
    int _arg1;
    double _arg2;
    std::string _arg3;
};

// Action_B implements an init(...) method that must be called once at the beginning.
class Action_B: public SyncActionNode
{

public:
    Action_B(const std::string& name, const NodeConfiguration& config):
        SyncActionNode(name, config) {}

    // we want this method to be called ONCE and BEFORE the first tick()
    void init( int arg1, double arg2, std::string arg3 )
    {
        _arg1 = (arg1);
        _arg2 = (arg2);
        _arg3 = (arg3);
    }

    NodeStatus tick() override
    {
        std::cout << "Action_B: " << _arg1 << " / " << _arg2 << " / " << _arg3 << std::endl;
        return NodeStatus::FAILURE;
    }
    static PortsList providedPorts() { return {}; }

private:
    int _arg1;
    double _arg2;
    std::string _arg3;
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    static GripperInterface grip_singleton;

    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
    factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &grip_singleton));
    factory.registerSimpleAction("CloseGripper", std::bind(&GripperInterface::close, &grip_singleton));
    factory.registerNodeType<ApproachObject>("ApproachObject");
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<MyAsyncAction>("MyAsyncAction");
}

} // end namespace

#endif   // SIMPLE_BT_NODES_H
