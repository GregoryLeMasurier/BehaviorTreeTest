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
        if(name() == "action_B_SEQ2")
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

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
    factory.registerNodeType<MyAsyncAction>("MyAsyncAction");
}

} // end namespace

#endif   // SIMPLE_BT_NODES_H
