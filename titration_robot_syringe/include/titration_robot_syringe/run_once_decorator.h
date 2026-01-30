#ifndef RUN_ONCE_DECORATOR_H
#define RUN_ONCE_DECORATOR_H

#include <behaviortree_cpp_v3/decorator_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

class RunOnce : public BT::DecoratorNode {
public:
    RunOnce(const std::string& name, const BT::NodeConfiguration& config)
        : BT::DecoratorNode(name, config), has_run_(false) {
            if (!getInput("then_skip", then_skip_)) throw BT::RuntimeError("Missing required input [then_skip]");
        }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<bool>("then_skip", true, "If true, subsequent ticks are skipped")};
    }

private:
    BT::NodeStatus tick() override {
        if (has_run_) {
            if (then_skip_) return BT::NodeStatus::SUCCESS;
            else return child_status_;
        }
        BT::NodeStatus child_status = child()->executeTick();
        if (child_status == BT::NodeStatus::SUCCESS || child_status == BT::NodeStatus::FAILURE) {
            has_run_ = true;
            child_status_ = child_status;
        }
        return child_status;
    }

    void halt() override {
        DecoratorNode::halt();
    }

    bool has_run_;
    bool then_skip_;
    BT::NodeStatus child_status_;
}; 


#endif // RUN_ONCE_DECORATOR_H