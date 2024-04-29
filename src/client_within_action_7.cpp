#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_tutorials_interfaces/action/fibonacci.hpp"

using namespace std::placeholders;
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
using ServerGoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
using ClientGoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

class ClientFromActionNode : public rclcpp::Node
{
private:
    // Action server
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
    // Server callback group
    rclcpp::CallbackGroup::SharedPtr server_callback_group_;
    // Action clients called in the callback
    rclcpp_action::Client<Fibonacci>::SharedPtr fibonacci_client_;

public:
    ClientFromActionNode()
        : Node("client_from_action")
    {
        // Create a callback group for the server
        server_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        // Create an action server
        action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "proxy",
            std::bind(&ClientFromActionNode::handle_goal, this, _1, _2),
            std::bind(&ClientFromActionNode::handle_cancel, this, _1),
            std::bind(&ClientFromActionNode::handle_accepted, this, _1),
            rcl_action_server_get_default_options(),
            server_callback_group_);
        // Create an action client
        fibonacci_client_ = rclcpp_action::create_client<Fibonacci>(
            this,
            "fibonacci");
    }
    // Action server callback which is called when a new goal is received
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Server callback: Received goal request");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    // Action server callback which is called when a goal is canceled
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<ServerGoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Server callback: Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    // Action server callback which is called when a goal is accepted
    void handle_accepted(const std::shared_ptr<ServerGoalHandleFibonacci> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Server callback: Goal has been accepted");
        auto result = std::make_shared<Fibonacci::Result>();
        // Call action clients
        auto goal = Fibonacci::Goal();
        goal.order = goal_handle->get_goal()->order;

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        // Set the feedback callback alone
        send_goal_options.feedback_callback =
            std::bind(&ClientFromActionNode::client_feedback_callback, this, _1, _2);
        auto send_goal_future = fibonacci_client_->async_send_goal(goal, send_goal_options);
        // spin until the future is ready
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_goal_future);

        auto client_goal_handle = send_goal_future.get();
        auto get_result_future = fibonacci_client_->async_get_result(client_goal_handle);
        
        // spin until the future is ready
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), get_result_future);
        auto client_result = get_result_future.get();
        goal_handle->succeed(client_result.result);
    }
    // Action client callback which is called when a feedback is received
    void client_feedback_callback(ClientGoalHandleFibonacci::SharedPtr,
                                  const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Client callback: Received feedback: %d", feedback->partial_sequence.back());
    }
}; // class ClientFromActionNode

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<ClientFromActionNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
