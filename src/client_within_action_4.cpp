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
    // Callback group for the server
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
        // Copy the goal
        auto goal = Fibonacci::Goal();
        goal.order = goal_handle->get_goal()->order;
        // Create a shared pointer to the result
        Fibonacci::Result::SharedPtr result = nullptr;
        // Create a shared pointer to the feedback
        Fibonacci::Feedback::SharedPtr feedback = nullptr;
        // Set the callback functions
        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ClientFromActionNode::client_goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&ClientFromActionNode::client_feedback_callback, this, _1, _2, std::ref(feedback));
        send_goal_options.result_callback =
            std::bind(&ClientFromActionNode::client_result_callback, this, _1, std::ref(result));
        // Call send_goal
        fibonacci_client_->async_send_goal(goal, send_goal_options);
        // Busy loop to wait for the result to be received
        while (rclcpp::ok() && result == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the result");
            if (feedback != nullptr)
            {
                goal_handle->publish_feedback(feedback);
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        goal_handle->succeed(result);
    }
    // Action client callback which is called when a goal response is received
    void client_goal_response_callback(const ClientGoalHandleFibonacci::SharedPtr &goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Client callback: Received goal response");
    }
    // Action client callback which is called when a feedback is received
    void client_feedback_callback(ClientGoalHandleFibonacci::SharedPtr,
                                  const std::shared_ptr<const Fibonacci::Feedback> feedback,
                                  Fibonacci::Feedback::SharedPtr &feedback_ptr)
    {
        RCLCPP_INFO(this->get_logger(), "Client callback: Received feedback: %d", feedback->partial_sequence.back());
        feedback_ptr = std::make_shared<Fibonacci::Feedback>();
        *feedback_ptr = *feedback;
    }
    // Action client callback which is called when a result is received
    void client_result_callback(const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult &result,
                                Fibonacci::Result::SharedPtr &result_ptr)
    {
        RCLCPP_INFO(this->get_logger(), "Client callback: Received result");
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            result_ptr = std::make_shared<Fibonacci::Result>();
            *result_ptr = *result.result;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
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
