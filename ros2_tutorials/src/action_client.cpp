#include <chrono>
#include <cinttypes>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

static double g_X;
static double g_Y;

class NavigateToPoseNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit NavigateToPoseNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("send_goal_client", node_options), goal_done_(false)
  {
    this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "navigate_to_pose");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&NavigateToPoseNode::send_goal, this));
  }

  bool is_goal_done() const
  {
    return this->goal_done_;
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = NavigateToPose::Goal();

    goal_msg.pose.header.stamp = this->get_clock()->now();
    goal_msg.pose.header.frame_id = "map";

    goal_msg.pose.pose.position.x = g_X;
    goal_msg.pose.pose.position.y = g_Y;
    goal_msg.pose.pose.position.z = 0.0;

    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavigateToPoseNode::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&NavigateToPoseNode::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&NavigateToPoseNode::result_callback, this, _1);
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void goal_response_callback(GoalHandleNavigateToPose::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
     RCLCPP_INFO(
      this->get_logger(),
      "\n\nCurrent Pose : x:%f, y: %f\nDistance Remaining:  %f m\nEstimated Time Remaining: %d.%d s\nNumber of Recoveries: %d\nTotal Time Taken: %d.%d s\n" ,
      feedback->current_pose.pose.position.x, 
      feedback->current_pose.pose.position.y, 
      feedback->distance_remaining,
      feedback->estimated_time_remaining.sec,
      feedback->estimated_time_remaining.nanosec,
      feedback->number_of_recoveries,
      feedback->navigation_time.sec,
      feedback->navigation_time.nanosec
      );
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
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

    RCLCPP_INFO(this->get_logger(), "Navigation Successful");
    rclcpp::shutdown();
  }
};  // class NavigateToPoseNode

int main(int argc, char ** argv)
{
  if (argc != 3)
  {
    throw std::invalid_argument("Requires two arguments, x<double>, y<double>");
  }

  g_X = std::stod(argv[1]);
  g_Y = std::stod(argv[2]);

  std::cout << g_X << " " << g_Y << std::endl;
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<NavigateToPoseNode>();

  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}