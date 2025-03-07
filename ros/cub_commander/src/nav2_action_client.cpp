// copy from https://zenn.dev/katsuitoh/articles/21adc95b5f4c2d
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/time.hpp>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

class WaypointFollowerClient : public rclcpp::Node
{
public:
  WaypointFollowerClient(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  ~WaypointFollowerClient();
    
private:
  void execute();
  void onGoalResponseReceived(std::shared_future<GoalHandleFollowWaypoints::SharedPtr> future);
  void onFeedbackReceived(
			  GoalHandleFollowWaypoints::SharedPtr,
			  const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
  void onResultReceived(const GoalHandleFollowWaypoints::WrappedResult & result);
    
  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_ptr_;
  rclcpp_action::ResultCode result_code_;
};

WaypointFollowerClient::WaypointFollowerClient(rclcpp::NodeOptions options)
  : Node("waypoint_follower_client", options)
{
  client_ptr_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
  std::thread{&WaypointFollowerClient::execute, this}.detach();
}
  
WaypointFollowerClient::~WaypointFollowerClient()
{
}
  
void WaypointFollowerClient::execute()
{


  auto goal_msg = FollowWaypoints::Goal();

  std::string filename = "coordinates.yaml";
  std::string file_path = ament_index_cpp::get_package_share_directory("cub_commander")+"/routes/"+filename;
  std::cout << "coordinate file path :" + file_path  << std::endl;
  YAML::Node config = YAML::LoadFile(file_path);

    // "coordinates" ノードを取得して各座標を出力
    if (config["coordinates"]) {
        for (const auto& coord : config["coordinates"]) {
            // x, y, yawの各値を取得
            double x = coord["x"].as<double>();
            double y = coord["y"].as<double>();
            double yaw = coord["yaw"].as<double>();
            
            // 標準出力に出力
            std::cout << "x: " << x << ", y: " << y << ", yaw: " << yaw << std::endl;
            geometry_msgs::msg::PoseStamped goal;
            goal.header.frame_id = "map";
            goal.header.stamp = this->now();
            goal.pose.position.x = x;
            goal.pose.position.y = y;
            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z = 0.0;
            goal.pose.orientation.w = 1.0;
            goal_msg.poses.push_back(goal);
        }
    } else {
        std::cerr << "Error: 'coordinates' not found in the YAML file." << std::endl;
        return;
    }


  // goal_msg.poses.resize(4);

  // goal_msg.poses[0].header.frame_id = "map";
  // goal_msg.poses[0].header.stamp = this->now();
  // goal_msg.poses[0].pose.position.x = 1.0;
  // goal_msg.poses[0].pose.position.y = 0.0;
  // goal_msg.poses[0].pose.orientation.x = 0.0;
  // goal_msg.poses[0].pose.orientation.y = 0.0;
  // goal_msg.poses[0].pose.orientation.z = 1.0;
  // goal_msg.poses[0].pose.orientation.w = 0.0;

  // goal_msg.poses[1].header.frame_id = "map";
  // goal_msg.poses[1].header.stamp = this->now();
  // goal_msg.poses[1].pose.position.x = 1.5;
  // goal_msg.poses[1].pose.position.y = -0.5;
  // goal_msg.poses[1].pose.orientation.x = 0.0;
  // goal_msg.poses[1].pose.orientation.y = 0.0;
  // goal_msg.poses[1].pose.orientation.z = -0.7;
  // goal_msg.poses[1].pose.orientation.w = 0.7;

  // goal_msg.poses[2].header.frame_id = "map";
  // goal_msg.poses[2].header.stamp = this->now();
  // goal_msg.poses[2].pose.position.x = 1.5;
  // goal_msg.poses[2].pose.position.y = 0.0;
  // goal_msg.poses[2].pose.orientation.x = 0.0;
  // goal_msg.poses[2].pose.orientation.y = 0.0;
  // goal_msg.poses[2].pose.orientation.z = 0.0;
  // goal_msg.poses[2].pose.orientation.w = 1.0;

  // goal_msg.poses[3].header.frame_id = "map";
  // goal_msg.poses[3].header.stamp = this->now();
  // goal_msg.poses[3].pose.position.x = 0.0;
  // goal_msg.poses[3].pose.position.y = 0.0;
  // goal_msg.poses[3].pose.orientation.x = 0.0;
  // goal_msg.poses[3].pose.orientation.y = 0.0;
  // goal_msg.poses[3].pose.orientation.z = 0.7;
  // goal_msg.poses[3].pose.orientation.w = 0.7;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
  // send_goal_options.goal_response_callback =
  //   std::bind(&WaypointFollowerClient::onGoalResponseReceived, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&WaypointFollowerClient::onFeedbackReceived, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&WaypointFollowerClient::onResultReceived, this, _1);

  result_code_ = rclcpp_action::ResultCode::UNKNOWN;

  std::this_thread::sleep_for(std::chrono::seconds(1));

  auto result = client_ptr_->async_send_goal(goal_msg, send_goal_options);

  while (rclcpp::ok() && result_code_ == rclcpp_action::ResultCode::UNKNOWN) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  rclcpp::shutdown();
}



void WaypointFollowerClient::onGoalResponseReceived(std::shared_future<GoalHandleFollowWaypoints::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), 
		"Goal accepted by server, waiting for result");
  }
}

void WaypointFollowerClient::onFeedbackReceived(
						GoalHandleFollowWaypoints::SharedPtr,
						const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "CurrentWaypoint = %d", feedback->current_waypoint);
}

void WaypointFollowerClient::onResultReceived(const GoalHandleFollowWaypoints::WrappedResult & result)
{
  result_code_ = result.code;

  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(), "Goal was succeeded");
    for (unsigned int i = 0; i < result.result->missed_waypoints.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "missed waypoints: %d", result.result->missed_waypoints[i]);
    }
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


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointFollowerClient>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}