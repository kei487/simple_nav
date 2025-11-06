// SPDX-FileCopyrightText: 2025 Keitaro Nakamura
// SPDX-License-Identifier: MIT License


#include "simple_nav/PurePursuitController.hpp"

#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

namespace simple_nav
{

PurePursuitController::PurePursuitController(const rclcpp::NodeOptions & options)
: Node("pure_pursuit_controller", options),
  current_state_(State::IDLE)
{
  RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller initializing...");

  // Initialize in order
  declareParameters();
  loadParameters();

  // Setup TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Setup ROS interfaces
  setupPublishers();
  setupSubscribers();
  setupServiceClient();
  setupTimer();

  RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller initialized successfully!");
}

// ============ Initialization Functions ============

void PurePursuitController::declareParameters()
{
  this->declare_parameter("lookahead_distance", 0.5);
  this->declare_parameter("target_linear_velocity", 0.3);
  this->declare_parameter("control_frequency", 20.0);
  this->declare_parameter("goal_tolerance_dist", 0.1);
  this->declare_parameter("path_service_name", "/get_path");
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("robot_base_frame", "base_link");
}

void PurePursuitController::loadParameters()
{
  this->get_parameter("lookahead_distance", lookahead_distance_);
  this->get_parameter("target_linear_velocity", target_linear_velocity_);
  this->get_parameter("control_frequency", control_frequency_);
  this->get_parameter("goal_tolerance_dist", goal_tolerance_dist_);
  this->get_parameter("path_service_name", path_service_name_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
  RCLCPP_INFO(this->get_logger(), "  lookahead_distance: %.2f m", lookahead_distance_);
  RCLCPP_INFO(this->get_logger(), "  target_linear_velocity: %.2f m/s", target_linear_velocity_);
  RCLCPP_INFO(this->get_logger(), "  control_frequency: %.1f Hz", control_frequency_);
  RCLCPP_INFO(this->get_logger(), "  goal_tolerance_dist: %.2f m", goal_tolerance_dist_);
  RCLCPP_INFO(this->get_logger(), "  path_service_name: %s", path_service_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  map_frame: %s", map_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  robot_base_frame: %s", robot_base_frame_.c_str());
}

void PurePursuitController::setupPublishers()
{
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10);
  
  current_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/current_path", rclcpp::QoS(1).transient_local());
  
  lookahead_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/lookahead_point", 1);

  RCLCPP_INFO(this->get_logger(), "Publishers created");
}

void PurePursuitController::setupSubscribers()
{
  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    std::bind(&PurePursuitController::goalPoseCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Subscribers created");
}

void PurePursuitController::setupServiceClient()
{
  get_path_client_ = this->create_client<value_iteration2_astar_msgs::srv::GetPath>(
    path_service_name_);

  RCLCPP_INFO(this->get_logger(), "Service client created for: %s", path_service_name_.c_str());
}

void PurePursuitController::setupTimer()
{
  auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&PurePursuitController::controlTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Control timer created with frequency: %.1f Hz", control_frequency_);
}

// ============ Callback Functions ============

void PurePursuitController::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  RCLCPP_INFO(this->get_logger(), "Goal pose received: [%.2f, %.2f]",
    msg->pose.position.x, msg->pose.position.y);

  // Save goal
  current_goal_ = *msg;

  // Change state to PLANNING
  current_state_ = State::PLANNING;

  // Get current pose
  geometry_msgs::msg::PoseStamped current_pose;
  if (!getCurrentPose(current_pose)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get current pose from TF. Returning to IDLE.");
    current_state_ = State::IDLE;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Current pose: [%.2f, %.2f]",
    current_pose.pose.position.x, current_pose.pose.position.y);

  // Call path service
  callPathService(current_pose, current_goal_);
}

void PurePursuitController::controlTimerCallback()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // Check state
  if (current_state_ == State::IDLE || current_state_ == State::PLANNING) {
    stopRobot();
    return;
  }

  // Get current pose
  geometry_msgs::msg::PoseStamped current_pose;
  if (!getCurrentPose(current_pose)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Failed to get current pose from TF");
    stopRobot();
    return;
  }

  // Check if goal is reached
  if (checkGoalReached(current_pose)) {
    RCLCPP_INFO(this->get_logger(), "Goal Reached!");
    stopRobot();
    current_path_.poses.clear();
    current_state_ = State::IDLE;
    return;
  }

  // Pure Pursuit calculation (NAVIGATING state)
  geometry_msgs::msg::Point lookahead_point;
  if (!findLookaheadPoint(current_pose, lookahead_point)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Failed to find lookahead point");
    stopRobot();
    return;
  }

  // Transform lookahead point to robot frame
  geometry_msgs::msg::Point target_in_robot_frame;
  if (!transformPointToRobotFrame(lookahead_point, target_in_robot_frame)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Failed to transform point to robot frame");
    stopRobot();
    return;
  }

  // Compute velocity command
  auto cmd_vel = computeVelocityCommand(target_in_robot_frame);

  // Publish command
  cmd_vel_pub_->publish(cmd_vel);

  // Publish visualization
  publishVisualization(lookahead_point);
}

void PurePursuitController::pathServiceCallback(
  rclcpp::Client<value_iteration2_astar_msgs::srv::GetPath>::SharedFuture future)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  try {
    auto response = future.get();
    
    // Check if path is valid
    if (response->path.poses.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Received empty path from service. Returning to IDLE.");
      current_state_ = State::IDLE;
      return;
    }

    // Save path
    current_path_ = response->path;
    
    RCLCPP_INFO(this->get_logger(), "Path received with %zu waypoints. Starting navigation.",
      current_path_.poses.size());

    // Change state to NAVIGATING
    current_state_ = State::NAVIGATING;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s. Returning to IDLE.", e.what());
    current_state_ = State::IDLE;
  }
}

// ============ TF Functions ============

bool PurePursuitController::getCurrentPose(geometry_msgs::msg::PoseStamped & pose)
{
  try {
    // Get transform from map to base_link
    auto transform = tf_buffer_->lookupTransform(
      map_frame_, robot_base_frame_,
      tf2::TimePointZero, tf2::durationFromSec(1.0));

    // Convert transform to pose
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;

    return true;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
    return false;
  }
}

bool PurePursuitController::transformPointToRobotFrame(
  const geometry_msgs::msg::Point & point_in,
  geometry_msgs::msg::Point & point_out)
{
  try {
    // Create PointStamped in map frame
    geometry_msgs::msg::PointStamped point_stamped_in;
    point_stamped_in.header.frame_id = map_frame_;
    point_stamped_in.header.stamp = this->now();
    point_stamped_in.point = point_in;

    // Transform to robot frame
    geometry_msgs::msg::PointStamped point_stamped_out;
    point_stamped_out = tf_buffer_->transform(
      point_stamped_in, robot_base_frame_,
      tf2::durationFromSec(1.0));

    point_out = point_stamped_out.point;
    return true;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Point transformation failed: %s", ex.what());
    return false;
  }
}

// ============ Pure Pursuit Algorithm ============

bool PurePursuitController::findLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & current_pose,
  geometry_msgs::msg::Point & lookahead_point)
{
  if (current_path_.poses.empty()) {
    return false;
  }

  const double tolerance = 0.1;  // 10cm tolerance for finding lookahead point
  double min_dist_diff = std::numeric_limits<double>::max();
  size_t best_idx = 0;
  bool found = false;

  // Search for point closest to lookahead distance
  for (size_t i = current_path_.poses.size() - 1; i > 0 ; --i) {
    const auto & path_point = current_path_.poses[i].pose.position;
    
    double dx = path_point.x - current_pose.pose.position.x;
    double dy = path_point.y - current_pose.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    double dist_diff = dist - lookahead_distance_;

    if(dist_diff < 0) {
      break;
    }
    
    if (dist_diff < min_dist_diff && dist >= lookahead_distance_ - tolerance) {
      min_dist_diff = dist_diff;
      best_idx = i;
      found = true;
    }
  }

  // If no point found at lookahead distance, use the last point
  if (!found) {
    best_idx = current_path_.poses.size() - 1;
  }

  lookahead_point = current_path_.poses[best_idx].pose.position;
  return true;
}

geometry_msgs::msg::Twist PurePursuitController::computeVelocityCommand(
  const geometry_msgs::msg::Point & target_point)
{
  geometry_msgs::msg::Twist cmd_vel;

  // Get coordinates in robot frame
  double xt = target_point.x;
  double yt = target_point.y;

  // Calculate distance to target
  double L = std::sqrt(xt * xt + yt * yt);

  // Avoid division by zero
  if (L < 0.01) {  // 1cm threshold
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
  }

  // Calculate curvature: κ = 2 * yt / L^2
  double curvature = 2.0 * yt / (L * L);

  // Set velocities
  cmd_vel.linear.x = target_linear_velocity_;
  cmd_vel.angular.z = target_linear_velocity_ * curvature;

  // Limit angular velocity (safety)
  const double max_angular_vel = 2.0;  // rad/s
  if (std::abs(cmd_vel.angular.z) > max_angular_vel) {
    cmd_vel.angular.z = std::copysign(max_angular_vel, cmd_vel.angular.z);
  }

  return cmd_vel;
}

// ============ Control Flow Functions ============

void PurePursuitController::callPathService(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  // Check if service is available
  if (!get_path_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(), 
      "Path service '%s' is not available. Returning to IDLE.",
      path_service_name_.c_str());
    current_state_ = State::IDLE;
    return;
  }

  // Create request
  auto request = std::make_shared<value_iteration2_astar_msgs::srv::GetPath::Request>();
  request->start = start;
  request->goal = goal;

  RCLCPP_INFO(this->get_logger(), "Calling path planning service...");

  // Send async request
  auto future = get_path_client_->async_send_request(
    request,
    std::bind(&PurePursuitController::pathServiceCallback, this, std::placeholders::_1));
}

bool PurePursuitController::checkGoalReached(const geometry_msgs::msg::PoseStamped & current_pose)
{
  if (current_path_.poses.empty()) {
    return false;
  }

  // Get final point of path
  const auto & goal_point = current_path_.poses.back().pose.position;

  // Calculate distance to goal (X, Y only)
  double dx = goal_point.x - current_pose.pose.position.x;
  double dy = goal_point.y - current_pose.pose.position.y;
  double dist = std::sqrt(dx * dx + dy * dy);

  return dist < goal_tolerance_dist_;
}

void PurePursuitController::stopRobot()
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = 0.0;

  cmd_vel_pub_->publish(cmd_vel);
}

void PurePursuitController::publishVisualization(const geometry_msgs::msg::Point & lookahead_point)
{
  // Publish current path
  if (!current_path_.poses.empty()) {
    current_path_.header.stamp = this->now();
    current_path_pub_->publish(current_path_);
  }

  // Publish lookahead point
  geometry_msgs::msg::PointStamped point_msg;
  point_msg.header.frame_id = map_frame_;
  point_msg.header.stamp = this->now();
  point_msg.point = lookahead_point;
  lookahead_point_pub_->publish(point_msg);
}

}  // namespace simple_nav

// ============ Main Function ============

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<simple_nav::PurePursuitController>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}

