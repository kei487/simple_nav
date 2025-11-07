// SPDX-FileCopyrightText: 2025 Keitaro Nakamura
// SPDX-License-Identifier: MIT License


#ifndef PURE_PURSUIT_CONTROLLER_HPP__
#define PURE_PURSUIT_CONTROLLER_HPP__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <value_iteration2_astar_msgs/srv/get_path.hpp>

#include <memory>
#include <string>
#include <mutex>

namespace simple_nav
{

/**
 * @brief Pure Pursuit controller for differential drive robots
 * 
 * This node implements the Pure Pursuit path following algorithm.
 * It receives a goal pose, requests a path from a planning service,
 * and follows the path using Pure Pursuit control.
 */
class PurePursuitController : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit PurePursuitController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~PurePursuitController() = default;

private:
  /**
   * @brief Internal state of the controller
   */
  enum class State
  {
    IDLE,        // Waiting for goal
    PLANNING,    // Requesting path from service
    NAVIGATING   // Following the path
  };

  // ============ Initialization Functions ============
  
  /**
   * @brief Declare ROS parameters
   */
  void declareParameters();

  /**
   * @brief Load parameters from ROS parameter server
   */
  void loadParameters();

  /**
   * @brief Setup publishers
   */
  void setupPublishers();

  /**
   * @brief Setup subscribers
   */
  void setupSubscribers();

  /**
   * @brief Setup service client
   */
  void setupServiceClient();

  /**
   * @brief Setup control timer
   */
  void setupTimer();

  // ============ Callback Functions ============

  /**
   * @brief Callback for goal pose topic
   * @param msg Goal pose message
   */
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Callback for scan topic
   * @param msg scan message
   */
  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

  /**
   * @brief Control loop callback (called periodically)
   */
  void controlTimerCallback();

  /**
   * @brief Callback for path planning service response
   * @param future Future object containing service response
   */
  void pathServiceCallback(
    rclcpp::Client<value_iteration2_astar_msgs::srv::GetPath>::SharedFuture future);

  // ============ TF Functions ============

  /**
   * @brief Get current robot pose from TF
   * @param pose Output pose
   * @return true if successful, false otherwise
   */
  bool getCurrentPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Transform a point from map frame to robot base frame
   * @param point_in Point in map frame
   * @param point_out Point in robot frame
   * @return true if successful, false otherwise
   */
  bool transformPointToRobotFrame(
    const geometry_msgs::msg::Point & point_in,
    geometry_msgs::msg::Point & point_out);

  // ============ Pure Pursuit Algorithm ============

  /**
   * @brief Find lookahead point on the path
   * @param current_pose Current robot pose
   * @param lookahead_point Output lookahead point
   * @return true if lookahead point found, false otherwise
   */
  bool findLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    geometry_msgs::msg::Point & lookahead_point);

  /**
   * @brief Compute velocity command using Pure Pursuit
   * @param target_point Target point in robot frame
   * @return Twist message with velocity commands
   */
  geometry_msgs::msg::Twist computeVelocityCommand(
    const geometry_msgs::msg::Point & target_point);

  // ============ Control Flow Functions ============

  /**
   * @brief Call path planning service
   * @param start Start pose
   * @param goal Goal pose
   */
  void callPathService(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief Check if goal is reached
   * @param current_pose Current robot pose
   * @return true if goal is reached, false otherwise
   */
  bool checkGoalReached(const geometry_msgs::msg::PoseStamped & current_pose);

  /**
   * @brief Stop the robot
   */
  void stopRobot();

  /**
   * @brief Publish visualization data
   * @param lookahead_point Current lookahead point
   */
  void publishVisualization(const geometry_msgs::msg::Point & lookahead_point);

  // ============ Member Variables ============

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr current_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_point_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // Service client
  rclcpp::Client<value_iteration2_astar_msgs::srv::GetPath>::SharedPtr get_path_client_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State
  State current_state_;
  nav_msgs::msg::Path current_path_;
  geometry_msgs::msg::PoseStamped current_goal_;

  // Mutex for thread safety
  std::mutex state_mutex_;

  //obstacle avoidance
  double avoidance_vel_liner;
  double avoidance_vel_angle;

  // Parameters
  double lookahead_distance_;
  double target_linear_velocity_;
  double max_angular_vel_;
  double control_frequency_;
  double goal_tolerance_dist_;
  std::string path_service_name_;
  std::string map_frame_;
  std::string robot_base_frame_;
  bool use_obstacle_avoidance_;
  double obstacle_detect_radius_;
  double obstacle_detect_angle_;
};

}  // namespace simple_nav

#endif  // PURE_PURSUIT_CONTROLLER_HPP__

