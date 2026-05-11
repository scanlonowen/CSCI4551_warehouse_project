/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <limits>
#include <queue>
#include <thread>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

inline static double point_distance(const geometry_msgs::msg::Point& one,
                                    const geometry_msgs::msg::Point& two)
{
  const double dx = one.x - two.x;
  const double dy = one.y - two.y;
  return sqrt(dx * dx + dy * dy);
}

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  return point_distance(one, two) < 0.01;
}

static std::vector<unsigned int> nhood8_local(
    unsigned int idx, const nav2_costmap_2d::Costmap2D& costmap)
{
  std::vector<unsigned int> out;
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();

  if (idx > size_x * size_y - 1) {
    return out;
  }

  const unsigned int x = idx % size_x;
  const unsigned int y = idx / size_x;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const int nx = static_cast<int>(x) + dx;
      const int ny = static_cast<int>(y) + dy;
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x) ||
          ny >= static_cast<int>(size_y)) {
        continue;
      }
      out.push_back(static_cast<unsigned int>(ny) * size_x +
                    static_cast<unsigned int>(nx));
    }
  }
  return out;
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , active_goal_cost_(0)
  , last_markers_count_(0)
  , goal_active_(false)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);
  this->declare_parameter<bool>("use_frontier_middle", true);
  this->declare_parameter<float>("goal_reached_radius", 0.6);
  this->declare_parameter<float>("goal_switch_min_distance", 1.0);
  this->declare_parameter<float>("goal_switch_min_cost_improvement", 0.75);
  this->declare_parameter<float>("goal_hold_time", 12.0);
  this->declare_parameter<float>("progress_distance_epsilon", 0.05);
  this->declare_parameter<float>("frontier_approach_search_radius", 2.0);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("use_frontier_middle", use_frontier_middle_);
  this->get_parameter("goal_reached_radius", goal_reached_radius_);
  this->get_parameter("goal_switch_min_distance", goal_switch_min_distance_);
  this->get_parameter("goal_switch_min_cost_improvement",
                      goal_switch_min_cost_improvement_);
  this->get_parameter("goal_hold_time", goal_hold_time_);
  this->get_parameter("progress_distance_epsilon", progress_distance_epsilon_);
  this->get_parameter("frontier_approach_search_radius",
                      frontier_approach_search_radius_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  progress_timeout_ = timeout;
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Publisher for exploration status
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  last_progress_ = this->now();
  // Start exploration right away
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
  
  
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  const auto blue = std_msgs::msg::ColorRGBA().set__b(1.0).set__a(0.5);
  const auto red = std_msgs::msg::ColorRGBA().set__r(1.0).set__a(0.5);
  const auto green = std_msgs::msg::ColorRGBA().set__g(1.0).set__a(0.5);

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // m.lifetime defaults to 0, means lives forever
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_WARN(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_WARN(logger_, "frontier %zd cost: %f min_distance: %f centroid: x=%f y=%f",
            i,
            frontiers[i].cost,
            frontiers[i].min_distance,
            frontiers[i].centroid.x,
            frontiers[i].centroid.y);
  }

  if (frontiers.empty()) {
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // Find a frontier whose reachable approach point is not blacklisted. The
  // raw frontier cells are unknown space, so sending Nav2 directly to those
  // points can produce a legal global path but no local-controller progress.
  const frontier_exploration::Frontier* frontier = nullptr;
  geometry_msgs::msg::Point target_position;
  for (const auto& candidate : frontiers) {
    if (goalOnBlacklist(candidate.centroid)) {
      continue;
    }
    const auto approach = selectGoalPoint(candidate, pose.position);
    if (goalOnBlacklist(approach)) {
      continue;
    }
    frontier = &candidate;
    target_position = approach;
    break;
  }

  if (frontier == nullptr) {
    RCLCPP_WARN(logger_, "All frontiers traversed/tried out, stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }

  // Time out if the robot is not making progress toward the currently active
  // goal. This is intentionally based on the committed goal, not the newest
  // frontier centroid, so SLAM jitter does not reset progress tracking.
  auto now = this->now();

  if (last_progress_.get_clock_type() != now.get_clock_type()) {
    last_progress_ = now;
  }

  // ensure only first call of makePlan was set resuming to true
  if (resuming_) {
    resuming_ = false;
  }

  if (goal_active_) {
    const double active_distance = point_distance(pose.position, active_goal_);

    if (active_distance <= goal_reached_radius_) {
      RCLCPP_INFO(logger_,
                  "Reached exploration approach point x=%.2f y=%.2f; choosing another frontier",
                  active_goal_.x, active_goal_.y);
      goal_active_ = false;
    } else {
      if (prev_distance_ - active_distance > progress_distance_epsilon_) {
        last_progress_ = now;
        prev_distance_ = active_distance;
      }

      if ((now - last_progress_ > tf2::durationFromSec(progress_timeout_))) {
        frontier_blacklist_.push_back(active_goal_);
        RCLCPP_WARN(logger_,
                    "No progress toward committed frontier; blacklisting x=%.2f y=%.2f",
                    active_goal_.x, active_goal_.y);
        goal_active_ = false;
        move_base_client_->async_cancel_all_goals();
        return;
      }

      const bool candidate_is_current =
          point_distance(target_position, active_goal_) <
          goal_switch_min_distance_;
      const bool held_long_enough =
          now - active_goal_sent_ > tf2::durationFromSec(goal_hold_time_);
      const bool candidate_is_much_better =
          frontier->cost + goal_switch_min_cost_improvement_ <
          active_goal_cost_;

      if (candidate_is_current || !held_long_enough ||
          !candidate_is_much_better) {
        RCLCPP_DEBUG(logger_,
                     "Keeping committed frontier goal x=%.2f y=%.2f",
                     active_goal_.x, active_goal_.y);
        return;
      }

      RCLCPP_INFO(logger_,
                  "Switching to better frontier x=%.2f y=%.2f cost %.2f -> %.2f",
                  target_position.x, target_position.y, active_goal_cost_,
                  frontier->cost);
    }
  }

  sendNavigationGoal(target_position, frontier->cost, pose.position);
}

geometry_msgs::msg::Point Explore::selectGoalPoint(
    const frontier_exploration::Frontier& frontier,
    const geometry_msgs::msg::Point& robot_position)
{
  (void)robot_position;

  if (!use_frontier_middle_) {
    return frontier.centroid;
  }

  nav2_costmap_2d::Costmap2D* costmap = costmap_client_.getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap->getMutex()));
  const unsigned char* map = costmap->getCharMap();

  geometry_msgs::msg::Point best = frontier.centroid;
  unsigned int start_mx;
  unsigned int start_my;
  if (!costmap->worldToMap(frontier.centroid.x, frontier.centroid.y, start_mx,
                           start_my)) {
    RCLCPP_WARN(logger_,
                "Frontier centroid x=%.2f y=%.2f is outside the costmap; using centroid",
                frontier.centroid.x, frontier.centroid.y);
    return frontier.centroid;
  }

  const unsigned int start = costmap->getIndex(start_mx, start_my);
  const unsigned int size_x = costmap->getSizeInCellsX();
  const unsigned int size_y = costmap->getSizeInCellsY();
  const double resolution = costmap->getResolution();
  const unsigned int max_cell_distance =
      static_cast<unsigned int>(frontier_approach_search_radius_ / resolution);

  std::queue<unsigned int> bfs;
  std::vector<bool> visited(size_x * size_y, false);
  bfs.push(start);
  visited[start] = true;

  while (!bfs.empty()) {
    const unsigned int idx = bfs.front();
    bfs.pop();

    unsigned int mx;
    unsigned int my;
    costmap->indexToCells(idx, mx, my);
    const unsigned int dx = mx > start_mx ? mx - start_mx : start_mx - mx;
    const unsigned int dy = my > start_my ? my - start_my : start_my - my;
    if (dx > max_cell_distance || dy > max_cell_distance) {
      continue;
    }

    if (map[idx] == nav2_costmap_2d::FREE_SPACE) {
      costmap->mapToWorld(mx, my, best.x, best.y);
      RCLCPP_INFO(logger_,
                  "Selected frontier approach x=%.2f y=%.2f near centroid x=%.2f y=%.2f",
                  best.x, best.y, frontier.centroid.x, frontier.centroid.y);
      return best;
    }

    for (const auto nbr : nhood8_local(idx, *costmap)) {
      if (!visited[nbr]) {
        visited[nbr] = true;
        bfs.push(nbr);
      }
    }
  }

  RCLCPP_WARN(logger_,
              "No free approach cell within %.2fm of frontier x=%.2f y=%.2f; using centroid",
              frontier_approach_search_radius_, frontier.centroid.x,
              frontier.centroid.y);
  return frontier.centroid;
}

void Explore::sendNavigationGoal(
    const geometry_msgs::msg::Point& target_position, double target_cost,
    const geometry_msgs::msg::Point& robot_position)
{
  RCLCPP_INFO(logger_, "Committing to frontier goal x=%.2f y=%.2f cost=%.2f",
              target_position.x, target_position.y, target_cost);

  active_goal_ = target_position;
  active_goal_cost_ = target_cost;
  active_goal_sent_ = this->now();
  goal_active_ = true;
  prev_goal_ = target_position;
  prev_distance_ = point_distance(robot_position, target_position);
  last_progress_ = active_goal_sent_;

  // Face the frontier so the forward lidar immediately collects useful map
  // data when the robot arrives instead of stopping with the mast pointed away.
  const double yaw =
      atan2(target_position.y - robot_position.y,
            target_position.x - robot_position.x);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  goal.pose.pose.orientation.z = sin(yaw * 0.5);
  goal.pose.pose.orientation.w = cos(yaw * 0.5);
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this,
       target_position](const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, target_position);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNING_TO_ORIGIN;
  status_pub_->publish(status_msg);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          auto status_msg = explore_lite_msgs::msg::ExploreStatus();
          status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
          status_pub_->publish(status_msg);
          RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
        }
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}
bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerace = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  const bool result_matches_active_goal =
      goal_active_ && same_point(frontier_goal, active_goal_);

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger_, "Frontier goal succeeded x=%f y=%f",
                  frontier_goal.x, frontier_goal.y);
      if (result_matches_active_goal) {
        goal_active_ = false;
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(logger_, "Goal was aborted; blacklisting x=%f y=%f",
            frontier_goal.x,
            frontier_goal.y);
      frontier_blacklist_.push_back(frontier_goal);
      if (result_matches_active_goal) {
        goal_active_ = false;
      }
      RCLCPP_WARN(logger_, "Blacklisting frontier goal at x=%f y=%f",
            frontier_goal.x,
            frontier_goal.y);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      if (result_matches_active_goal) {
        goal_active_ = false;
      }
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      if (result_matches_active_goal) {
        goal_active_ = false;
      }
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
  makePlan();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");

  // Only publish paused status if manually stopped (not finished exploring)
  if (!finished_exploring) {
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
    status_pub_->publish(status_msg);
  }

  goal_active_ = false;
  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  resuming_ = true;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS;
  status_pub_->publish(status_msg);
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
