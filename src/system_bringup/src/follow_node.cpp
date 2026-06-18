#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <deque>
#include <cmath>
#include <string>
#include <algorithm>

class UGVHistoryFollowNode
{
public:
  UGVHistoryFollowNode()
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("ugv_odom_topic", ugv_odom_topic_, std::string("/ugv/state_estimation"));
    pnh.param<std::string>("uav_odom_topic", uav_odom_topic_, std::string("/uav/state_estimation"));
    pnh.param<std::string>("uav_goal_topic", uav_goal_topic_, std::string("/uav/way_point"));
    pnh.param<std::string>("uav_speed_topic", uav_speed_topic_, std::string("/uav/speed"));
    pnh.param<std::string>("history_path_topic", history_path_topic_, std::string("/airground/ugv_history_path"));
    pnh.param<std::string>("goal_frame", goal_frame_, std::string("map"));
    pnh.param<std::string>("follow_mode", follow_mode_, std::string("history"));
    pnh.param<std::string>("direct_follow_offset_mode", direct_follow_offset_mode_, std::string("heading"));

    pnh.param<double>("follow_height", follow_height_, 2.0);
    pnh.param<double>("follow_x_lag", follow_x_lag_, 0.0);
    pnh.param<double>("follow_y_lag", follow_y_lag_, 0.0);
    pnh.param<double>("lookahead_distance", lookahead_distance_, 1.5);
    pnh.param<double>("direct_follow_offset", direct_follow_offset_, 0.0);
    pnh.param<double>("min_record_distance", min_record_distance_, 0.2);
    pnh.param<double>("publish_rate", publish_rate_, 2.0);
    pnh.param<double>("follow_speed_stop_distance", follow_speed_stop_distance_, 0.8);
    pnh.param<double>("follow_speed_slow_distance", follow_speed_slow_distance_, 3.0);
    pnh.param<double>("follow_speed_max_ratio", follow_speed_max_ratio_, 0.8);
    pnh.param<double>("follow_speed_ahead_max_ratio", follow_speed_ahead_max_ratio_, 0.25);
    pnh.param<double>("follow_speed_ahead_tolerance", follow_speed_ahead_tolerance_, 0.4);

    pnh.param<int>("max_history_size", max_history_size_, 1000);

    pnh.param<bool>("use_ugv_z_as_base", use_ugv_z_as_base_, false);
    pnh.param<bool>("publish_speed", publish_speed_, true);

    pnh.param<double>("ugv_timeout", ugv_timeout_, 1.0);
    pnh.param<double>("uav_timeout", uav_timeout_, 1.0);

    pnh.param<bool>("limit_index_jump", limit_index_jump_, true);
    pnh.param<int>("max_anchor_advance_points", max_anchor_advance_points_, 25);
    pnh.param<int>("search_back_points", search_back_points_, 5);

    pnh.param<double>("max_goal_jump", max_goal_jump_, 5.0);
    pnh.param<bool>("reset_history_on_reverse", reset_history_on_reverse_, true);
    pnh.param<double>("reverse_dot_threshold", reverse_dot_threshold_, -0.9);

    if (publish_rate_ <= 0.0)
    {
      ROS_WARN("publish_rate <= 0, reset to 2.0 Hz");
      publish_rate_ = 2.0;
    }

    if (follow_mode_ != "history" && follow_mode_ != "direct")
    {
      ROS_WARN_STREAM("Unknown follow_mode: " << follow_mode_ << ", reset to history.");
      follow_mode_ = "history";
    }

    if (direct_follow_offset_mode_ != "heading" && direct_follow_offset_mode_ != "motion")
    {
      ROS_WARN_STREAM(
        "Unknown direct_follow_offset_mode: " << direct_follow_offset_mode_
        << ", reset to heading."
      );
      direct_follow_offset_mode_ = "heading";
    }

    ugv_odom_sub_ = nh.subscribe(ugv_odom_topic_, 50, &UGVHistoryFollowNode::ugvOdomCallback, this);
    uav_odom_sub_ = nh.subscribe(uav_odom_topic_, 50, &UGVHistoryFollowNode::uavOdomCallback, this);

    uav_goal_pub_ = nh.advertise<geometry_msgs::PointStamped>(uav_goal_topic_, 5);
    uav_speed_pub_ = nh.advertise<std_msgs::Float32>(uav_speed_topic_, 5);
    history_path_pub_ = nh.advertise<nav_msgs::Path>(history_path_topic_, 1, true);

    timer_ = nh.createTimer(
      ros::Duration(1.0 / publish_rate_),
      &UGVHistoryFollowNode::timerCallback,
      this
    );

    ROS_INFO_STREAM("ugv_history_follow_node started.");
    ROS_INFO_STREAM("Subscribe UGV odom: " << ugv_odom_topic_);
    ROS_INFO_STREAM("Subscribe UAV odom: " << uav_odom_topic_);
    ROS_INFO_STREAM("Publish UAV goal: " << uav_goal_topic_);
    ROS_INFO_STREAM("Publish UAV speed: " << uav_speed_topic_);
    ROS_INFO_STREAM("Publish UGV history path: " << history_path_topic_);
    ROS_INFO_STREAM("follow_mode: " << follow_mode_);
    ROS_INFO_STREAM("follow_height: " << follow_height_);
    ROS_INFO_STREAM("follow_x_lag: " << follow_x_lag_);
    ROS_INFO_STREAM("follow_y_lag: " << follow_y_lag_);
    ROS_INFO_STREAM("lookahead_distance: " << lookahead_distance_);
    ROS_INFO_STREAM("direct_follow_offset: " << direct_follow_offset_);
    ROS_INFO_STREAM("direct_follow_offset_mode: " << direct_follow_offset_mode_);
    ROS_INFO_STREAM("publish_speed: " << publish_speed_);
    ROS_INFO_STREAM("follow_speed_stop_distance: " << follow_speed_stop_distance_);
    ROS_INFO_STREAM("follow_speed_slow_distance: " << follow_speed_slow_distance_);
    ROS_INFO_STREAM("follow_speed_max_ratio: " << follow_speed_max_ratio_);
    ROS_INFO_STREAM("min_record_distance: " << min_record_distance_);
  }

private:
  ros::Subscriber ugv_odom_sub_;
  ros::Subscriber uav_odom_sub_;
  ros::Publisher uav_goal_pub_;
  ros::Publisher uav_speed_pub_;
  ros::Publisher history_path_pub_;
  ros::Timer timer_;

  std::string ugv_odom_topic_;
  std::string uav_odom_topic_;
  std::string uav_goal_topic_;
  std::string uav_speed_topic_;
  std::string history_path_topic_;
  std::string goal_frame_;
  std::string follow_mode_;
  std::string direct_follow_offset_mode_;

  double follow_height_;
  double follow_x_lag_;
  double follow_y_lag_;
  double lookahead_distance_;
  double direct_follow_offset_;
  double min_record_distance_;
  double publish_rate_;
  double follow_speed_stop_distance_;
  double follow_speed_slow_distance_;
  double follow_speed_max_ratio_;
  double follow_speed_ahead_max_ratio_;
  double follow_speed_ahead_tolerance_;
  double ugv_timeout_;
  double uav_timeout_;
  double max_goal_jump_;
  double reverse_dot_threshold_;

  int max_history_size_;
  int max_anchor_advance_points_;
  int search_back_points_;

  bool use_ugv_z_as_base_;
  bool publish_speed_;
  bool limit_index_jump_;
  bool reset_history_on_reverse_;

  bool has_ugv_odom_ = false;
  bool has_uav_odom_ = false;
  bool has_last_goal_ = false;
  bool has_anchor_index_ = false;

  nav_msgs::Odometry latest_ugv_odom_;
  nav_msgs::Odometry latest_uav_odom_;

  std::deque<geometry_msgs::Point> ugv_history_;

  int last_anchor_index_ = 0;
  geometry_msgs::PointStamped last_goal_;

private:
  double dist2D(const geometry_msgs::Point& a, const geometry_msgs::Point& b) const
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  bool isTimeout(const ros::Time& stamp, double timeout) const
  {
    if (timeout <= 0.0)
    {
      return false;
    }

    if (stamp.isZero())
    {
      return false;
    }

    const double dt = (ros::Time::now() - stamp).toSec();
    return dt > timeout;
  }

  double yawFromQuaternion(const geometry_msgs::Quaternion& q) const
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

    return std::atan2(siny_cosp, cosy_cosp);
  }

  bool isReverseSegment(const geometry_msgs::Point& p) const
  {
    if (!reset_history_on_reverse_ || ugv_history_.size() < 2)
    {
      return false;
    }

    const geometry_msgs::Point& p0 = ugv_history_[ugv_history_.size() - 2];
    const geometry_msgs::Point& p1 = ugv_history_.back();

    const double prev_dx = p1.x - p0.x;
    const double prev_dy = p1.y - p0.y;
    const double next_dx = p.x - p1.x;
    const double next_dy = p.y - p1.y;

    const double prev_len = std::sqrt(prev_dx * prev_dx + prev_dy * prev_dy);
    const double next_len = std::sqrt(next_dx * next_dx + next_dy * next_dy);

    if (prev_len < 1e-6 || next_len < 1e-6)
    {
      return false;
    }

    const double dot = (prev_dx * next_dx + prev_dy * next_dy) / (prev_len * next_len);
    return dot < reverse_dot_threshold_;
  }

  void resetHistoryAtReverse(const geometry_msgs::Point& p)
  {
    const geometry_msgs::Point last_point = ugv_history_.back();

    ugv_history_.clear();
    ugv_history_.push_back(last_point);
    ugv_history_.push_back(p);

    last_anchor_index_ = 0;
    has_anchor_index_ = false;
    has_last_goal_ = false;

    ROS_WARN_THROTTLE(
      1.0,
      "UGV reverse motion detected. Reset history path to avoid overlapping forward/backward segments."
    );
  }

  void ugvOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    latest_ugv_odom_ = *msg;
    has_ugv_odom_ = true;

    const geometry_msgs::Point& p = msg->pose.pose.position;

    if (ugv_history_.empty())
    {
      ugv_history_.push_back(p);
      publishHistoryPath();
      return;
    }

    const double moved = dist2D(p, ugv_history_.back());

    if (moved >= min_record_distance_)
    {
      if (isReverseSegment(p))
      {
        resetHistoryAtReverse(p);
      }
      else
      {
        ugv_history_.push_back(p);
      }

      while (static_cast<int>(ugv_history_.size()) > max_history_size_)
      {
        ugv_history_.pop_front();

        if (has_anchor_index_)
        {
          last_anchor_index_ = std::max(0, last_anchor_index_ - 1);
        }
      }

      publishHistoryPath();
    }
  }

  void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    latest_uav_odom_ = *msg;
    has_uav_odom_ = true;
  }

  int findNearestHistoryIndex(const geometry_msgs::Point& uav_pos)
  {
    const int n = static_cast<int>(ugv_history_.size());

    if (n <= 0)
    {
      return -1;
    }

    int start_index = 0;
    int end_index = n - 1;

    if (limit_index_jump_ && has_anchor_index_)
    {
      start_index = std::max(0, last_anchor_index_ - search_back_points_);
      end_index = std::min(n - 1, last_anchor_index_ + max_anchor_advance_points_);
    }

    int nearest_index = start_index;
    double nearest_dist = dist2D(uav_pos, ugv_history_[start_index]);

    for (int i = start_index + 1; i <= end_index; ++i)
    {
      const double d = dist2D(uav_pos, ugv_history_[i]);
      if (d < nearest_dist)
      {
        nearest_dist = d;
        nearest_index = i;
      }
    }

    return nearest_index;
  }

  geometry_msgs::Point getLookaheadPoint(int start_index, double lookahead_distance)
  {
    const int n = static_cast<int>(ugv_history_.size());

    if (n <= 0)
    {
      geometry_msgs::Point p;
      return p;
    }

    if (start_index < 0)
    {
      start_index = 0;
    }

    if (start_index >= n)
    {
      start_index = n - 1;
    }

    if (lookahead_distance <= 0.0)
    {
      return ugv_history_[start_index];
    }

    double accumulated = 0.0;

    for (int i = start_index; i < n - 1; ++i)
    {
      const geometry_msgs::Point& p0 = ugv_history_[i];
      const geometry_msgs::Point& p1 = ugv_history_[i + 1];

      const double segment_length = dist2D(p0, p1);

      if (segment_length < 1e-6)
      {
        continue;
      }

      if (accumulated + segment_length >= lookahead_distance)
      {
        const double remain = lookahead_distance - accumulated;
        const double ratio = remain / segment_length;

        geometry_msgs::Point target;
        target.x = p0.x + ratio * (p1.x - p0.x);
        target.y = p0.y + ratio * (p1.y - p0.y);
        target.z = p0.z + ratio * (p1.z - p0.z);

        return target;
      }

      accumulated += segment_length;
    }

    return ugv_history_.back();
  }

  geometry_msgs::Point getDirectFollowPoint() const
  {
    geometry_msgs::Point target = latest_ugv_odom_.pose.pose.position;

    if (direct_follow_offset_ <= 0.0)
    {
      return target;
    }

    if (direct_follow_offset_mode_ == "heading")
    {
      const double yaw = yawFromQuaternion(latest_ugv_odom_.pose.pose.orientation);

      target.x -= direct_follow_offset_ * std::cos(yaw);
      target.y -= direct_follow_offset_ * std::sin(yaw);

      return target;
    }

    if (ugv_history_.size() < 2)
    {
      const double yaw = yawFromQuaternion(latest_ugv_odom_.pose.pose.orientation);

      target.x -= direct_follow_offset_ * std::cos(yaw);
      target.y -= direct_follow_offset_ * std::sin(yaw);

      return target;
    }

    const geometry_msgs::Point& p0 = ugv_history_[ugv_history_.size() - 2];
    const geometry_msgs::Point& p1 = ugv_history_.back();

    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    const double len = std::sqrt(dx * dx + dy * dy);

    if (len < 1e-6)
    {
      const double yaw = yawFromQuaternion(latest_ugv_odom_.pose.pose.orientation);

      target.x -= direct_follow_offset_ * std::cos(yaw);
      target.y -= direct_follow_offset_ * std::sin(yaw);

      return target;
    }

    target.x -= direct_follow_offset_ * dx / len;
    target.y -= direct_follow_offset_ * dy / len;

    return target;
  }

  void publishFollowSpeed(const geometry_msgs::Point& target)
  {
    if (!publish_speed_)
    {
      return;
    }

    const geometry_msgs::Point& uav_pos = latest_uav_odom_.pose.pose.position;
    const double error = dist2D(uav_pos, target);

    double speed_ratio = 0.0;
    if (follow_speed_slow_distance_ <= follow_speed_stop_distance_)
    {
      speed_ratio = error > follow_speed_stop_distance_ ? follow_speed_max_ratio_ : 0.0;
    }
    else if (error > follow_speed_stop_distance_)
    {
      speed_ratio = follow_speed_max_ratio_
        * (error - follow_speed_stop_distance_)
        / (follow_speed_slow_distance_ - follow_speed_stop_distance_);
    }

    speed_ratio = std::max(0.0, std::min(follow_speed_max_ratio_, speed_ratio));

    if (direct_follow_offset_ > 0.0 && error < follow_speed_slow_distance_)
    {
      double follow_dir_x = 0.0;
      double follow_dir_y = 0.0;
      bool has_follow_dir = false;

      if (direct_follow_offset_mode_ == "motion" && ugv_history_.size() >= 2)
      {
        const geometry_msgs::Point& p0 = ugv_history_[ugv_history_.size() - 2];
        const geometry_msgs::Point& p1 = ugv_history_.back();
        const double dx = p1.x - p0.x;
        const double dy = p1.y - p0.y;
        const double len = std::sqrt(dx * dx + dy * dy);

        if (len > 1e-6)
        {
          follow_dir_x = dx / len;
          follow_dir_y = dy / len;
          has_follow_dir = true;
        }
      }

      if (!has_follow_dir)
      {
        const double yaw = yawFromQuaternion(latest_ugv_odom_.pose.pose.orientation);
        follow_dir_x = std::cos(yaw);
        follow_dir_y = std::sin(yaw);
        has_follow_dir = true;
      }

      const geometry_msgs::Point& ugv_pos = latest_ugv_odom_.pose.pose.position;
      const double uav_along_follow_dir =
        (uav_pos.x - ugv_pos.x) * follow_dir_x
        + (uav_pos.y - ugv_pos.y) * follow_dir_y;

      const double desired_along_follow_dir = -direct_follow_offset_;
      if (has_follow_dir && uav_along_follow_dir > desired_along_follow_dir + follow_speed_ahead_tolerance_)
      {
        speed_ratio = std::min(speed_ratio, follow_speed_ahead_max_ratio_);
      }
    }

    std_msgs::Float32 speed_msg;
    speed_msg.data = static_cast<float>(std::max(0.0, std::min(1.0, speed_ratio)));
    uav_speed_pub_.publish(speed_msg);

    ROS_INFO_THROTTLE(
      2.0,
      "Publish UAV follow speed: %.2f, target_error=%.2f",
      speed_msg.data,
      error
    );
  }

  void timerCallback(const ros::TimerEvent&)
  {
    if (!has_ugv_odom_)
    {
      ROS_WARN_THROTTLE(2.0, "No UGV odometry received yet.");
      return;
    }

    if (!has_uav_odom_)
    {
      ROS_WARN_THROTTLE(2.0, "No UAV odometry received yet.");
      return;
    }

    if (isTimeout(latest_ugv_odom_.header.stamp, ugv_timeout_))
    {
      ROS_WARN_THROTTLE(1.0, "UGV odometry timeout.");
      return;
    }

    if (isTimeout(latest_uav_odom_.header.stamp, uav_timeout_))
    {
      ROS_WARN_THROTTLE(1.0, "UAV odometry timeout.");
      return;
    }

    geometry_msgs::Point target;
    int nearest_index = -1;

    if (follow_mode_ == "direct")
    {
      target = getDirectFollowPoint();
    }
    else
    {
      if (ugv_history_.size() < 2)
      {
        ROS_WARN_THROTTLE(2.0, "UGV history is too short.");
        return;
      }

      const geometry_msgs::Point& uav_pos = latest_uav_odom_.pose.pose.position;

      nearest_index = findNearestHistoryIndex(uav_pos);

      if (nearest_index < 0)
      {
        ROS_WARN_THROTTLE(1.0, "Cannot find nearest UGV history index.");
        return;
      }

      last_anchor_index_ = nearest_index;
      has_anchor_index_ = true;

      target = getLookaheadPoint(nearest_index, lookahead_distance_);
    }

    target.x -= follow_x_lag_;
    target.y -= follow_y_lag_;

    geometry_msgs::PointStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = goal_frame_;

    goal.point.x = target.x;
    goal.point.y = target.y;

    if (use_ugv_z_as_base_)
    {
      goal.point.z = target.z + follow_height_;
    }
    else
    {
      goal.point.z = follow_height_;
    }

    if (has_last_goal_ && max_goal_jump_ > 0.0)
    {
      const double jump = dist2D(goal.point, last_goal_.point);

      if (jump > max_goal_jump_)
      {
        ROS_WARN_THROTTLE(
          1.0,
          "Reject UAV goal jump %.2f m. mode=%s, nearest_index=%d, history_size=%zu",
          jump,
          follow_mode_.c_str(),
          nearest_index,
          ugv_history_.size()
        );
        return;
      }
    }

    last_goal_ = goal;
    has_last_goal_ = true;

    uav_goal_pub_.publish(goal);
    publishFollowSpeed(goal.point);

    ROS_INFO_THROTTLE(
      2.0,
      "Publish UAV %s-follow goal: x=%.2f y=%.2f z=%.2f, nearest_index=%d, history_size=%zu",
      follow_mode_.c_str(),
      goal.point.x,
      goal.point.y,
      goal.point.z,
      nearest_index,
      ugv_history_.size()
    );
  }

  void publishHistoryPath()
  {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = goal_frame_;

    for (const auto& p : ugv_history_)
    {
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = p;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    history_path_pub_.publish(path);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_node");

  UGVHistoryFollowNode node;

  ros::spin();

  return 0;
}
