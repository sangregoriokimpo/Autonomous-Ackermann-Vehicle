#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <string>
#include <chrono>

class GapFollower2D : public rclcpp::Node
{
public:
  GapFollower2D()
  : Node("gap_follower_2d")
  {
    // Parameters
    this->declare_parameter<std::string>("scan_topic", "/gazebo_ros_ray_sensor/out");
    this->declare_parameter<std::string>("cmd_topic", "/cmd_vel");

    this->declare_parameter<double>("max_speed", 8.0);            // m/s
    this->declare_parameter<double>("min_speed", 1.5);            // m/s
    this->declare_parameter<double>("max_steering_angle", 0.6);   // rad
    this->declare_parameter<double>("bubble_radius", 0.5);        // m (safety around closest obstacle)
    this->declare_parameter<double>("gap_threshold", 1.0);        // m of free space to count as gap

    // Vehicle is facing -X in the world, while LiDAR zero is +X → offset of PI
    this->declare_parameter<double>("sensor_yaw_offset", M_PI);   // rad, LiDAR +X to vehicle forward
    this->declare_parameter<double>("forward_fov", 1.8);          // rad, total FOV around forward (≈103°)

    std::string scan_topic, cmd_topic;
    this->get_parameter("scan_topic", scan_topic);
    this->get_parameter("cmd_topic", cmd_topic);
    this->get_parameter("max_speed", max_speed_);
    this->get_parameter("min_speed", min_speed_);
    this->get_parameter("max_steering_angle", max_steer_);
    this->get_parameter("bubble_radius", bubble_radius_);
    this->get_parameter("gap_threshold", gap_threshold_);
    this->get_parameter("sensor_yaw_offset", sensor_yaw_offset_);
    this->get_parameter("forward_fov", forward_fov_);

    RCLCPP_INFO(
      get_logger(),
      "GapFollower2D: scan='%s', cmd='%s'\n"
      "  max_speed=%.2f, min_speed=%.2f, max_steer=%.2f\n"
      "  bubble_radius=%.2f, gap_threshold=%.2f\n"
      "  sensor_yaw_offset=%.2f rad, forward_fov=%.2f rad",
      scan_topic.c_str(), cmd_topic.c_str(),
      max_speed_, min_speed_, max_steer_,
      bubble_radius_, gap_threshold_,
      sensor_yaw_offset_, forward_fov_);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&GapFollower2D::scanCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    // Simple heartbeat
    alive_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        RCLCPP_INFO(
          this->get_logger(),
          "gap_follower_2d alive. cmd_vel subs=%zu",
          cmd_pub_->get_subscription_count());
      });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr alive_timer_;

  double max_speed_{8.0};
  double min_speed_{1.5};
  double max_steer_{0.6};
  double bubble_radius_{0.5};
  double gap_threshold_{1.0};
  double sensor_yaw_offset_{M_PI};
  double forward_fov_{1.8};

  static double normalizeAngle(double a)
  {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const auto & ranges_in = msg->ranges;
    const size_t N = ranges_in.size();
    if (N == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 2000,
        "Empty LaserScan received");
      publishStop();
      return;
    }

    // 1) Sanitize ranges
    std::vector<float> ranges(ranges_in.begin(), ranges_in.end());
    const float r_min = msg->range_min;
    const float r_max = msg->range_max;

    for (auto & r : ranges) {
      if (!std::isfinite(r)) {
        r = r_max;
      } else if (r < r_min) {
        r = r_min;
      } else if (r > r_max) {
        r = r_max;
      }
    }

    // Precompute which beams are within forward FOV in *vehicle* frame
    std::vector<bool> in_forward(N, false);
    size_t num_forward = 0;
    const double half_fov = forward_fov_ * 0.5;

    for (size_t i = 0; i < N; ++i) {
      double theta_sensor = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      // Rotate so that 0 rad = vehicle forward (-X in world for your setup)
      double theta_vehicle = normalizeAngle(theta_sensor - sensor_yaw_offset_);
      if (std::abs(theta_vehicle) <= half_fov) {
        in_forward[i] = true;
        ++num_forward;
      }
    }

    if (num_forward == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 2000,
        "No beams in forward FOV; stopping");
      publishStop();
      return;
    }

    // 2) Closest obstacle within forward FOV
    size_t closest_idx = 0;
    float closest_range = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < N; ++i) {
      if (!in_forward[i]) continue;
      float r = ranges[i];
      if (r < closest_range) {
        closest_range = r;
        closest_idx = i;
      }
    }

    if (!std::isfinite(closest_range)) {
      // Nothing detected in forward FOV → go straight at max speed
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x  = max_speed_;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
      return;
    }

    // 3) Safety bubble around the closest obstacle (only applied to forward beams)
    float use_range = std::max(closest_range, 0.5f);
    float bubble_angle = bubble_radius_ / use_range;  // radians
    int bubble_cells = static_cast<int>(std::round(bubble_angle / msg->angle_increment));

    for (int offset = -bubble_cells; offset <= bubble_cells; ++offset) {
      int idx = static_cast<int>(closest_idx) + offset;
      if (idx >= 0 && idx < static_cast<int>(N) && in_forward[static_cast<size_t>(idx)]) {
        ranges[static_cast<size_t>(idx)] = r_min;  // mark as "occupied"
      }
    }

    // 4) Find largest gap (consecutive beams with range > gap_threshold_ in forward FOV)
    size_t best_start = 0, best_size = 0;
    size_t cur_start = 0, cur_size = 0;

    for (size_t i = 0; i < N; ++i) {
      bool free_cell = in_forward[i] && (ranges[i] > gap_threshold_);
      if (free_cell) {
        if (cur_size == 0) cur_start = i;
        ++cur_size;
      } else {
        if (cur_size > best_size) {
          best_size = cur_size;
          best_start = cur_start;
        }
        cur_size = 0;
      }
    }
    if (cur_size > best_size) {
      best_size = cur_size;
      best_start = cur_start;
    }

    if (best_size == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 500,
        "No safe gap in forward FOV; publishing stop");
      publishStop();
      return;
    }

    // 5) Furthest beam in that largest gap
    size_t best_idx = best_start;
    float best_range = -1.0f;
    for (size_t i = best_start; i < best_start + best_size; ++i) {
      if (!in_forward[i]) continue;  // should already be true, but just in case
      if (ranges[i] > best_range) {
        best_range = ranges[i];
        best_idx   = i;
      }
    }

    // Compute steering angle in vehicle frame (0 forward)
    double theta_sensor_best =
      msg->angle_min + static_cast<double>(best_idx) * msg->angle_increment;
    double steering_angle = normalizeAngle(theta_sensor_best - sensor_yaw_offset_);

    // Clamp steering
    if (steering_angle >  max_steer_) steering_angle =  max_steer_;
    if (steering_angle < -max_steer_) steering_angle = -max_steer_;

    // 6) Speed schedule based on distance to goal point and curvature
    double dist  = static_cast<double>(best_range);
    double speed = max_speed_;

    if (dist < 0.5) {
      speed = 0.0;
    } else if (dist < 1.0) {
      speed = min_speed_;
    } else if (dist < 2.0) {
      speed = 0.5 * (min_speed_ + max_speed_);
    } else {
      speed = max_speed_;
    }

    // Slow down on tight turns
    double steer_mag = std::abs(steering_angle);
    double curvature_factor = 1.0 / (1.0 + steer_mag);
    if (curvature_factor < 0.4) curvature_factor = 0.4;
    speed *= curvature_factor;

    // Don't crawl forever; if path is reasonably clear, enforce min_speed
    if (speed < 0.3 && dist > 1.0) {
      speed = min_speed_;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = speed;
    cmd.angular.z = steering_angle;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), 500,
      "Gap: start=%zu size=%zu best_idx=%zu | ang=%.1fdeg dist=%.2fm -> v=%.2f, steer=%.2f (cmd subs=%zu)",
      best_start, best_size, best_idx,
      steering_angle * 180.0 / M_PI,
      dist, speed, cmd.angular.z,
      cmd_pub_->get_subscription_count());

    cmd_pub_->publish(cmd);
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    cmd_pub_->publish(stop);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GapFollower2D>());
  rclcpp::shutdown();
  return 0;
}
