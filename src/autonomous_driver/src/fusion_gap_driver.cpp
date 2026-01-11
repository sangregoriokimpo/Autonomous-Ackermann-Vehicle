// fusion_driver.cpp (ROS 2 Humble)
// - TF transforms LiDAR cloud to base_frame
// - Follow-the-Gap + wall stabilization
// - Runtime sign flips to fix "always turns left" issues
// - Hard-stop also clamps yaw so you don't spin into walls

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <limits>
#include <cmath>
#include <mutex>
#include <vector>
#include <algorithm>
#include <tuple>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "autonomous_driver/pid.hpp"
#include "autonomous_driver/kalman_1d.hpp"

class FusionDriver : public rclcpp::Node
{
public:
  FusionDriver()
  : Node("fusion_driver"),
    wall_pid_(0.6, 0.08, 0.25),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    RCLCPP_INFO(this->get_logger(), "FusionDriver node started");

    // ---- Params (flip these at runtime to fix turning sign bugs) ----
    base_frame_     = this->declare_parameter<std::string>("base_frame", "base_link");
    forward_sign_   = this->declare_parameter<int>("forward_sign", 1);   // +1 means +x forward in base_frame
    lateral_sign_   = this->declare_parameter<int>("lateral_sign", 1);   // +1 means +y is LEFT (ROS standard). Use -1 if +y is RIGHT
    cmd_w_sign_     = this->declare_parameter<int>("cmd_w_sign", 1);     // +1 means +angular.z turns LEFT. Use -1 if sim turns opposite.

    // Behavior params
    max_speed_      = this->declare_parameter<double>("max_speed", 6.0);
    min_speed_      = this->declare_parameter<double>("min_speed", 0.8);
    max_yaw_rate_   = this->declare_parameter<double>("max_yaw_rate", 1.5);
    heading_gain_   = this->declare_parameter<double>("heading_gain", 2.0);
    gap_threshold_  = this->declare_parameter<double>("gap_threshold", 1.0);
    fov_deg_        = this->declare_parameter<double>("ftg_fov_deg", 35.0); // critical: keep it tight
    self_radius_    = this->declare_parameter<double>("self_radius", 0.4);

    near_stop_center_ = this->declare_parameter<double>("near_stop_center", 0.7);
    near_stop_wide_   = this->declare_parameter<double>("near_stop_wide", 0.6);
    wall_stop_        = this->declare_parameter<double>("wall_stop", 0.35);

    // Speed slew
    accel_up_      = this->declare_parameter<double>("accel_up", 3.0);   // m/s^2
    accel_down_    = this->declare_parameter<double>("accel_down", 6.0); // m/s^2

    cmd_pub_    = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("steering_arc", 10);

    auto sensor_qos = rclcpp::SensorDataQoS();

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "top_3d_lidar_plugin/out",
      sensor_qos,
      std::bind(&FusionDriver::lidarCallback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "front_rgbd/depth/image_raw",
      sensor_qos,
      std::bind(&FusionDriver::depthCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Params: base_frame=%s forward_sign=%d lateral_sign=%d cmd_w_sign=%d",
      base_frame_.c_str(), forward_sign_, lateral_sign_, cmd_w_sign_);
  }

private:
  // ---------------- Helpers ----------------
  static inline double clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
  }

  static inline double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }

  static inline double approach(double current, double target, double max_delta) {
    if (target > current) return std::min(target, current + max_delta);
    return std::max(target, current - max_delta);
  }

  static inline bool finite(double x) { return std::isfinite(x); }

  // ---------------- Follow-The-Gap bins ----------------
  struct GapBins {
    int start = 0;
    int end   = 0;
    int size() const { return end - start + 1; }
    int center() const { return (start + end) / 2; }
  };

  static double binToAngle(int bin, double angle_res) {
    // bins cover [-pi/2, +pi/2]
    return (static_cast<double>(bin) + 0.5) * angle_res - M_PI / 2.0;
  }

  static std::vector<GapBins> findGapsInBins(const std::vector<double>& ranges,
                                             int i_min, int i_max,
                                             double gap_threshold)
  {
    std::vector<GapBins> gaps;
    bool in_gap = false;
    int start = i_min;

    for (int i = i_min; i <= i_max; ++i) {
      bool free_bin = finite(ranges[i]) && (ranges[i] > gap_threshold);
      if (free_bin && !in_gap) {
        in_gap = true;
        start = i;
      } else if (!free_bin && in_gap) {
        in_gap = false;
        gaps.push_back(GapBins{start, i - 1});
      }
    }
    if (in_gap) gaps.push_back(GapBins{start, i_max});
    return gaps;
  }

  static double calculateFinalHeadingAngle(double theta_goal,
                                           double theta_c,
                                           double d_min,
                                           double alpha)
  {
    if (!finite(d_min) || d_min <= 1e-3) d_min = 1e-3;
    double w = alpha / d_min;
    return (w * theta_c + theta_goal) / (w + 1.0);
  }

  static std::tuple<bool, double, int, int, double, double>
  followTheGapOnce(const std::vector<double>& min_ranges,
                   double angle_res,
                   double gap_threshold,
                   double max_range_m,
                   double fov_angle_max_rad,
                   double theta_goal,
                   double alpha)
  {
    const int num_bins = static_cast<int>(min_ranges.size());
    if (num_bins <= 0) {
      return {false, 0.0, -1, -1, 0.0, std::numeric_limits<double>::infinity()};
    }

    auto angleToBin = [&](double angle)->int {
      double t = (angle + M_PI / 2.0) / angle_res;
      return static_cast<int>(std::floor(t));
    };

    double fov = clamp(fov_angle_max_rad, 0.0, M_PI / 2.0);
    int i_min = clamp(angleToBin(-fov), 0, num_bins - 1);
    int i_max = clamp(angleToBin(+fov), 0, num_bins - 1);
    if (i_min > i_max) std::swap(i_min, i_max);

    // Effective ranges
    std::vector<double> eff = min_ranges;
    for (int i = 0; i < num_bins; ++i) {
      double a = binToAngle(i, angle_res);
      if (std::abs(a) > fov) {
        eff[i] = std::numeric_limits<double>::infinity();
        continue;
      }
      if (!finite(eff[i]) || eff[i] > max_range_m) {
        eff[i] = std::numeric_limits<double>::infinity();
      }
    }

    // Nearest obstacle distance inside FOV window
    double d_min = std::numeric_limits<double>::infinity();
    for (int i = i_min; i <= i_max; ++i) {
      if (!finite(eff[i])) continue;
      d_min = std::min(d_min, eff[i]);
    }

    // Find gaps
    std::vector<GapBins> gaps = findGapsInBins(eff, i_min, i_max, gap_threshold);
    if (gaps.empty()) {
      return {false, 0.0, -1, -1, 0.0, d_min};
    }

    // Pick "best" gap: large AND near center (prevents grabbing side wall at 55°)
    const int forward_bin = num_bins / 2;
    const double center_bias = 0.35; // higher => more center-seeking
    auto best_it = std::max_element(
      gaps.begin(), gaps.end(),
      [&](const GapBins& a, const GapBins& b){
        double sa = static_cast<double>(a.size()) - center_bias * std::abs(a.center() - forward_bin);
        double sb = static_cast<double>(b.size()) - center_bias * std::abs(b.center() - forward_bin);
        return sa < sb;
      });

    GapBins best_gap = *best_it;

    int cbin = best_gap.center();
    double theta_c = binToAngle(cbin, angle_res);

    double theta_final = calculateFinalHeadingAngle(theta_goal, theta_c, d_min, alpha);
    if (!finite(theta_final)) {
      return {false, 0.0, -1, -1, theta_c, d_min};
    }

    return {true, theta_final, best_gap.start, best_gap.end, theta_c, d_min};
  }

  std::tuple<bool, double, int, int, double, double, double, double>
  followTheGapAdaptive(const std::vector<double>& min_ranges,
                       double angle_res,
                       double gap_threshold,
                       double fov_rad)
  {
    const double theta_goal = 0.0;
    const double alpha      = 2.5;

    bool ok = false;
    double theta_final = 0.0;
    double theta_c = 0.0;
    double d_min = std::numeric_limits<double>::infinity();
    int gap_s = -1, gap_e = -1;
    double used_max_range = 0.0;
    double used_fov = fov_rad;

    // Retry max range downwards (close-in robustness)
    for (double mr = 4.0; mr >= 2.0 && !ok; mr -= 0.5) {
      used_max_range = mr;
      auto [ok1, th, s, e, tc, dm] =
        followTheGapOnce(min_ranges, angle_res, gap_threshold, mr, used_fov, theta_goal, alpha);
      if (ok1) { ok = true; theta_final = th; gap_s = s; gap_e = e; theta_c = tc; d_min = dm; }
    }

    if (!ok) {
      for (double mr = 2.0; mr >= 0.5 && !ok; mr -= 0.25) {
        used_max_range = mr;
        auto [ok1, th, s, e, tc, dm] =
          followTheGapOnce(min_ranges, angle_res, gap_threshold, mr, used_fov, theta_goal, alpha);
        if (ok1) { ok = true; theta_final = th; gap_s = s; gap_e = e; theta_c = tc; d_min = dm; }
      }
    }

    if (ok) last_heading_angle_ = theta_final;
    return {ok, theta_final, gap_s, gap_e, theta_c, d_min, used_max_range, used_fov};
  }

  // ---------------- TF ----------------
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ---------------- Params ----------------
  std::string base_frame_;
  int forward_sign_;
  int lateral_sign_;
  int cmd_w_sign_;

  double max_speed_;
  double min_speed_;
  double max_yaw_rate_;
  double heading_gain_;
  double gap_threshold_;
  double fov_deg_;
  double self_radius_;

  double near_stop_center_;
  double near_stop_wide_;
  double wall_stop_;

  double accel_up_;
  double accel_down_;

  // ---------------- State ----------------
  AdaptivePIDController wall_pid_;
  autonomous_driver::Kalman1D steering_filter_{0.02, 0.2};

  pcl::PointCloud<pcl::PointXYZ>::Ptr rgbd_cloud_;
  std::mutex rgbd_mutex_;

  double current_speed_ = 0.0;
  rclcpp::Time last_speed_time_{0, 0, RCL_ROS_TIME};

  double last_heading_angle_ = 0.0;

  // ---------------- Callbacks ----------------
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Keep collecting if you want, but DO NOT fuse unless you TF it too.
    if (msg->encoding != "32FC1" && msg->encoding != "16UC1") return;

    double fx = 554.25, fy = 554.25;
    double cx = msg->width / 2.0;
    double cy = msg->height / 2.0;

    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->is_dense = false;

    int width  = static_cast<int>(msg->width);
    int height = static_cast<int>(msg->height);

    for (int v = 0; v < height; v += 4) {
      for (int u = 0; u < width; u += 4) {
        int idx = v * width + u;
        float depth = 0.0f;

        if (msg->encoding == "32FC1") {
          const float* p = reinterpret_cast<const float*>(&msg->data[0]);
          depth = p[idx];
        } else {
          const uint16_t* p = reinterpret_cast<const uint16_t*>(&msg->data[0]);
          depth = static_cast<float>(p[idx]) / 1000.0f;
        }

        if (!std::isfinite(depth) || depth <= 0.1f || depth > 20.0f) continue;

        pcl::PointXYZ point;
        point.z = depth;
        point.x = (u - cx) * depth / fx;
        point.y = (v - cy) * depth / fy;
        cloud->points.push_back(point);
      }
    }

    cloud->width  = cloud->points.size();
    cloud->height = 1;

    std::lock_guard<std::mutex> lock(rgbd_mutex_);
    rgbd_cloud_ = cloud;
  }

  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // TF to base_frame_
    sensor_msgs::msg::PointCloud2 cloud_base;
    try {
      cloud_base = tf_buffer_.transform(*msg, base_frame_, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "TF failed (%s -> %s): %s",
                  msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_base, *cloud);

    // IMPORTANT: keep RGBD fusion OFF until you also TF it into base_frame_
    // (Uncomment only after you convert depth->cloud with correct header.frame_id and TF it)
    // {
    //   std::lock_guard<std::mutex> lock(rgbd_mutex_);
    //   if (rgbd_cloud_ && !rgbd_cloud_->points.empty()) {
    //     *cloud += *rgbd_cloud_;
    //   }
    // }

    const double self_radius_sq = self_radius_ * self_radius_;
    const double max_y = 5.0;
    const double max_range = 50.0;

    // bins: [-90, +90]
    const double angle_res = 1.0 * M_PI / 180.0;
    const int num_bins = 180;
    std::vector<double> min_ranges(num_bins, std::numeric_limits<double>::infinity());

    double left_min = max_range;
    double right_min = max_range;

    // corridor metrics
    const double front_cone = 10.0 * M_PI / 180.0;
    const double center_y_max = 0.60;
    const double xf_min_for_front = 0.8;
    double front_far_x = 0.0;
    double front_near_center = std::numeric_limits<double>::infinity();
    int front_center_hits = 0;

    for (const auto& pt : cloud->points) {
      double x = pt.x;
      double y = pt.y;

      // runtime sign normalization
      double xf = static_cast<double>(forward_sign_) * x;
      double yf = static_cast<double>(lateral_sign_) * y;

      double r2 = xf*xf + yf*yf;
      if (r2 < self_radius_sq) continue;
      if (std::abs(yf) > max_y) continue;
      if (xf <= 0.2 || xf > 25.0) continue; // forward window

      double dist = std::hypot(xf, yf);

      // walls
      if (yf > 0.0) left_min = std::min(left_min, dist);
      else          right_min = std::min(right_min, dist);

      double ang = std::atan2(yf, xf);
      if (ang < -M_PI/2 || ang > M_PI/2) continue;

      int bin = static_cast<int>((ang + M_PI/2) / angle_res);
      if (bin >= 0 && bin < num_bins) {
        min_ranges[bin] = std::min(min_ranges[bin], dist);
      }

      if (std::abs(ang) <= front_cone && std::abs(yf) <= center_y_max && xf >= xf_min_for_front) {
        front_far_x = std::max(front_far_x, xf);
        front_near_center = std::min(front_near_center, xf);
        front_center_hits++;
      }
    }

    if (left_min == max_range) left_min = 15.0;
    if (right_min == max_range) right_min = 15.0;
    if (front_center_hits == 0) {
      front_far_x = 12.0;
      front_near_center = 12.0;
    } else if (!std::isfinite(front_near_center)) {
      front_near_center = front_far_x;
    }

    // Wide forward near distance
    const int forward_bin = num_bins / 2;
    const int sector_half_width = 12;
    double front_near_wide = std::numeric_limits<double>::infinity();
    int wide_hits = 0;
    for (int i = forward_bin - sector_half_width; i <= forward_bin + sector_half_width; ++i) {
      if (i < 0 || i >= num_bins) continue;
      if (!std::isfinite(min_ranges[i])) continue;
      front_near_wide = std::min(front_near_wide, min_ranges[i]);
      wide_hits++;
    }
    if (wide_hits == 0 || !std::isfinite(front_near_wide)) front_near_wide = 50.0;

    // Wall-follow
    double wall_error = left_min - right_min;
    double closest_wall = std::min(left_min, right_min);
    double wall_yaw = wall_pid_.compute(wall_error, closest_wall, this->now());

    // FTG (tight FOV)
    double fov_rad = clamp(fov_deg_ * M_PI / 180.0, 5.0 * M_PI/180.0, 90.0 * M_PI/180.0);
    auto [ftg_ok, theta_final, gap_s, gap_e, theta_c, d_min, used_max_range, used_fov] =
      followTheGapAdaptive(min_ranges, angle_res, gap_threshold_, fov_rad);

    // fallback
    if (!ftg_ok && std::isfinite(last_heading_angle_) && std::abs(last_heading_angle_) > 1e-6) {
      theta_final = last_heading_angle_;
      ftg_ok = true;
    }

    // Convert heading -> yaw
    const double max_heading_angle = M_PI / 3.0;
    double theta_cmd = clamp(theta_final, -max_heading_angle, max_heading_angle);
    double ftg_yaw = heading_gain_ * theta_cmd;

    // Blend (FTG dominates when close)
    double steering = 0.0;
    if (!ftg_ok) {
      steering = wall_yaw;
    } else {
      double d_use = std::isfinite(d_min) ? d_min : 10.0;
      double w_ftg = clamp(1.0 - (d_use / 8.0), 0.35, 0.85);
      steering = w_ftg * ftg_yaw + (1.0 - w_ftg) * wall_yaw;
    }

    steering = clamp(steering, -max_yaw_rate_, max_yaw_rate_);

    steering_filter_.predict();
    steering_filter_.update(steering);
    double w_cmd = steering_filter_.value();

    // Apply runtime sign for cmd_vel consumer
    w_cmd *= static_cast<double>(cmd_w_sign_);

    // Hard stop: do NOT spin in place into a wall
    bool hard_stop = (front_near_center <= near_stop_center_) ||
                     (front_near_wide   <= near_stop_wide_)   ||
                     (closest_wall      <= wall_stop_);

    double v_des = 0.0;
    if (hard_stop) {
      v_des = 0.0;
      w_cmd = 0.0; // <-- key fix: no spin when stopped
    } else {
      // speed from front_far_x
      const double far_stop = 2.0;
      const double far_free = 22.0;
      double t_far = clamp((front_far_x - far_stop) / (far_free - far_stop), 0.0, 1.0);
      double v_far = lerp(min_speed_, max_speed_, t_far);

      // turn limiter (lat accel)
      const double a_lat_max = 3.0;
      const double w_eps = 0.05;
      double v_curve = max_speed_;
      double abs_w = std::abs(w_cmd);
      if (abs_w > w_eps) v_curve = std::min(max_speed_, a_lat_max / abs_w);

      v_des = std::min(v_far, v_curve);
    }

    applySpeedSlewAndPublish(v_des, w_cmd,
                             left_min, right_min, wall_error, closest_wall,
                             ftg_ok, theta_final, theta_c, d_min, gap_s, gap_e, used_max_range, used_fov,
                             front_near_center, front_far_x, front_near_wide, front_center_hits);
  }

  void applySpeedSlewAndPublish(double v_des, double w_cmd,
                                double left_min, double right_min, double wall_error,
                                double closest_wall,
                                bool ftg_ok, double theta_final, double theta_c, double d_min,
                                int gap_s, int gap_e, double used_max_range, double used_fov,
                                double front_near_center, double front_far_x, double front_near_wide, int hits)
  {
    rclcpp::Time now = this->now();
    if (last_speed_time_.nanoseconds() == 0) last_speed_time_ = now;
    double dt = (now - last_speed_time_).seconds();
    last_speed_time_ = now;
    dt = clamp(dt, 0.0, 0.1);

    double max_up = accel_up_ * dt;
    double max_dn = accel_down_ * dt;

    if (v_des > current_speed_) current_speed_ = approach(current_speed_, v_des, max_up);
    else                        current_speed_ = approach(current_speed_, v_des, max_dn);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = current_speed_;
    cmd.angular.z = w_cmd;

    auto deg = [](double r){ return r * 180.0 / M_PI; };

    RCLCPP_INFO(this->get_logger(),
      "L=%.2f R=%.2f err=%.2f closest=%.2f | front_near=%.2f wide=%.2f far=%.2f hits=%d | "
      "v_des=%.2f v=%.2f | w=%.2f | FTG[%s] theta=%.1f° tc=%.1f° dmin=%.2f gap=[%d..%d] fov=%.1f° maxR=%.2f | "
      "SIGNS fwd=%d lat=%d cmdw=%d",
      left_min, right_min, wall_error, closest_wall,
      front_near_center, front_near_wide, front_far_x, hits,
      v_des, current_speed_, w_cmd,
      (ftg_ok ? "OK" : "FAIL"),
      deg(theta_final), deg(theta_c), d_min, gap_s, gap_e, deg(used_fov), used_max_range,
      forward_sign_, lateral_sign_, cmd_w_sign_);

    cmd_pub_->publish(cmd);
    publishSteeringArc(cmd.angular.z, cmd.linear.x);
  }

  void publishSteeringArc(double yaw_rate, double speed)
  {
    visualization_msgs::msg::Marker arc;
    arc.header.frame_id = base_frame_;
    arc.header.stamp = this->now();
    arc.ns = "steering";
    arc.id = 0;
    arc.type = visualization_msgs::msg::Marker::LINE_STRIP;
    arc.action = visualization_msgs::msg::Marker::ADD;
    arc.pose.orientation.w = 1.0;
    arc.scale.x = 0.05;
    arc.color.a = 1.0;
    arc.color.r = 1.0;
    arc.color.g = 1.0;
    arc.color.b = 0.0;
    arc.lifetime = rclcpp::Duration::from_seconds(0.1);

    const double dt = 0.1;
    const double total_time = 1.5;
    int steps = static_cast<int>(total_time / dt);

    double x = 0.0, y = 0.0, theta = 0.0;
    for (int i = 0; i < steps; ++i) {
      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y;
      p.z = 0.1;
      arc.points.push_back(p);

      double v = speed;
      double w = yaw_rate;
      x += v * dt * std::cos(theta);
      y += v * dt * std::sin(theta);
      theta += w * dt;
    }

    marker_pub_->publish(arc);
  }

  // ---------------- ROS interfaces ----------------
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionDriver>());
  rclcpp::shutdown();
  return 0;
}
