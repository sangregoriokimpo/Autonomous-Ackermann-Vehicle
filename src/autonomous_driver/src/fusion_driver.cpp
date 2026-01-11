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

#include "autonomous_driver/pid.hpp"
#include "autonomous_driver/kalman_1d.hpp"

class FusionDriver : public rclcpp::Node
{
public:
    FusionDriver() : Node("fusion_driver"), wall_pid_(0.6, 0.08, 0.25)
    {
        RCLCPP_INFO(this->get_logger(), "FusionDriver node started");

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
    }

private:
    // ---------------- Helpers ----------------
    static inline double clamp(double x, double lo, double hi)
    {
        return std::max(lo, std::min(hi, x));
    }

    static inline double lerp(double a, double b, double t)
    {
        return a + (b - a) * t;
    }

    static inline double approach(double current, double target, double max_delta)
    {
        if (target > current) return std::min(target, current + max_delta);
        return std::max(target, current - max_delta);
    }

    // ---------------- State ----------------
    AdaptivePIDController wall_pid_;
    autonomous_driver::Kalman1D steering_filter_{0.02, 0.2};  // Q, R

    pcl::PointCloud<pcl::PointXYZ>::Ptr rgbd_cloud_;
    std::mutex rgbd_mutex_;

    // Speed slew limiting
    double current_speed_ = 0.0;
    rclcpp::Time last_speed_time_{0, 0, RCL_ROS_TIME};

    // ---------------- Callbacks ----------------
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (msg->encoding != "32FC1" && msg->encoding != "16UC1") {
            return;
        }

        double fx = 554.25;
        double fy = 554.25;
        double cx = msg->width / 2.0;
        double cy = msg->height / 2.0;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->is_dense = false;

        int width = msg->width;
        int height = msg->height;

        for (int v = 0; v < height; v += 4) {
            for (int u = 0; u < width; u += 4) {
                float depth = 0.0f;
                int idx = v * width + u;

                if (msg->encoding == "32FC1") {
                    const float* depth_ptr = reinterpret_cast<const float*>(&msg->data[0]);
                    depth = depth_ptr[idx];
                } else { // "16UC1"
                    const uint16_t* depth_ptr = reinterpret_cast<const uint16_t*>(&msg->data[0]);
                    depth = depth_ptr[idx] / 1000.0f;
                }

                if (!std::isfinite(depth) || depth <= 0.1f || depth > 20.0f) {
                    continue;
                }

                pcl::PointXYZ point;
                point.z = depth;
                point.x = (u - cx) * depth / fx;
                point.y = (v - cy) * depth / fy;

                cloud->points.push_back(point);
            }
        }

        cloud->width = cloud->points.size();
        cloud->height = 1;

        {
            std::lock_guard<std::mutex> lock(rgbd_mutex_);
            rgbd_cloud_ = cloud;
        }
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Fuse RGBD cloud if present
        pcl::PointCloud<pcl::PointXYZ>::Ptr rgbd_cloud;
        {
            std::lock_guard<std::mutex> lock(rgbd_mutex_);
            if (rgbd_cloud_) {
                rgbd_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZ>(*rgbd_cloud_));
            }
        }

        if (rgbd_cloud && !rgbd_cloud->points.empty()) {
            *cloud += *rgbd_cloud;
        }

        // ---------------- Filtering + Polar bins ----------------
        const double self_radius_sq = 0.4 * 0.4;
        const double min_x = -25.0;  // remember: forward is -x
        const double max_x = -1.0;
        const double max_y = 5.0;
        const double max_range = 50.0;

        double left_min = max_range;
        double right_min = max_range;

        const double gap_threshold = 1.0;
        const double angle_res = 1.0 * M_PI / 180.0;
        const int num_bins = 180;
        std::vector<double> min_ranges(num_bins, std::numeric_limits<double>::infinity());

        // -------- NEW: front corridor metrics computed from raw points (NOT min_ranges) --------
        // Tight “centerline corridor” in front to ignore side walls
        const double front_cone = 10.0 * M_PI / 180.0; // +/- 10 degrees
        const double center_y_max = 0.60;              // meters: reject walls
        const double xf_min_for_front = 0.8;           // ignore very close clutter
        double front_far_x = 0.0;                      // farthest forward point (xf)
        double front_near_center = std::numeric_limits<double>::infinity();
        int front_center_hits = 0;

        for (const auto& pt : cloud->points)
        {
            double x = pt.x;
            double y = pt.y;
            double r2 = x * x + y * y;

            if (r2 < self_radius_sq) continue;
            if (x > max_x || x < min_x) continue;
            if (std::abs(y) > max_y) continue;

            // forward is -x, so flip to forward-positive frame:
            double xf = -x;
            double yf = y;
            if (xf <= 0.0) continue;

            double dist = std::hypot(xf, yf);

            // wall mins (for steering + safety only)
            if (yf > 0.0) {
                if (dist < left_min) left_min = dist;
            } else {
                if (dist < right_min) right_min = dist;
            }

            double angle = std::atan2(yf, xf);
            if (angle < -M_PI / 2 || angle > M_PI / 2) continue;

            // min_ranges: nearest obstacle per angle bin (good for steering / collision)
            int bin = static_cast<int>((angle + M_PI / 2) / angle_res);
            if (bin >= 0 && bin < num_bins) {
                if (dist < min_ranges[bin])
                    min_ranges[bin] = dist;
            }

            // -------- NEW: farthest forward point in a narrow front corridor --------
            if (std::abs(angle) <= front_cone && std::abs(yf) <= center_y_max && xf >= xf_min_for_front) {
                front_far_x = std::max(front_far_x, xf);
                front_near_center = std::min(front_near_center, xf);
                front_center_hits++;
            }
        }

        if (left_min == max_range) left_min = 15.0;
        if (right_min == max_range) right_min = 15.0;

        // Fallback if corridor gate found nothing
        if (front_center_hits == 0) {
            front_far_x = 12.0;                 // assume open-ish
            front_near_center = 12.0;
        } else if (!std::isfinite(front_near_center)) {
            front_near_center = front_far_x;
        }

        // ---------------- Wall-follow steering ----------------
        double wall_error = left_min - right_min;
        double closest_wall = std::min(left_min, right_min);
        double wall_steering = wall_pid_.compute(wall_error, closest_wall, this->now());

        // ---------------- Gap scoring steering ----------------
        int forward_bin = num_bins / 2;
        double best_score = -1.0;
        int best_bin = forward_bin;
        const double bias = 0.12;

        for (int i = 0; i < num_bins; ++i)
        {
            double range = min_ranges[i];
            if (!std::isfinite(range) || range < gap_threshold)
                continue;

            double score = range - bias * std::abs(i - forward_bin);
            if (score > best_score)
            {
                best_score = score;
                best_bin = i;
            }
        }

        double best_angle = (best_bin + 0.5) * angle_res - M_PI / 2;

        double steering = 0.0;
        double gap_steering = 0.0;

        if (best_score < 0) {
            steering = wall_steering;
        } else {
            const double max_steering_angle = M_PI / 3.0;
            best_angle = clamp(best_angle, -max_steering_angle, max_steering_angle);

            gap_steering = 5.5 * best_angle;
            steering = 0.5 * gap_steering + 0.5 * wall_steering;
        }

        // Clamp angular velocity
        const double max_angular_vel = 2.0;
        steering = clamp(steering, -max_angular_vel, max_angular_vel);

        // Filter steering (use what you actually command)
        steering_filter_.predict();
        steering_filter_.update(steering);
        double filtered_steering = steering_filter_.value();

        // Your cmd uses negative sign
        double w_cmd = -filtered_steering;

        // ---------------- Safety near in a wider forward sector (but not speed driver) ----------------
        // This is for “don’t hit stuff immediately in front”
        const int sector_half_width = 12; // +/- ~12 degrees
        double front_near_wide = std::numeric_limits<double>::infinity();
        int wide_hits = 0;
        for (int i = forward_bin - sector_half_width; i <= forward_bin + sector_half_width; ++i) {
            if (i < 0 || i >= num_bins) continue;
            double r = min_ranges[i];
            if (!std::isfinite(r)) continue;
            front_near_wide = std::min(front_near_wide, r);
            wide_hits++;
        }
        if (wide_hits == 0 || !std::isfinite(front_near_wide)) front_near_wide = 50.0;

        // ---------------- FAST Speed control (front_far_x-driven) ----------------
        // Primary speed driver: farthest point ahead in the center corridor
        const double v_max = 240.0;  // faster
        const double v_min = 1.5;   // keep rolling instead of hovering near 0

        // Map front_far_x -> speed
        const double far_stop = 2.0;   // if farthest ahead is only ~2m, go slow
        const double far_free = 22.0;  // if farthest ahead >= 22m, full speed

        // Hard safety stops
        const double near_stop_center = 0.7; // if something in center corridor is too close
        const double near_stop_wide   = 0.6; // if anything in forward cone is too close
        const double wall_stop        = 0.35;

        // Turning-based speed limit (curvature / lateral accel cap)
        const double a_lat_max = 11.0; // higher => faster in turns
        const double w_eps = 0.05;

        // Hard safety stop
        if (front_near_center <= near_stop_center || front_near_wide <= near_stop_wide || closest_wall <= wall_stop) {
            applySpeedSlewAndPublish(0.0, w_cmd,
                                     left_min, right_min, wall_error, closest_wall,
                                     best_angle, gap_steering, wall_steering,
                                     steering, filtered_steering,
                                     front_near_center, front_far_x, front_near_wide, front_center_hits);
            return;
        }

        // front_far_x-based desired speed (smooth)
        double t_far = clamp((front_far_x - far_stop) / (far_free - far_stop), 0.0, 1.0);
        double v_far_limit = lerp(v_min, v_max, t_far);

        // turning limiter
        double abs_w = std::abs(w_cmd);
        double v_curve_limit = v_max;
        if (abs_w > w_eps) {
            v_curve_limit = std::min(v_max, a_lat_max / abs_w);
        }

        // final desired speed: driven by farthest-ahead, limited by turn physics
        double v_des = std::min(v_far_limit, v_curve_limit);

        applySpeedSlewAndPublish(v_des, w_cmd,
                                 left_min, right_min, wall_error, closest_wall,
                                 best_angle, gap_steering, wall_steering,
                                 steering, filtered_steering,
                                 front_near_center, front_far_x, front_near_wide, front_center_hits);
    }

    void applySpeedSlewAndPublish(double v_des, double w_cmd,
                                  double left_min, double right_min, double wall_error,
                                  double closest_wall, double best_angle,
                                  double gap_steering, double wall_steering,
                                  double steering_raw, double steering_filtered,
                                  double front_near_center, double front_far_x,
                                  double front_near_wide, int front_hits)
    {
        // Faster slew so it actually accelerates
        const double accel_up   = 140.0;
        const double accel_down = 180.0;

        rclcpp::Time now = this->now();
        if (last_speed_time_.nanoseconds() == 0) last_speed_time_ = now;
        double dt = (now - last_speed_time_).seconds();
        last_speed_time_ = now;
        dt = clamp(dt, 0.0, 0.1);

        double max_up = accel_up * dt;
        double max_dn = accel_down * dt;

        if (v_des > current_speed_)
            current_speed_ = approach(current_speed_, v_des, max_up);
        else
            current_speed_ = approach(current_speed_, v_des, max_dn);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = current_speed_;
        cmd.angular.z = w_cmd;

        RCLCPP_INFO(this->get_logger(),
            "front_far_x=%.2f front_near_center=%.2f front_near_wide=%.2f hits=%d | "
            "closest_wall=%.2f | w=%.2f | v_des=%.2f v_out=%.2f | "
            "L=%.2f R=%.2f err=%.2f best_ang=%.1f° gap=%.2f wall=%.2f",
            front_far_x, front_near_center, front_near_wide, front_hits,
            closest_wall,
            w_cmd,
            v_des, current_speed_,
            left_min, right_min, wall_error, best_angle * 180.0 / M_PI,
            gap_steering, wall_steering);

        cmd_pub_->publish(cmd);
        publishSteeringArc(cmd.angular.z, cmd.linear.x);
    }

    void publishSteeringArc(double yaw_rate, double speed)
    {
        visualization_msgs::msg::Marker arc;
        arc.header.frame_id = "base_link";
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
        const double total_time = 2.0;
        int steps = static_cast<int>(total_time / dt);

        double x = 0.0, y = 0.0, theta = 0.0;

        for (int i = 0; i < steps; ++i)
        {
            geometry_msgs::msg::Point p;
            p.x = -x;
            p.y = y;
            p.z = 0.1;
            arc.points.push_back(p);

            double v = speed;
            double w = yaw_rate;

            x -= v * dt * std::cos(theta);
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
