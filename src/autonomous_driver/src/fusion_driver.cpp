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
    AdaptivePIDController wall_pid_;
    autonomous_driver::Kalman1D steering_filter_{0.02, 0.2};  // Q, R 
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr rgbd_cloud_;
    std::mutex rgbd_mutex_;
    
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
                } else if (msg->encoding == "16UC1") {
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
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Fused cloud: LIDAR=%zu + RGBD=%zu = %zu points",
                cloud->points.size() - rgbd_cloud->points.size(),
                rgbd_cloud->points.size(),
                cloud->points.size());
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "LIDAR: Received %zu points", cloud->points.size());

        const double self_radius_sq = 0.4 * 0.4;
        const double min_x = -25.0;
        const double max_x = -1.0;
        const double max_y = 5.0;
        const double max_range = 50.0;

        double left_min = max_range;
        double right_min = max_range;
        
        const double gap_threshold = 1.0;
        const double angle_res = 1.0 * M_PI / 180.0;
        const int num_bins = 180;
        std::vector<double> min_ranges(num_bins, std::numeric_limits<double>::infinity());

        for (const auto& pt : cloud->points)
        {
            double x = pt.x;
            double y = pt.y;
            double r2 = x * x + y * y;

            if (r2 < self_radius_sq) continue;
            if (x > max_x || x < min_x) continue;
            if (std::abs(y) > max_y) continue;

            double xf = -x;
            double yf = y;
            if (xf <= 0.0) continue;

            double dist = std::hypot(xf, yf);

            if (yf > 0.0) {
                if (dist < left_min) left_min = dist;
            } else {
                if (dist < right_min) right_min = dist;
            }

            double angle = std::atan2(yf, xf);
            if (angle < -M_PI / 2 || angle > M_PI / 2) continue;

            int bin = static_cast<int>((angle + M_PI / 2) / angle_res);
            if (bin < 0 || bin >= num_bins) continue;

            if (dist < min_ranges[bin])
                min_ranges[bin] = dist;
        }

        if (left_min == max_range) left_min = 15.0;
        if (right_min == max_range) right_min = 15.0;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Walls detected - L: %.2f R: %.2f (before filtering had %zu points)",
            left_min, right_min, cloud->points.size());

        double wall_error = left_min - right_min;
        double closest_wall = std::min(left_min, right_min);
        
        double wall_steering = wall_pid_.compute(wall_error, closest_wall, this->now());

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
        double best_range = min_ranges[best_bin];

        RCLCPP_INFO(this->get_logger(),
            "DEBUG: best_bin=%d/%d best_angle=%.1f° best_range=%.2fm forward_range=%.2fm",
            best_bin, num_bins, best_angle * 180.0 / M_PI, 
            std::isfinite(best_range) ? best_range : -1.0,
            std::isfinite(min_ranges[forward_bin]) ? min_ranges[forward_bin] : -1.0);

        double steering = 0.0;
        double gap_steering = 0.0;
        
        if (best_score < 0) {
            RCLCPP_WARN(this->get_logger(), "No gap! Using wall-following only");
            steering = wall_steering;
        } else {
            const double max_steering_angle = M_PI / 3.0;
            if (best_angle > max_steering_angle) best_angle = max_steering_angle;
            if (best_angle < -max_steering_angle) best_angle = -max_steering_angle;
            
            gap_steering = 5.5 * best_angle;
            
            steering = 0.5 * gap_steering + 0.5 * wall_steering;
        }

        const double max_angular_vel = 2.0;
        if (steering > max_angular_vel) steering = max_angular_vel;
        if (steering < -max_angular_vel) steering = -max_angular_vel;

        double front_min = std::numeric_limits<double>::infinity();
        for (int i = forward_bin - 10; i <= forward_bin + 10; ++i) {
            if (i >= 0 && i < num_bins)
                front_min = std::min(front_min, min_ranges[i]);
        }
        if (!std::isfinite(front_min)) front_min = 50.0;

        double closest = std::min(left_min, right_min);
        
        double speed = 8.0;
        // if (front_min < 4.0 || closest < 3.0) speed = 6.0;
        // if (front_min < 2.5 || closest < 2.0) speed = 4.0;
        // if (front_min < 1.5 || closest < 1.2) speed = 2.0;
        // if (front_min < 1.0 || closest < 0.8) speed = 1.0;
        // if (front_min < 0.6 || closest < 0.5) speed = 0.0;

        if (front_min < 3.0 || closest < 2.5) speed = 6.0;
        if (front_min < 1.8 || closest < 1.5) speed = 4.0;
        if (front_min < 1.2 || closest < 1.0) speed = 2.0;
        if (front_min < 0.8 || closest < 0.6) speed = 1.0;
        if (front_min < 0.5 || closest < 0.4) speed = 0.0;


        geometry_msgs::msg::Twist cmd;

        steering_filter_.predict();
        steering_filter_.update(steering);
        double filtered_steering = steering_filter_.value();

        cmd.linear.x = speed;
        //cmd.angular.z = -steering;
        cmd.angular.z = -filtered_steering;


        RCLCPP_INFO(this->get_logger(),
            "L=%.2f R=%.2f err=%.2f closest=%.2f | gap_ang=%.1f° gap_steer=%.2f wall_steer=%.2f | final_steer=%.2f ang_z=%.2f | spd=%.2f",
            left_min, right_min, wall_error, closest,
            ((best_bin + 0.5) * angle_res - M_PI / 2) * 180.0 / M_PI,
            gap_steering, wall_steering, steering, cmd.angular.z, speed);

        cmd_pub_->publish(cmd);
        publishSteeringArc(steering, speed);
    }

    void publishSteeringArc(double steering_angle, double speed)
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
            p.y = -y;
            p.z = 0.1;
            arc.points.push_back(p);

            double v = speed;
            double w = steering_angle;

            x -= v * dt * std::cos(theta);
            y += v * dt * std::sin(theta);
            theta += w * dt;
        }

        marker_pub_->publish(arc);
    }

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