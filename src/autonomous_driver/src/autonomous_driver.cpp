#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/qos.hpp>


class AutonomousDriver : public rclcpp::Node
{
public:
    AutonomousDriver() : Node("autonomous_driver")
    {
        RCLCPP_INFO(this->get_logger(), "AutonomousDriver node CONSTRUCTOR running");
        
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/top_3d_lidar_plugin/out",
            sensor_qos,
            std::bind(&AutonomousDriver::lidarCallback, this, std::placeholders::_1));
    }

private:
        void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        const double self_radius      = 0.6;   // ignore points this close to the sensor
        const double self_radius_sq   = self_radius * self_radius;

        const double min_forward      = 0.5;   // start looking this far ahead (m)
        const double max_forward      = 5.0;   // up to this far ahead (m)
        const double max_side         = 5.0;   // ignore points farther than this sideways
        const double max_range        = 50.0;  // large number for "no reading"

        double left_min  = max_range;
        double right_min = max_range;

        int removed_self = 0;

        for (const auto &pt : cloud->points)
        {
            double x = pt.x;
            double y = pt.y;
            double z = pt.z;
            (void)z; // unused, but kept for clarity / future use

            // 1) Remove points on our own car (near the sensor)
            double r2 = x * x + y * y;
            if (r2 < self_radius_sq) {
                removed_self++;
                continue;
            }

            // 2) Keep only points roughly in front of the car
            //    This assumes +X is forward. If in your setup forward is -X,
            //    change this block to:
            //
            //    if (x > -min_forward || x < -max_forward) continue;
            //
            if (x < min_forward || x > max_forward) {
                continue;
            }

            // 3) Limit side extent
            if (std::abs(y) > max_side) {
                continue;
            }

            // 4) Classify as left or right wall
            double dist = std::hypot(x, y);

            if (y > 0.0) {
                // left side
                if (dist < left_min)
                    left_min = dist;
            } else {
                // right side
                if (dist < right_min)
                    right_min = dist;
            }
        }

        if (cloud->points.empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Received empty point cloud");
        }

        // If one side wasn't seen, assume it's far away
        if (left_min  == max_range) left_min  = max_forward;
        if (right_min == max_range) right_min = max_forward;

        // Error: positive if more space on right than left
        double error = left_min - right_min;

        // Simple P controller for steering
        const double k_steer   = 0.6;  // steering gain
        const double max_steer = 0.6;  // rad

        double steering = -k_steer * error;  // minus because of convention
        if (steering >  max_steer) steering =  max_steer;
        if (steering < -max_steer) steering = -max_steer;

        // Base speed, slow down if very close to walls
        double closest = std::min(left_min, right_min);
        double speed   = 100.0;          // m/s, tune as needed

        if (closest < 1.0) speed = 0.5;
        if (closest < 0.5) speed = 0.0;  // emergency stop

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = speed;
        cmd.angular.z = -steering;

        RCLCPP_INFO(
            this->get_logger(),
            "Cloud pts=%zu, removed_self=%d, L=%.2f R=%.2f err=%.2f -> v=%.2f, steer=%.2f",
            cloud->points.size(), removed_self, left_min, right_min, error,
            cmd.linear.x, cmd.angular.z);

        cmd_pub_->publish(cmd);
    }



    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomousDriver>());
    rclcpp::shutdown();
    return 0;
}
