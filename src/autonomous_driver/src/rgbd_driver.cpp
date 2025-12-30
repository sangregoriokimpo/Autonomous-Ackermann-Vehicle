#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <limits>
#include <cmath>

#include "autonomous_driver/kalman_1d.hpp"


class RgbdDriver : public rclcpp::Node
{
public:
    RgbdDriver() : Node("rgbd_driver")
    {
        RCLCPP_INFO(this->get_logger(), "RgbdDriver node started");

        // Relative topic names so namespaces work: /botX/cmd_vel, /botX/front_rgbd/depth
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "front_rgbd/depth/image_raw",
            10,
            std::bind(&RgbdDriver::depthCallback, this, std::placeholders::_1));
    }

private:
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Expect 32-bit float depth image (meters)
        if (msg->encoding != "32FC1") {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Expected 32FC1 depth image, got '%s'", msg->encoding.c_str());
            return;
        }

        const int width  = msg->width;
        const int height = msg->height;
        if (width == 0 || height == 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Received empty depth image");
            return;
        }

        const int row = height / 2;  // middle row

        const float *data = reinterpret_cast<const float *>(&msg->data[0]);
        const int stride  = msg->step / sizeof(float);

        float left_min  = std::numeric_limits<float>::infinity();
        float right_min = std::numeric_limits<float>::infinity();

        // Ignore super-close noise and super-far readings
        const float min_valid = 0.1f;
        const float max_valid = 50.0f;

        for (int u = 0; u < width; ++u) {
            float d = data[row * stride + u];
            if (!std::isfinite(d) || d < min_valid || d > max_valid)
                continue;

            if (u < width / 2) {
                if (d < left_min) left_min = d;
            } else {
                if (d < right_min) right_min = d;
            }
        }

        // If one side wasn't seen, treat it as far away
        if (!std::isfinite(left_min))  left_min  = max_valid;
        if (!std::isfinite(right_min)) right_min = max_valid;

        // Positive error = more space on right than left
        double error = static_cast<double>(left_min - right_min);

        // Simple P steering controller
        const double k_steer   = 0.3;
        const double max_steer = 0.3;
        double steering = -k_steer * error;  // minus to match your convention

        if (steering >  max_steer) steering =  max_steer;
        if (steering < -max_steer) steering = -max_steer;

        // Speed: fast base, only slow for emergency
        double closest = static_cast<double>(std::min(left_min, right_min));

        double speed = 10.0;          // base racing speed (m/s)
        if (closest < 0.5) speed = 0.0;  // e-stop if something is very close

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = speed;
        // Front is -X in your base frame; keep your double-minus convention:
        cmd.angular.z = -steering;

        RCLCPP_INFO(
            this->get_logger(),
            "Depth row mid: L=%.2f R=%.2f closest=%.2f err=%.2f -> v=%.2f, steer=%.2f",
            left_min, right_min, closest, error, cmd.linear.x, cmd.angular.z);

        cmd_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RgbdDriver>());
    rclcpp::shutdown();
    return 0;
}
