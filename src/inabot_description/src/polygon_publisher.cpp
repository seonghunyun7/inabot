#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"

class PolygonPublisher : public rclcpp::Node
{
public:
    PolygonPublisher() : Node("polygon_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("polygon_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PolygonPublisher::publish_polygon, this));
    }

private:
    void publish_polygon()
    {
        auto msg = geometry_msgs::msg::PolygonStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "base_link";

        geometry_msgs::msg::Point32 p1, p2, p3, p4;
        p1.x = 0.401; p1.y = 0.27346; p1.z = 0.0;
        p2.x = 0.401; p2.y = -0.27346; p2.z = 0.0;
        p3.x = -0.401; p3.y = -0.27346; p3.z = 0.0;
        p4.x = -0.401; p4.y = 0.27346; p4.z = 0.0;

        msg.polygon.points = {p1, p2, p3, p4};

        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PolygonPublisher>());
    rclcpp::shutdown();
    return 0;
}
