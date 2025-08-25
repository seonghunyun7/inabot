#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <std_msgs/msg/string.hpp>

class JoyToCmdVelNode : public rclcpp::Node
{

public:
    bool command = false;

    JoyToCmdVelNode()
        : Node("joy_to_cmdvel_node")
    {
        RCLCPP_INFO(this->get_logger(), "start JoyToCmdVelNode");

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sound_publisher_ = this->create_publisher<std_msgs::msg::String>("robot_sound", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&JoyToCmdVelNode::joy_callback, this, std::placeholders::_1)
        );
    }
private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        auto sound_msg = std_msgs::msg::String();

        if(msg->buttons[5] != 0){
            twist_msg.linear.x = msg->axes[1]/3.5;
            // twist_msg.linear.y = msg->axes[0];
            twist_msg.angular.z = msg->axes[2]/2;
            // if(command == true){
            //     command = false;
                sound_msg.data = "default";
                sound_publisher_->publish(sound_msg);
            // }
            cmd_vel_pub_->publish(twist_msg);
        }
        else{
            twist_msg.linear.x = 0.0;
            // twist_msg.linear.y = msg->axes[0];
            twist_msg.angular.z = 0.0;
            // if(command == false)
            // {
                command = true;
                sound_msg.data = "stop";
                sound_publisher_->publish(sound_msg);
                cmd_vel_pub_->publish(twist_msg);
            // }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sound_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToCmdVelNode>());
    rclcpp::shutdown();
    return 0;
}
