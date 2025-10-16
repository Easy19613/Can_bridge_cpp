#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "can_bridge_cpp/state_machine.hpp"



class JoyControlNode : public rclcpp::Node {
public:
    JoyControlNode() : Node("joy_control_node") {        
        // 创建发布者和订阅者
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyControlNode::joyCallback, this, std::placeholders::_1));
            
        point_pub_ = this->create_publisher<geometry_msgs::msg::Point>("controlled_point", 10);
        point2_pub_ = this->create_publisher<geometry_msgs::msg::Point>("controlled_point2", 10);
        point3_pub_ = this->create_publisher<geometry_msgs::msg::Point>("controlled_point3", 10);
        point4_pub_ = this->create_publisher<geometry_msgs::msg::Point>("controlled_point4", 10);
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // Xbox手柄摇杆映射（通常左摇杆是axes[0]和axes[1]，右摇杆是axes[3]和axes[4]）
        double left_y = msg->axes[1];  // 左摇杆Y轴 [-1.0, 1.0]
        double right_y = msg->axes[4]; // 右摇杆Y轴 [-1.0, 1.0] (用于Z轴控制)
        double button_axe_x = msg->axes[7];
        double button_axe_y = msg->axes[6];
        double button_6 = msg->buttons[6];
        double button_0 = msg->buttons[0];
        double button_1 = msg->buttons[1];
        double button_2 = msg->buttons[2];
        double button_3 = msg->buttons[3];
        // 创建三维点
        geometry_msgs::msg::Point point_add;
        geometry_msgs::msg::Point point2_add;
        geometry_msgs::msg::Point point3_add;
        geometry_msgs::msg::Point point4_add;
        // 使用合理的控制量值 (0.01-0.1范围)
        point_add.x = button_axe_x*0.8*joint_velocity;
        point_add.y = button_axe_y*0.8*joint_velocity;
        point_add.z = right_y * 0.4; 
        point2_add.x = 0.0;
        point2_add.y = left_y*0.01;
        point2_add.z = button_6*0.01;
        point3_add.x = button_0;
        point3_add.y = button_1;
        point3_add.z = 0.0;
        point4_add.x = button_2;
        point4_add.y = button_3;
        point4_add.z = 0.0;
        // 发布控制点
        point_pub_->publish(point_add);
        point2_pub_->publish(point2_add);
        point3_pub_->publish(point3_add);
        point4_pub_->publish(point4_add);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point2_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point3_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point4_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
