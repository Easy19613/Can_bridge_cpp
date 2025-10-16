#include "rclcpp/rclcpp.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "can_msgs/msg/frame.hpp"
#include "can_bridge_cpp/state_machine.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

// can接收者者工作空间
using namespace drivers::socketcan;
// 定时器时间参数工作空间
using namespace std::chrono_literals;

float current_joint1_state[9]={0};
float current_joint2_state[9]={0};
float current_joint3_state[9]={0};

class JointStatePublisher : public rclcpp::Node
{
  public:
    JointStatePublisher(std::string name)
    : Node(name)
    {
      joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("current_joint_states", 10);
      timer_ = this->create_wall_timer(
      50ms, std::bind(&JointStatePublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto joint_state_message = sensor_msgs::msg::JointState();
        // 填充消息内容
        joint_state_message.header.stamp = this->now();  // 当前时间戳
        joint_state_message.name = {"joint1", "joint2", "joint3", "joint4"}; // 关节名称列表
        joint_state_message.position = {current_joint1_state[0], current_joint2_state[0], current_joint3_state[0], 0};      // 关节位置（弧度）
        joint_state_message.velocity = {current_joint1_state[1], current_joint2_state[1], current_joint3_state[1], 0};       // 关节速度（可选）
        joint_state_message.effort = {current_joint1_state[2], current_joint2_state[2] ,current_joint3_state[2] ,0};         // 关节力矩（可选）
        current_q1 = current_joint1_state[0];
        current_q2 = current_joint2_state[0];
        current_q3 = current_joint3_state[0];
        // 发布消息
        joint_state_pub_->publish(joint_state_message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

class CanReceiver : public rclcpp::Node
{
public:
    /* 构造函数 */
    CanReceiver(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "节点的名称为:%s.", name.c_str());
        can_receiver_0 = std::make_shared<SocketCanReceiver>(
            "q1_can",
            // 是否使用高速CAN
            false
        );
        can_receiver_1 = std::make_shared<SocketCanReceiver>(
            "q2_can",
            // 是否使用高速CAN
            false
        );
        can_receiver_2 = std::make_shared<SocketCanReceiver>(
            "q3_can",
            // 是否使用高速CAN
            false
        );
        timer_ = this->create_wall_timer(4ms, std::bind(&CanReceiver::TimerCallback, this));
    }
   
private:
    void TimerCallback(void)
    {
        try {
            if (processCanMessage(can_receiver_0, can_id_0, current_joint1_state)) {}
            if (processCanMessage(can_receiver_1, can_id_1, current_joint2_state)) {}
            if (processCanMessage(can_receiver_2, can_id_2, current_joint3_state)) {}
        }
        catch (const SocketCanTimeout& e) {
            RCLCPP_DEBUG(this->get_logger(), "接收超时（正常现象）");
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "接收错误: %s", e.what());
        }
    }

    bool processCanMessage(std::shared_ptr<SocketCanReceiver> receiver, CanId& can_id, float* output_data)
    {
        unsigned char message[8] = {0};
        can_id = receiver->receive(message, 100us);
        float *data = dm4340_receive(message);
        
        if(can_id.get() == 0x11 || can_id.get() == 0x12 || can_id.get() == 0x13) {
            RCLCPP_INFO(
                this->get_logger(),
                "ID：%d, 是否扩展格式：%s, 帧格式：%d, 数据长度：%u",
                can_id.get(), 
                can_id.is_extended() ? "是" : "否", 
                static_cast<int>(can_id.frame_type()),
                can_id.length()
            );
            for(int i = 0; i < 3; i++) {
                output_data[i] = data[i];
            }
            return true;
        }
        return false;
    }
    std::shared_ptr<drivers::socketcan::SocketCanReceiver> can_receiver_0;
    std::shared_ptr<drivers::socketcan::SocketCanReceiver> can_receiver_1;
    std::shared_ptr<drivers::socketcan::SocketCanReceiver> can_receiver_2;
    rclcpp::TimerBase::SharedPtr timer_;
    CanId can_id_0;
    CanId can_id_1;
    CanId can_id_2;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node1 = std::make_shared<CanReceiver>("can_receiver");
    auto node2 = std::make_shared<JointStatePublisher>("current_jointstate_publisher");
    // rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 
    8  // 明确指定线程数（根据CPU核心数调整）
    );
    executor.add_node(node1);
    executor.add_node(node2);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
