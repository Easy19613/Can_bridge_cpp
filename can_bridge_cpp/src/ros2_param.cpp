#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "can_bridge_cpp/state_machine.hpp"

using namespace std::chrono_literals;

// 声明外部变量（在state_machine.hpp中定义）
extern int terminal_motor_state;
extern int Relay_state;

class ParamUpdater : public rclcpp::Node {
public:
    ParamUpdater() 
    : Node("control_param") {
        // 创建定时器周期性更新参数
        timer_ = this->create_wall_timer(1.5s, std::bind(&ParamUpdater::timer_callback, this));
        point3_sub_ = this->create_subscription<geometry_msgs::msg::Point>("controlled_point3", 10, std::bind(&ParamUpdater::Point3Callback, this, std::placeholders::_1));
    }

private:
    void timer_callback() {
        // 更新param1基于Relay_state
        if (Relay_state == 0) {
            system("ros2 param set /multi_param_node param1 false");
        } else if (Relay_state == 1) {
            system("ros2 param set /multi_param_node param1 true");
        }
        
        // 更新param2基于terminal_motor_state
        if (terminal_motor_state == 0) {
            system("ros2 param set /multi_param_node param2 false");
        } else if (terminal_motor_state == 1) {
            system("ros2 param set /multi_param_node param2 true");
        }
    }
    
    void Point3Callback(const geometry_msgs::msg::Point::SharedPtr msg){
        Relay_state = msg->y;
        terminal_motor_state = msg->z;
    }

    rclcpp::SyncParametersClient::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point3_sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    // 创建单线程执行器
    auto executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    
    // 创建节点并添加到执行器
    auto node = std::make_shared<ParamUpdater>();
    executor->add_node(node);
    
    // 运行执行器
    executor->spin();
    
    rclcpp::shutdown();
    return 0;
}
