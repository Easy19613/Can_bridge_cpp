#include "rclcpp/rclcpp.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include "can_msgs/msg/frame.hpp"
#include <eigen3/Eigen/Dense>
#include <cstdlib>
#include "sensor_msgs/msg/joy.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/timer.hpp>  // TimerOptions的定义
#include "can_bridge_cpp/state_machine.hpp"
#include <functional>

// atexit处理函数声明
static void atexit_handler();

// can发送者工作空间
using namespace drivers::socketcan;
// 定时器时间参数工作空间
using namespace std::chrono_literals;
const double x_offset=0.163;
ArmState control_mode;
double ros_current_q1=0;
double ros_current_q2=0;
double ros_current_q3=0;
bool isAtTarget = true;

geometry_msgs::msg::Point last_visual_point = []() {
    geometry_msgs::msg::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    return p;
}();

void signalHandler(int signum)
{
    std::cout << "\n >>>>>>接收到SIGINT(" << signum << ")，准备终止程序。" << std::endl;

    std::this_thread::sleep_for(std::chrono::microseconds(2000)); // 微秒
    // 在此执行需要在Ctrl+C时要做的操作
    // 例如执行清理函数，或者只是标志需要退出
    // 如果希望atexit函数也被调用，可在这里调用exit()

    atexit_handler();  // 直接调用退出处理函数
    std::exit(signum); // 程序正常退出
}

class JointStatePublisher : public rclcpp::Node
{
  public:
    JointStatePublisher(std::string name)
    : Node(name)
    {
      joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
      desired_point_message_pub_ = this->create_publisher<geometry_msgs::msg::Point>("desired_point_message", 10);


      timer_ = this->create_wall_timer(
      10ms, std::bind(&JointStatePublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto joint_state_message = sensor_msgs::msg::JointState();
        joint_state_message.header.stamp = this->now();  // 当前时间戳
        joint_state_message.name = {"joint1", "joint2", "joint3"}; // 关节名称列表
        joint_state_message.position = {desired_q1, desired_q2, desired_q3};      // 关节位置（弧度）
        joint_state_message.velocity = {0.0, 0.0, 0.0};       // 关节速度（可选）
        joint_state_message.effort = {0.0, 0.0, 0.0};         // 关节力矩（可选）
        // 发布消息
        joint_state_pub_->publish(joint_state_message);
        desired_point_message_pub_->publish(desired_point_message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr desired_point_message_pub_;
};

class CanSender : public rclcpp::Node
{
public:
    /* 构造函数 */
    CanSender(std::string name) : Node(name) {
        //RCLCPP_INFO(this->get_logger(), "节点的名称为:%s.",name.c_str());
        /* 创建发送者 */
        can0_sender_ = std::make_shared<SocketCanSender>(
            // can接口
            "q1_can",
            // 启用can_fd
            false,
            // 默认配置
            CanId(
                // 默认ID
                0x101, 
                // 时间戳，0表示立即发送
                0,
                // 数据帧、遥控帧、错误帧
                FrameType::DATA,
                // 标准格式、扩展格式
                StandardFrame_{})
        );
        can1_sender_ = std::make_shared<SocketCanSender>(
            "q2_can",false,CanId(0x102,0,FrameType::DATA,StandardFrame_{}) 
        );
        can2_sender_ = std::make_shared<SocketCanSender>(
            "q3_can",false,
            CanId(0x103,0,FrameType::DATA,StandardFrame_{}) 
        );

        dm4340_Init();
        /* 创建定时器 */
        timer_ = this->create_wall_timer(4ms, std::bind(&CanSender::TimerCallback, this));
        point_sub_ = this->create_subscription<geometry_msgs::msg::Point>("controlled_point", 10, std::bind(&CanSender::PointCallback, this, std::placeholders::_1));
        point2_sub_ = this->create_subscription<geometry_msgs::msg::Point>("controlled_point2", 10, std::bind(&CanSender::Point2Callback, this, std::placeholders::_1));
        visual_point_sub_ = this->create_subscription<geometry_msgs::msg::Point>("object_point", 10, std::bind(&CanSender::VisualPointCallback, this, std::placeholders::_1));
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("current_joint_states", 10, std::bind(&CanSender::JointStateCallback, this, std::placeholders::_1));
    }

    bool configure_can_interface(const std::string& ifname, int bitrate) {
        std::string cmd1 = "sudo ip link set " + ifname + " type can bitrate " + std::to_string(bitrate);
        std::string cmd2 = "sudo ip link set " + ifname + " up";
        std::string cmd3 = "sudo ifconfig " + ifname + " txqueuelen 10000";
        
        if (system(cmd1.c_str()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "已经设置过比特率");
            return false;
        }
        rclcpp::sleep_for(100ms);
        
        if (system(cmd2.c_str()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "启用接口失败");
            return false;
        }
        rclcpp::sleep_for(100ms);
        
        if (system(cmd3.c_str()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "配置缓冲区失败");
            return false;
        }
        rclcpp::sleep_for(100ms);
        return true;
    }

    bool configure_can_down(const std::string& ifname) {
        std::string cmd = "sudo ip link set " + ifname + " down";
        
        if (system(cmd.c_str()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "关闭接口失败");
            return false;
        }
        rclcpp::sleep_for(100ms);
        
        return true;
    }

    void dm4340_Init(){
        unsigned char init[]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
        // RCLCPP_INFO(this->get_logger(), "设置比特率");
        configure_can_interface("q1_can", 1000000);
        configure_can_interface("q2_can", 1000000);
        configure_can_interface("q3_can", 1000000);
        // RCLCPP_INFO(this->get_logger(), "使能电机");

        sendWithRetry(can0_sender_, init, 0x101);
        sendWithRetry(can1_sender_, init, 0x102);
        sendWithRetry(can2_sender_, init, 0x103);
    }
    
    void dm4340_Stop(){
        unsigned char stop[]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
        timer_->cancel();

        can0_sender_->send(stop,CanId(0x101,0,FrameType::DATA,StandardFrame_{}),100ms);
        can1_sender_->send(stop,CanId(0x102,0,FrameType::DATA,StandardFrame_{}),100ms);
        can2_sender_->send(stop,CanId(0x103,0,FrameType::DATA,StandardFrame_{}),100ms);

        configure_can_down("q1_can");
        configure_can_down("q2_can");
        configure_can_down("q3_can");
        // RCLCPP_INFO(this->get_logger(), "终止");
    }


private:
    template<typename T>
    void sendWithRetry(std::shared_ptr<drivers::socketcan::SocketCanSender>& sender, const T& data, uint32_t can_id) {
        bool success = false;
        int retry_count = 0;
        const int max_retries = 5;
        
        while (!success && retry_count < max_retries) {
            try {
                if constexpr (std::is_pointer_v<std::decay_t<T>>) {
                    // 处理原始数组
                    sender->send(
                        data,
                        CanId(can_id, 0, FrameType::DATA, StandardFrame_{}),
                        100ms
                    );
                } else {
                    // 处理容器数据
                    sender->send(
                        data.data(),
                        data.size(),
                        CanId(can_id, 0, FrameType::DATA, StandardFrame_{}),
                        100ms
                    );
                }
                success = true;
            } catch (const std::runtime_error& e) {
                RCLCPP_ERROR(this->get_logger(), "发送失败(尝试 %d/%d): %s", 
                            retry_count + 1, max_retries, e.what());
                rclcpp::sleep_for(1s);
                retry_count++;
            }
        }
        
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "发送失败，已达到最大重试次数");
        }
    }

    /* 定时器回调函数 */
    void TimerCallback(void) {
        try
        {
            isAtTarget = (fabs(ros_current_q1/70.0f*15.0f - desired_q1) < 0.05) &&
                         (fabs(ros_current_q2 - desired_q2) < 0.05) &&
                         (fabs(ros_current_q3 + desired_q3) < 0.05);
            control_mode.run(isAtTarget); // 循环运行
            // RCLCPP_INFO(this->get_logger(), "JointValues: %.4f, %.4f, %.4f",desired_q1,desired_q2,desired_q3);
            // RCLCPP_INFO(this->get_logger(), "JointValues: %.4f, %.4f, %.4f",ros_current_q1,ros_current_q2,ros_current_q3);
            RCLCPP_INFO(this->get_logger(), "PointValues: %.4f, %.4f, %.4f",desired_point_message.x,desired_point_message.y,desired_point_message.z);
            RCLCPP_INFO(this->get_logger(), "PointValues: %.4f, %.4f, %.4f",ros_current_q1 - desired_q1,ros_current_q2 - desired_q2,ros_current_q3 + desired_q3);
            RCLCPP_INFO(this->get_logger(), "Bool value: %d", isAtTarget);
            // RCLCPP_INFO(this->get_logger(), "VelocityValues: %.2f",joint_velocity);
            //     RCLCPP_INFO(this->get_logger(), "ModeValues: %.2f",control_mode_num);
            // if (control_mode_num>=0 && control_mode_num<=3) {
            //     RCLCPP_INFO(this->get_logger(), "Mode:JoyController");
            // } else if (control_mode_num>=4 && control_mode_num<=7) {
            //     RCLCPP_INFO(this->get_logger(), "Mode:JointAutoServo");
            // }
            // else if (control_mode_num>=8 && control_mode_num<=11) {
            //     RCLCPP_INFO(this->get_logger(), "Mode:JoyControllerPoint");
            // }
        
        }
        catch (const std::exception &e)
        {
            std::cerr << "程序异常：" << e.what() << std::endl;
        }
        sendWithRetry(can0_sender_, arr, 0x101);
        sendWithRetry(can1_sender_, arr1, 0x102);
        sendWithRetry(can2_sender_, arr2, 0x103);
    }
    /*手柄的回调函数*/
    void PointCallback(const geometry_msgs::msg::Point::SharedPtr msg){
        x_add = msg->x;
        y_add = msg->y; 
        z_add = msg->z;
    }
    void Point2Callback(const geometry_msgs::msg::Point::SharedPtr msg){
        velocity_add = msg->y;
        mode_add = msg->z;
    }
    
    void VisualPointCallback(const geometry_msgs::msg::Point::SharedPtr msg){
        visual_point.x = 0;
        visual_point.y = 0;
        visual_point.z = 0;
        if (isAtTarget) {
            visual_point.x=msg->x;
            visual_point.y=msg->y;
            visual_point.z=msg->z;
        }
    }

    void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
        // 更新当前关节状态
        ros_current_q1 = msg->position[0];
        ros_current_q2 = msg->position[1];
        ros_current_q3 = msg->position[2];
    }
    std::shared_ptr<drivers::socketcan::SocketCanSender> can0_sender_;
    std::shared_ptr<drivers::socketcan::SocketCanSender> can1_sender_;
    std::shared_ptr<drivers::socketcan::SocketCanSender> can2_sender_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point2_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr visual_point_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};




// 全局弱指针用于atexit处理
static std::weak_ptr<CanSender> g_can_sender_weak;

// atexit处理函数
static void atexit_handler() {
    if (auto node = g_can_sender_weak.lock()) {
        node->dm4340_Stop();
    }
    // std::cout << "\n >>>>>>失能机械臂" << std::endl;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node1 = std::make_shared<CanSender>("can_sender");
    auto node2 = std::make_shared<JointStatePublisher>("jointstate_publisher");
    
    // 保存弱引用并注册atexit处理
    g_can_sender_weak = node1;
    std::atexit(atexit_handler);
    // 注册SIGINT信号处理函数
    std::signal(SIGINT, signalHandler);
    // 运行节点
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
