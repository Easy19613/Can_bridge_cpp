#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/msg/point.hpp"
#include "can_bridge_cpp/dm4340.hpp"
#include "can_bridge_cpp/mathmetica_model.hpp"
#include <cmath>

constexpr float PI = 3.14159265358979323846f;
constexpr float Q1_MIN = -PI/2.0f;
constexpr float Q1_MAX = PI/2.0f;
constexpr float Q2_MIN = 0.0f;
constexpr float Q2_MAX = 1.83;
constexpr float Q3_MIN = -8.0f*PI/10.0f;
constexpr float Q3_MAX = 2.0f*PI/3.0f;
constexpr float Velocity_MAX = 12.5f;
constexpr float Velocity_MIN = 0.0f;

double joint_velocity=0.25;
double control_mode_num=5.0;

enum class State {
    JoyController,
    JointAutoServo,
    JoyControllerPoint
};

const double first_q1=0; const double first_q2=0; const double first_q3=0;//初始位姿
const double zero_offset_x=73.36; const double zero_offset_y=0; const double zero_offset_z=33.17;//末端位置的零位偏移
const double zero_offset_q1=0; const double zero_offset_q2=17.24*PI/180; const double zero_offset_q3=0.0;

Eigen::Vector3d first_target=forwardKinematics(first_q1,first_q2,first_q3);

std::array<uint8_t, 8> arr=control_pos_vel(0,5);
std::array<uint8_t, 8> arr1=control_pos_vel(0,5);
std::array<uint8_t, 8> arr2=control_pos_vel(0,5);

double x_add=0; double y_add=0; double z_add=0;
double mode_add=0; double velocity_add=0;
inline double current_q1=0; inline double current_q2=0; inline double current_q3=0;//电机回传的角度值
// extern double current_q1;
// extern double current_q2;
// extern double current_q3;//电机回传的角度值
double desired_q1=0; double desired_q2=0; double desired_q3=0;//电机跟踪的角度值

int Relay_state=0; //继电器状态
int terminal_motor_state=0; //终端电机状态


// Eigen::Vector3d point_message=forwardKinematics(desired_q1+zero_offset_x,desired_q2+zero_offset_y,desired_q3+zero_offset_z);
// geometry_msgs::msg::Point desired_point_message = []() {
//     geometry_msgs::msg::Point p;
//     p.x = point_message[0];
//     p.y = point_message[1];
//     p.z = point_message[2];
//     return p;
// }();

geometry_msgs::msg::Point desired_point_message = []() {
    geometry_msgs::msg::Point p;
    p.x = 150.0;
    p.y = 0.0;
    p.z = 100.0;
    return p;
}();

geometry_msgs::msg::Point visual_point = []() {
    geometry_msgs::msg::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    return p;
}();



void limitEndEffectorPosition(double* x, double* y, double* z, double L1, double L2) {
    // 检查指针有效性
    if (x == nullptr || y == nullptr || z == nullptr) {
        return; // 或者抛出异常，根据您的需求
    }

    // 计算点到原点的距离
    double distance = sqrt((*x)*(*x) + (*y)*(*y) + (*z)*(*z));
    double max_reach = L1 + L2;
    
    // 如果点在可达空间内，直接返回
    if (distance <= max_reach) {
        return;
    }
    
    // 如果点超出可达空间，将其投影到可达空间的边界上
    double scale = max_reach / distance;
    *x *= scale;
    *y *= scale;
    *z *= scale;
}

class ArmState {
public:
    ArmState();

    void run(bool isAtTarget){
        switch (currentState)
        {
        case State::JoyController:
            joy_controller();
            break;
    
        case State::JointAutoServo:
            joint_auto_servo(isAtTarget);
            break;
        
        case State::JoyControllerPoint:
            joy_cotroller_point();
            break;
        }
    }
    
    void updateState(int new_mode) {
    (void)new_mode; // 显式标记未使用参数
    if (control_mode_num>=0 && control_mode_num<=3) {
        currentState = State::JoyController;
    } else if (control_mode_num>=4 && control_mode_num<=7) {
        currentState = State::JointAutoServo;
    }
    else if (control_mode_num>=8 && control_mode_num<=11) {
        currentState = State::JoyControllerPoint;
    }
    // 可以添加状态变更回调或通知
    if(control_mode_num>11){control_mode_num=0;}//超出范围的控制模式切换到初始模式
    }

    void joy_controller(){
        updateState(control_mode_num);
        //关节角度值累加
        // 使用增量控制并重置增量
        desired_q1 += x_add*0.005;
        desired_q2 += y_add*0.005;
        desired_q3 += z_add*0.005;
        joint_velocity+=velocity_add;
        control_mode_num+=mode_add;
        // x_add = 0;y_add = 0;z_add = 0;
        desired_q1 = clamp_joint_angle(desired_q1, Q1_MIN, Q1_MAX);
        desired_q2 = clamp_joint_angle(desired_q2, Q2_MIN, Q2_MAX);
        desired_q3 = clamp_joint_angle(desired_q3, Q3_MIN, Q3_MAX);
        arr=control_pos_vel(desired_q1*70.0f/15.0f,joint_velocity);
        arr1=control_pos_vel(desired_q2,joint_velocity);
        arr2=control_pos_vel(-desired_q3,joint_velocity);
    }
    


void joint_auto_servo(bool isAtTarget) {
    static bool canAddNewPoint = true;
    updateState(control_mode_num);

    // 如果到达目标，重置标记并清零视觉点（避免重复使用）
    if (isAtTarget) {
        // visual_point.x = 0;
        // visual_point.y = 0;
        // visual_point.z = 0;
        canAddNewPoint = true;  // 允许累加下一个视觉点
    }

    // 仅当存在新视觉点且允许累加时执行
    if (canAddNewPoint && (visual_point.x != 0 || visual_point.y != 0 || visual_point.z != 0)) {
        desired_point_message.x += visual_point.x - 100;
        desired_point_message.y += visual_point.y;
        desired_point_message.z += visual_point.z;
        canAddNewPoint = false;  // 防止重复累加
    }


    // 后续计算（限位、运动学、控制等）
    limitEndEffectorPosition(&desired_point_message.x, &desired_point_message.y, &desired_point_message.z, L1, L2);
    auto angles = inverseKinematics(desired_point_message.x, desired_point_message.y, desired_point_message.z);
    
    desired_q1 = angles[0];
    desired_q2 = angles[1] - zero_offset_q2;
    desired_q3 = angles[2] - zero_offset_q3;

    joint_velocity += velocity_add;
    control_mode_num += mode_add;

    desired_q1 = clamp_joint_angle(desired_q1, Q1_MIN, Q1_MAX);
    desired_q2 = clamp_joint_angle(desired_q2, Q2_MIN, Q2_MAX);
    desired_q3 = clamp_joint_angle(desired_q3, Q3_MIN, Q3_MAX);

    arr = control_pos_vel(desired_q1 * 70.0f / 15.0f, joint_velocity);
    arr1 = control_pos_vel(desired_q2, joint_velocity);
    arr2 = control_pos_vel(-desired_q3, joint_velocity);

}

    void joy_cotroller_point(){
        updateState(control_mode_num);
        // double last_desired_q1=desired_q1;//补偿底座的空隙
        //手柄控制
        desired_point_message.x+=x_add;
        desired_point_message.y+=y_add;
        desired_point_message.z+=z_add;
        joint_velocity+=velocity_add;
        control_mode_num+=mode_add;
        //解算
        limitEndEffectorPosition(&desired_point_message.x,&desired_point_message.y,&desired_point_message.z,L1,L2);
        auto angles = inverseKinematics(desired_point_message.x, desired_point_message.y, desired_point_message.z);

        // checkParallelLinkConstraints(angles);

        desired_q1 = angles[0];  // 访问第1个元素
        desired_q2 = angles[1]-zero_offset_q2;  // 访问第2个元素
        desired_q3 = angles[2]-zero_offset_q3;  // 访问第3个元素
        //限幅
        // if(PI-abs(desired_q2)<PI-abs(desired_q3)){desired_q3=abs(desired_q2)-0.65;}//平行四边形约束

        desired_q1 = clamp_joint_angle(desired_q1, Q1_MIN, Q1_MAX);
        desired_q2 = clamp_joint_angle(desired_q2, Q2_MIN, Q2_MAX);
        desired_q3 = clamp_joint_angle(desired_q3, Q3_MIN, Q3_MAX);
        joint_velocity = clamp_joint_angle(joint_velocity, Velocity_MIN, Velocity_MAX);

        // if(desired_q1>0 && last_desired_q1-desired_q1>0){desired_q1-=0.1;}//正转切反转的补偿
        // if(desired_q1<0 && last_desired_q1-desired_q1<0){desired_q1+=0.1;}//反转切正转的补偿
        arr=control_pos_vel(desired_q1*70.0f/15.0f, 11);
        arr1=control_pos_vel(desired_q2,joint_velocity);
        arr2=control_pos_vel(-desired_q3,joint_velocity);
    }
private:
    State currentState;
};

ArmState::ArmState() : currentState(State::JointAutoServo) {}//成员初始化列表
