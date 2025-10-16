#include <eigen3/Eigen/Dense>


const double L1 = 252.49; // 上臂长度(mm)
const double L2 = 281.83; // 前臂长度(mm)
constexpr double PI2 = 3.141592653589793;

inline float clamp_joint_angle(float angle, float min, float max) {
    return (angle < min) ? min : (angle > max) ? max : angle;
}

//求解出以基座为base的向量，初始姿态为二连杆躺在地上
// Eigen::Vector3d forwardKinematics(double q1, double q2, double q3) {
//     double x = (L1*cos(q2) + L2*cos(q2+q3)) * cos(q1);
//     double y = (L1*cos(q2) + L2*cos(q2+q3)) * sin(q1);
//     double z = L1*sin(q2) + L2*sin(q2+q3);
//     return Eigen::Vector3d(x, y, z);
// }

Eigen::Vector3d forwardKinematics(double q1, double q2, double q3) {
    double c23 = cos(q2 + q3);
    double s23 = sin(q2 + q3);
    double c2 = cos(q2);
    double s2 = sin(q2);
    
    double xy_plane = L1*c2 + L2*c23;
    double x = xy_plane * cos(q1);
    double y = xy_plane * sin(q1);
    double z = L1*s2 + L2*s23;
    
    return Eigen::Vector3d(x, y, z);
}


// Eigen::Vector3d inverseKinematics(double x, double y, double z) {
//     // 底座旋转角计算
//     double q1 = atan2(y, x);
    
//     // 平面投影计算
//     double r_proj = hypot(x, y);
//     double r = hypot(r_proj, z);
    
//     // 检查可达性
//     if(r > (L1 + L2) || r < fabs(L1 - L2)) {
//         // 处理不可达情况，可以抛出异常或返回特殊值
//         return Eigen::Vector3d::Zero(); // 示例处理
//     }
    
//     // 二连杆几何解算 - 只选择肘关节朝上的解
//     // double cos_q3 = (r*r - L1*L1 - L2*L2) / (2.0*L1*L2);
//     double numerator = r*r - (L1*L1 + L2*L2);
//     double denominator = 2.0*L1*L2;
//     double cos_q3 = numerator / denominator;
//     cos_q3 = std::clamp(cos_q3, -1.0, 1.0);
//     double q3 = -acos(cos_q3); // 负号确保肘关节朝上
    
//     // 计算q2 - 肘关节朝上的配置
//     double alpha = atan2(z, r_proj);
//     // double sin_beta = (L2*sin(-q3))/r; // 注意q3现在是负的
//     // sin_beta = std::clamp(sin_beta, -1.0, 1.0);
//     // double beta = asin(sin_beta);
//     double tan_beta = (L2*sin(-q3)) / sqrt(r*r - z*z);
//     double beta = atan(tan_beta);
//     double q2 = alpha + beta; // 改为加号确保肘关节朝上

//     return Eigen::Vector3d(q1, q2, q3);
// }

Eigen::Vector3d inverseKinematics(double x, double y, double z) {

    // 底座旋转角
    double q1 = atan2(y, x);
    
    // 平面投影
    double r_proj = hypot(x, y);
    double r = hypot(r_proj, z);
    
    // 检查可达性
    if(r > (L1 + L2) || r < fabs(L1 - L2)) {
        return Eigen::Vector3d::Zero(); // 不可达
    }
    
    // 计算q3（肘关节朝上）
    double cos_q3 = (r*r - L1*L1 - L2*L2) / (2.0 * L1 * L2);
    cos_q3 = std::clamp(cos_q3, -1.0, 1.0);
    double q3 = -acos(cos_q3); // 负号确保肘关节朝上
    
    // 计算alpha（末端仰角）
    double alpha;
    if (r_proj < 1e-10) { // 处理投影接近零
        alpha = (z >= 0) ? M_PI/2 : -M_PI/2;
    } else {
        alpha = atan2(z, r_proj);
    }
    
    // 修正q2公式：使用正确的几何关系
    double numerator = L2 * sin(-q3);   // 注意：sin(-q3) > 0（因q3<0）
    double denominator = L1 + L2 * cos(q3);
    double gamma = atan2(numerator, denominator);
    double q2 = alpha + gamma; // 肘关节朝上构型
    
    return Eigen::Vector3d(q1, q2, q3);
}