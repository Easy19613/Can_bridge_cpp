#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <utility>
#include <vector>
#include <unordered_map>
#include <array>
#include <variant>
#include <cstdint>
#include <cmath>

    enum Control_Mode
    {
        MIT_MODE=1,
        POS_VEL_MODE=2,
        VEL_MODE=3,
        POS_FORCE_MODE=4,
    };

typedef struct
    {
        float Q_MAX;
        float DQ_MAX;
        float TAU_MAX;
    }Limit_param;

    //电机PMAX DQMAX TAUMAX参数
    Limit_param limit_param_cmd={12.5, 8, 28}; // DM4340
    Limit_param limit_param_receive={12.5, 8, 28}; // DM4340
    
std::array<uint8_t, 8>control_mit(float kp, float kd, float q, float dq, float tau)
        {
            // 位置、速度和扭矩采用线性映射的关系将浮点型数据转换成有符号的定点数据
            static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
                float span = xmax - xmin;
                float data_norm = (x - xmin) / span;
                uint16_t data_uint = data_norm * ((1 << bits) - 1);
                return data_uint;
            };
            uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
            uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
            uint16_t q_uint = float_to_uint(q, -limit_param_cmd.Q_MAX, limit_param_cmd.Q_MAX, 16);
            uint16_t dq_uint = float_to_uint(dq, -limit_param_cmd.DQ_MAX,limit_param_cmd.DQ_MAX, 12);
            uint16_t tau_uint = float_to_uint(tau, -limit_param_cmd.TAU_MAX, limit_param_cmd.TAU_MAX, 12);

            std::array<uint8_t, 8> data_buf{};
            data_buf[0] = (q_uint >> 8) & 0xff;
            data_buf[1] = q_uint & 0xff;
            data_buf[2] = dq_uint >> 4;
            data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
            data_buf[4] = kp_uint & 0xff;
            data_buf[5] = kd_uint >> 4;
            data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
            data_buf[7] = tau_uint & 0xff;

            return data_buf;
        }

std::array<uint8_t, 8> control_pos_vel(float pos,float vel)
    {
        std::array<uint8_t, 8> data_buf={0};
        memcpy(data_buf.data(), &pos, sizeof(float));
        memcpy(data_buf.data() + 4, &vel, sizeof(float));
        return data_buf;
    }

    float* dm4340_receive(unsigned char* data)
    {
            static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
                float span = xmax - xmin;
                float data_norm = float(x) / ((1 << bits) - 1);
                float data = data_norm * span + xmin;
                return data;
            };
            uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
            uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
            uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];
            float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);
            float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
            float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
            float *message=new float[3]{receive_q,receive_dq,receive_tau};
            return message;
    }

// std::array<uint8_t, 8> write_motor_param(Motor &DM_Motor,uint8_t RID,const uint8_t data[4])
//         {
//             uint32_t id = DM_Motor.GetSlaveId();
//             uint8_t can_low = id & 0xff;
//             uint8_t can_high = (id >> 8) & 0xff;
//             std::array<uint8_t, 8> data_buf{can_low, can_high, 0x55, RID, 0x00, 0x00, 0x00, 0x00};
//             data_buf[4] = data[0];
//             data_buf[5] = data[1];
//             data_buf[6] = data[2];
//             data_buf[7] = data[3];
//             send_data.modify(0x7FF, data_buf.data());
//             serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
//         }


    // bool switchControlMode(Motor &DM_Motor,Control_Mode mode)
    //     {
    //         uint8_t write_data[4]={(uint8_t)mode, 0x00, 0x00, 0x00};
    //         uint8_t RID = 10;
    //         write_motor_param(DM_Motor,RID,write_data);
    //         if (motors.find(DM_Motor.GetSlaveId()) == motors.end())
    //         {
    //             return false;
    //         }
    //         for(uint8_t i =0;i<max_retries;i++)
    //         {
    //             usleep(retry_interval);
    //             receive_param();
    //             if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID))
    //             {
    //                 return motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID) == mode;
    //             }
    //         }
    //         return false;
    //     }