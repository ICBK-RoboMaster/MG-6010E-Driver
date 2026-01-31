#ifndef __MG6010E_H__
#define __MG6010E_H__

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stm32f4xx_hal.h> // 依赖HAL库，请根据实际情况修改为对应的HAL库头文件路径

#define MG6010E_SUCCESS 0
#define MG6010E_ERROR_CONFIG_NULL_PTR 1
#define MG6010E_ERROR_CAN_NULL_PTR 2
#define MG6010E_ERROR_INVALID_ID 3
#define MG6010E_ERROR_NOT_INITIALIZED 4
#define MG6010E_ERROR_SEND_FAILED 5
#define MG6010E_CAN_CMD_BASE_ID 0x140
#define MG6010E_CAN_CMD_ID(motor_id) (MG6010E_CAN_CMD_BASE_ID + motor_id)
#define MG6010E_CAN_FEEDBACK_BASE_ID 0x140 // 手册中是0x180，但实际测试为0x140
#define MG6010E_CAN_FEEDBACK_ID(motor_id) (MG6010E_CAN_FEEDBACK_BASE_ID + motor_id)
#define MG6010E_CAN_GET_MOTOR_ID(feedback_id) ((feedback_id) - MG6010E_CAN_FEEDBACK_BASE_ID)

// 领控6010E电机配置结构体，依赖HAL库
typedef struct mg6010e_config
{
    CAN_HandleTypeDef *can_handle; // CAN句柄
    uint32_t can_tx_mailbox;       // CAN发送邮箱
    uint32_t motor_id;             // 电机ID(1-32)
} mg6010e_config_t;

// 领控6010E电机状态结构体
typedef struct mg6010e_status
{
    int8_t temperature;    // 电机温度，单位 1℃/LSB
    int16_t voltage;       // 电机母线电压，单位：0.01V/LSB
    int16_t current;       // 电机母线电流，单位：0.01A/LSB
    uint8_t motorState;    // 电机状态，0x00 电机处于开启状态；0x10 电机处于关闭状态。
    uint8_t errorState;    // 电机错误标志位
    int16_t iqActual;      // 实际转矩电流，单位：(66/4096 A ≈ 0.01622 A) / LSB
    int16_t speed;         // 电机实际速度，单位：1dps/LSB
    uint16_t encoder;      // 电机编码器值，14bit编码器的数值范围0-16383，15bit编码器的数值范围0-32767，16bit编码器的数值范围0-65535
    int16_t iA;            // 相A电流，单位：(66/4096 A ≈ 0.01622 A)/LSB
    int16_t iB;            // 相B电流，单位：(66/4096 A ≈ 0.01622 A)/LSB
    int16_t iC;            // 相C电流，单位：(66/4096 A ≈ 0.01622 A)/LSB
    uint8_t brakeStatus;   // 抱闸器状态，0：抱闸器断电，刹车启动，1：抱闸器通电，刹车释放
    int64_t angle;         // 电机多圈角度，单位：0.01°/LSB
    uint32_t single_angle; // 电机单圈角度，单位：0.01°/LSB
} mg6010e_status_t;

// 领控6010E电机控制参数结构体
typedef struct mg6010e_control_params
{
    uint16_t anglekp;         // 角度环比例系数
    uint16_t angleki;         // 角度环积分系数
    uint16_t anglekd;         // 角度环微分系数
    uint16_t speedkp;         // 速度环比例系数
    uint16_t speedki;         // 速度环积分系数
    uint16_t speedkd;         // 速度环微分系数
    uint16_t currentkp;       // 电流环比例系数
    uint16_t currentki;       // 电流环积分系数
    uint16_t currentkd;       // 电流环微分系数
    int16_t inputTorqueLimit; // 最大力矩电流
    int32_t inputSpeedLimit;  // 最大速度限制
    int32_t inputAngleLimit;  // 角度限制
    int32_t inputCurrentRamp; // 电流斜率
    int32_t inputSpeedRamp;   // 速度斜率
} mg6010e_control_params_t;

// 领控6010E电机编码器数据结构体
typedef struct mg6010e_encoder_data
{
    uint16_t encoder;       // 编码器值
    uint16_t encoderRaw;    // 编码器原始位置
    uint16_t encoderOffset; // 编码器零偏
} mg6010e_encoder_data_t;

// 领控6010E电机句柄结构体
typedef struct mg6010e_handle
{
    mg6010e_config_t config;                 // 电机配置
    mg6010e_status_t status;                 // 电机状态
    mg6010e_encoder_data_t encoder_data;     // 电机编码器数据
    mg6010e_control_params_t control_params; // 电机控制参数
    uint8_t initialized;                     // 初始化标志
} mg6010e_handle_t;

uint8_t mg6010e_init(mg6010e_config_t *mg6010e_config);
uint8_t mg6010e_read_status_1(uint8_t motor_id);
uint8_t mg6010e_read_status_2(uint8_t motor_id);
uint8_t mg6010e_read_status_3(uint8_t motor_id);
uint8_t mg6010e_clean_error_flag(uint8_t motor_id);
uint8_t mg6010e_iq_control(uint8_t motor_id, int16_t iqControl);
uint8_t mg6010e_speed_control(uint8_t motor_id, int16_t iqControl, int32_t speedControl);
uint8_t mg6010e_angle_control(uint8_t motor_id, int32_t angleControl);
uint8_t mg6010e_angle_control_2(uint8_t motor_id, int32_t angleControl, uint16_t maxSpeed);
uint8_t mg6010e_single_angle_control(uint8_t motor_id, uint32_t angleControl, uint8_t spinDirection);
uint8_t mg6010e_single_angle_control_2(uint8_t motor_id, int32_t angleControl, uint16_t maxSpeed, uint8_t spinDirection);
uint8_t mg6010e_angle_increment_control(uint8_t motor_id, int32_t angleIncrement);
uint8_t mg6010e_angle_increment_control_2(uint8_t motor_id, int32_t angleIncrement, uint16_t maxSpeed);
uint8_t mg6010e_read_control_param(uint8_t motor_id, uint8_t controlParamID);
uint8_t mg6010e_write_control_param(uint8_t motor_id, uint8_t controlParamID, uint8_t *paramData);
uint8_t mg6010e_disable(uint8_t motor_id);
uint8_t mg6010e_run(uint8_t motor_id);
uint8_t mg6010e_stop(uint8_t motor_id);
uint8_t mg6010e_break_status_read(uint8_t motor_id);
uint8_t mg6010e_break_control(uint8_t motor_id, uint8_t engage);
uint8_t mg6010e_read_encoder(uint8_t motor_id);
uint8_t mg6010e_write_encoder_zero_point(uint8_t motor_id);
uint8_t mg6010e_read_angle(uint8_t motor_id);
uint8_t mg6010e_read_single_angle(uint8_t motor_id);
uint8_t mg6010e_set_angle(uint8_t motor_id, int32_t motorAngle);
uint8_t mg6010e_get_motor_status(uint8_t motor_id, mg6010e_status_t *status);
uint8_t mg6010e_get_motor_control_params(uint8_t motor_id, mg6010e_control_params_t *control_params);
uint8_t mg6010e_get_motor_encoder_data(uint8_t motor_id, mg6010e_encoder_data_t *encoder_data);
void mg6010e_can_rx_callback_hook(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);

#endif /* __MG6010E_H__ */