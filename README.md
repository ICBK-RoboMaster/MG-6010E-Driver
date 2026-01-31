# MG-6010E 电机驱动

## 简介

项目基于HAL库开发，封装了电机CAN手册中的所有操作

## 注意

1. 使用本驱动时应配合手册（已包含在仓库中）
2. 在实际测试中，手册中的接收`基CAN ID`有误，手册中是`0x180`，但实际测试为`0x140`，如果你没有遇到这个情况，请修改`mg6010e.h`中的`MG6010E_CAN_FEEDBACK_BASE_ID`

## 用法

1. 在`mg6010.h`中，include你所使用的hal库头文件
2. 如果你使用的不是can而是canfd，请自行修改相关代码，搜索“依赖HAL库“可找到与HAL库相关的代码
3. 添加`mg6010.h`与`mg6010.c`到你的项目文件中
4. 将`mg6010e_can_rx_callback_hook`放入CAN的接收回调函数中

#### 初始化电机

```c
#include "mg6010.h"

mg6010e_config_t config; // 电机配置结构体
config.can_handle = &hcan1; // 电机所在can总线的句柄
config.can_tx_mailbox = tx_mailbox; // 发送邮箱
config.motor_id = 1; // 电机ID

mg6010e_init(config); // 初始化电机
```

#### 发送命令

详见`mg6010e.c`，所有方法均有详细的使用说明

#### 接收反馈数据

请确保您已经正确的将`mg6010e_can_rx_callback_hook`注册到can接收回调中，如：
```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    mg6010e_can_rx_callback_hook(&rx_header, rx_data); // 注册在CAN回调中
}
```

然后：
```c
mg6010e_status_t motor_status; // 创建电机状态结构体
mg6010e_control_params_t motor_control_params; // 创建电机控制参数结构体
mg6010e_encoder_data_t motor_encoder_data; // 创建电机编码器数据结构体

// 在此处发送read类命令通知电机发送反馈数据，如：
mg6010e_read_status_2(motor_id);
// 值得注意：有一些控制类命令本身会返回一些状态数据，所以不需要重复调用一些read命令，具体请参见手册

mg6010e_get_motor_status(motor_id, &motor_status); // 读取电机状态
mg6010e_get_motor_control_params(motor_id, &motor_control_params); // 读取电机控制参数
mg6010e_get_motor_encoder_data(motor_id, &motor_encoder_data); // 读取编码器数据信息

printf(motor_status.motorState);
```