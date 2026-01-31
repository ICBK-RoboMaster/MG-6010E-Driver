/**
 * @file mg6010e.c
 * @brief 领控6010E电机驱动源文件
 * @author DW_yang (yang1003317819@163.com)
 * @date 2026-01-21
 */
#include "mg6010e.h"

static mg6010e_handle_t *mg6010e_handle_table[32] = {0}; // 电机句柄表

/**
 * @brief 注册领控6010E电机句柄到句柄表
 *
 * @param mg6010e_handle 电机句柄指针
 * @return uint8_t 错误码，0表示成功，4表示未初始化
 */
static uint8_t mg6010e_register_handle(mg6010e_handle_t *mg6010e_handle)
{
    if (mg6010e_handle == NULL || !mg6010e_handle->initialized)
    {
        return MG6010E_ERROR_NOT_INITIALIZED;
    }
    uint32_t motor_id = mg6010e_handle->config.motor_id;
    mg6010e_handle_table[motor_id - 1] = mg6010e_handle;
    return MG6010E_SUCCESS;
}

/**
 * @brief 初始化领控6010E电机配置
 *
 * @param mg6010e_config 电机配置结构体指针
 * @return uint8_t 错误码，0表示成功，1表示配置结构体指针为空，2表示CAN句柄为空，3表示电机ID无效
 */
uint8_t mg6010e_init(mg6010e_config_t *mg6010e_config)
{
    if (mg6010e_config == NULL)
    {
        return MG6010E_ERROR_CONFIG_NULL_PTR; // 配置结构体指针为空错误
    }
    if (mg6010e_config->can_handle == NULL)
    {
        return MG6010E_ERROR_CAN_NULL_PTR; // CAN句柄为空错误
    }
    if (mg6010e_config->motor_id < 1 || mg6010e_config->motor_id > 32)
    {
        return MG6010E_ERROR_INVALID_ID; // 电机ID无效错误
    }
    mg6010e_handle_t *mg6010e_handle = malloc(sizeof(mg6010e_handle_t));
    mg6010e_handle->config = *mg6010e_config;
    mg6010e_handle->status = (mg6010e_status_t){0};
    mg6010e_handle->encoder_data = (mg6010e_encoder_data_t){0};
    mg6010e_handle->control_params = (mg6010e_control_params_t){0};
    mg6010e_handle->initialized = 1;
    return mg6010e_register_handle(mg6010e_handle);
}

/**
 * @brief 通过电机ID获取领控6010E电机句柄
 *
 * @param motor_id 电机ID（1-32）
 * @return mg6010e_handle_t* 电机句柄指针，若ID无效则返回NULL
 */
static mg6010e_handle_t *mg6010e_get_handle_by_id(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > 32)
    {
        return NULL; // 电机ID无效
    }
    return mg6010e_handle_table[motor_id - 1];
}

/**
 * @brief 发送领控6010E电机命令
 *
 * @param mg6010e_handle 电机句柄指针
 * @param cmd_data 命令数据数组指针
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 依赖HAL库
 */
static uint8_t mg6010e_send_cmd(mg6010e_handle_t *mg6010e_handle, uint8_t *cmd_data)
{
    if (mg6010e_handle == NULL || !mg6010e_handle->initialized)
    {
        return MG6010E_ERROR_NOT_INITIALIZED; // 未初始化错误
    }

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = MG6010E_CAN_CMD_ID(mg6010e_handle->config.motor_id);
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    if (HAL_CAN_AddTxMessage(mg6010e_handle->config.can_handle, &tx_header, cmd_data, &mg6010e_handle->config.can_tx_mailbox) != HAL_OK)
    {
        return MG6010E_ERROR_SEND_FAILED; // 发送失败错误
    }
    return MG6010E_SUCCESS;
}

/**
 * @brief 发送领控6010E电机读取状态1命令
 *
 * @param motor_id 电机ID（1-32）
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 该命令读取当前电机的温度、电压和错误状态标志
 */
uint8_t mg6010e_read_status_1(uint8_t motor_id)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机清除错误标志命令
 *
 * @param motor_id 电机ID（1-32）
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 该命令清除当前电机的错误状态，电机收到后返回，电机状态没有恢复正常时，错误标志无法清除。
 */
uint8_t mg6010e_clean_error_flag(uint8_t motor_id)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0x9B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机读取状态2命令
 *
 * @param motor_id 电机ID（1-32）
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 该命令读取当前电机的温度、电机转矩电流（MF、MG）/电机输出功率（MS）、转速、编码器位置。
 */
uint8_t mg6010e_read_status_2(uint8_t motor_id)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机读取状态3命令
 *
 * @param motor_id 电机ID（1-32）
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 该命令读取当前电机的温度和 3 相电流数据
 */
uint8_t mg6010e_read_status_3(uint8_t motor_id)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0x9D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机关闭命令
 *
 * @param motor_id 电机ID（1-32）
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 将电机从开启状态（上电后默认状态）切换到关闭状态，清除电机转动圈数及之前接收的控制指令，
LED 由常亮转为慢闪。此时电机仍然可以回复控制命令，但不会执行动作。
 */
uint8_t mg6010e_disable(uint8_t motor_id)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机开启命令
 *
 * @param motor_id 电机ID（1-32）
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 将电机从关闭状态切换到开启状态，LED 由慢闪转为常亮。此时再发送控制指令即可控制电机动作。
 */
uint8_t mg6010e_run(uint8_t motor_id)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机停止命令
 *
 * @param motor_id 电机ID（1-32）
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 停止电机，但不清除电机运行状态。再次发送控制指令即可控制电机动作。
 */
uint8_t mg6010e_stop(uint8_t motor_id)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机抱闸器状态读取命令
 *
 * @param motor_id 电机ID（1-32）
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 读取当前抱闸器的状态。
 */
uint8_t mg6010e_break_status_read(uint8_t motor_id)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0x8C, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机抱闸器控制命令
 *
 * @param motor_id 电机ID（1-32）
 * @param engage 抱闸器状态，0：抱闸器断电，刹车启动，1：抱闸器通电，刹车释放
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 控制抱闸器的开合。
 */
uint8_t mg6010e_break_control(uint8_t motor_id, uint8_t engage)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0x8C, engage ? 0x01 : 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机转矩电流闭环控制命令
 *
 * @param motor_id 电机ID（1-32）
 * @param iqControl 转矩电流，数值范围-2048~ 2048，对应 MG 电机实际转矩电流范围-33A~33A
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 主机发送该命令以控制电机的转矩电流输出，母线电流和电机的实际扭矩因不同电机而异。
 * 该命令中的控制值 iqControl 不受上位机中的 Max Torque Current 值限制。
 */
uint8_t mg6010e_iq_control(uint8_t motor_id, int16_t iqControl)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0xA1, 0x00, 0x00, 0x00, *(uint8_t *)(&iqControl), *((uint8_t *)(&iqControl) + 1), 0x00, 0x00};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机速度闭环控制命令
 *
 * @param motor_id 电机ID（1-32）
 * @param iqControl 转矩电流，数值范围-2048~ 2048，对应 MG 电机实际转矩电流范围-33A~33A
 * @param speedControl 速度控制值，对应实际转速为 0.01dps/LSB
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 主机发送该命令以控制电机的速度，同时带有力矩限制。母线电流和电机的实际扭矩因不同电机而异。
 * 该命令下电机的 speedControl 由上位机中的 Max Speed 值限制。
 * 该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 */
uint8_t mg6010e_speed_control(uint8_t motor_id, int16_t iqControl, int32_t speedControl)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0xA2, 0x00, *(uint8_t *)(&iqControl), *((uint8_t *)(&iqControl) + 1), *(uint8_t *)(&speedControl), *((uint8_t *)(&speedControl) + 1), *((uint8_t *)(&speedControl) + 2), *((uint8_t *)(&speedControl) + 3)};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机多圈角度位置闭环控制命令
 *
 * @param motor_id 电机ID（1-32）
 * @param angleControl 位置控制值，对应实际位置为 0.01deg/LSB，即 36000 代表 360°
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 主机发送该命令以控制电机的位置（多圈角度）。电机转动方向由目标位置和当前位置的差值决定。
 * 1. 该命令下的控制值 angleControl 受上位机中的 Max Angle 值限制。
 * 2. 该命令下电机的最大速度由上位机中的 Max Speed 值限制。
 * 3. 该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 * 4. 该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque Current 值限制。
 */
uint8_t mg6010e_angle_control(uint8_t motor_id, int32_t angleControl)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0xA3, 0x00, 0x00, 0x00, *(uint8_t *)(&angleControl), *((uint8_t *)(&angleControl) + 1), *((uint8_t *)(&angleControl) + 2), *((uint8_t *)(&angleControl) + 3)};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机多圈角度位置闭环控制命令2
 *
 * @param motor_id 电机ID（1-32）
 * @param angleControl 位置控制值，对应实际位置为 0.01deg/LSB，即 36000 代表 360°
 * @param maxSpeed 最大速度控制值，对应实际转速 1dps/LSB，即 360 代表 360dps。
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 主机发送该命令以控制电机的位置（多圈角度）。电机转动方向由目标位置和当前位置的差值决定。携带最大速度参数。
 * 1. 该命令下的控制值 angleControl 受上位机中的 Max Angle 值限制。
 * 2. 该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 * 3. 该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque Current 值限制。
 */
uint8_t mg6010e_angle_control_2(uint8_t motor_id, int32_t angleControl, uint16_t maxSpeed)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0xA4, 0x00, *(uint8_t *)(&maxSpeed), *((uint8_t *)(&maxSpeed) + 1), *(uint8_t *)(&angleControl), *((uint8_t *)(&angleControl) + 1), *((uint8_t *)(&angleControl) + 2), *((uint8_t *)(&angleControl) + 3)};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机单圈角度位置闭环控制命令
 *
 * @param motor_id 电机ID（1-32）
 * @param angleControl 位置控制值，范围（0 ~ 36000）对应实际位置为 0.01deg/LSB，即 36000 代表 360°
 * @param spinDirection 旋转方向，0表示顺时针，1表示逆时针
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 主机发送该命令以控制电机的位置（单圈角度）。
 * 1. 该命令下电机的最大速度由上位机中的 Max Speed 值限制。
 * 2. 该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 * 3. 该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque Current 值限制。
 */
uint8_t mg6010e_single_angle_control(uint8_t motor_id, uint32_t angleControl, uint8_t spinDirection)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0xA5, spinDirection, 0x00, 0x00, *(uint8_t *)(&angleControl), *((uint8_t *)(&angleControl) + 1), *((uint8_t *)(&angleControl) + 2), *((uint8_t *)(&angleControl) + 3)};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机单圈角度位置闭环控制命令2
 *
 * @param motor_id 电机ID（1-32）
 * @param angleControl 位置控制值，对应实际位置为 0.01deg/LSB，即 36000 代表 360°
 * @param maxSpeed 最大速度控制值，对应实际转速 1dps/LSB，即 360 代表 360dps。
 * @param spinDirection 旋转方向，0表示顺时针，1表示逆时针
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 主机发送该命令以控制电机的位置（单圈角度）。携带最大速度参数。
 * 1. 该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 * 2. 该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque Current 值限制；
 */
uint8_t mg6010e_single_angle_control_2(uint8_t motor_id, int32_t angleControl, uint16_t maxSpeed, uint8_t spinDirection)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0xA6, spinDirection, *(uint8_t *)(&maxSpeed), *((uint8_t *)(&maxSpeed) + 1), *(uint8_t *)(&angleControl), *((uint8_t *)(&angleControl) + 1), *((uint8_t *)(&angleControl) + 2), *((uint8_t *)(&angleControl) + 3)};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机多圈角度位置闭环控制命令
 *
 * @param motor_id 电机ID（1-32）
 * @param angleIncrement 增量值，对应实际位置为 0.01deg/LSB，即 36000 代表 360°
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 主机发送该命令以控制电机的位置增量。
 * 1. 该命令下电机的最大速度由上位机中的 Max Speed 值限制。
 * 2. 该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 * 3. 该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque Current 值限制。
 */
uint8_t mg6010e_angle_increment_control(uint8_t motor_id, int32_t angleIncrement)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0xA7, 0x00, 0x00, 0x00, *(uint8_t *)(&angleIncrement), *((uint8_t *)(&angleIncrement) + 1), *((uint8_t *)(&angleIncrement) + 2), *((uint8_t *)(&angleIncrement) + 3)};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机多圈角度位置闭环控制命令2
 *
 * @param motor_id 电机ID（1-32）
 * @param angleIncrement 增量值，对应实际位置为 0.01deg/LSB，即 36000 代表 360°
 * @param maxSpeed 最大速度控制值，对应实际转速 1dps/LSB，即 360 代表 360dps。
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 主机发送该命令以控制电机的位置增量。携带最大速度参数。
 * 1. 该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 * 2. 该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque Current 值限制。
 */
uint8_t mg6010e_angle_increment_control_2(uint8_t motor_id, int32_t angleIncrement, uint16_t maxSpeed)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0xA8, 0x00, *(uint8_t *)(&maxSpeed), *((uint8_t *)(&maxSpeed) + 1), *(uint8_t *)(&angleIncrement), *((uint8_t *)(&angleIncrement) + 1), *((uint8_t *)(&angleIncrement) + 2), *((uint8_t *)(&angleIncrement) + 3)};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机控制参数读取命令
 *
 * @param motor_id 电机ID（1-32）
 * @param controlParamID 控制参数ID
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 主机发送该命令读取当前电机的控制参数，读取的参数由序号 controlParamID 确定，见电机控制参数表
 */
uint8_t mg6010e_read_control_param(uint8_t motor_id, uint8_t controlParamID)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0xC0, controlParamID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机控制参数写入命令
 *
 * @param motor_id 电机ID（1-32）
 * @param controlParamID 控制参数ID
 * @param paramData 控制参数数据指针，长度根据具体参数而定，最大6字节
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 主机发送该命令写入控制参数到 RAM 中，即时生效，断电后失效。写入的参数和序号 controlParamID 见电机控制参数表
 */
uint8_t mg6010e_write_control_param(uint8_t motor_id, uint8_t controlParamID, uint8_t *paramData)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0xC1, controlParamID, paramData[0], paramData[1], paramData[2], paramData[3], paramData[4], paramData[5]};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机读取编码器数据命令
 *
 * @param motor_id 电机ID（1-32）
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 主机发送该命令以读取编码器的当前位置。
 */
uint8_t mg6010e_read_encoder(uint8_t motor_id)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机写入编码器零点命令
 *
 * @param motor_id 电机ID（1-32）
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 设置电机当前位置的编码器原始值作为电机上电后的初始零点
 * 1．该命令需要重新上电后才能生效
 * 2．该命令会将零点写入驱动的 ROM，多次写入将会影响芯片寿命，不建议频繁使用
 */
uint8_t mg6010e_write_encoder_zero_point(uint8_t motor_id)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机读取多圈角度命令
 *
 * @param motor_id 电机ID（1-32）
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 主机发送该命令以读取当前电机的多圈绝对角度值。
 */
uint8_t mg6010e_read_angle(uint8_t motor_id)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机读取单圈角度命令
 *
 * @param motor_id 电机ID（1-32）
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 主机发送该命令以读取当前电机的单圈绝对角度值。
 */
uint8_t mg6010e_read_single_angle(uint8_t motor_id)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 发送领控6010E电机设置当前位置为任意角度（多圈角度）
 *
 * @param motor_id 电机ID（1-32）
 * @param motorAngle 角度值，数据单位 0.01°/LSB。
 * @return uint8_t 错误码，0表示成功，4表示未初始化，5表示发送失败
 * @note 主机发送该命令以设置电机的当前位置作为任意角度（写入 RAM，电机下电后丢失数据）。
 */
uint8_t mg6010e_set_angle(uint8_t motor_id, int32_t motorAngle)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);

    uint8_t cmd_data[8] = {0x95, 0x00, 0x00, 0x00, *(uint8_t *)(&motorAngle), *((uint8_t *)(&motorAngle) + 1), *((uint8_t *)(&motorAngle) + 2), *((uint8_t *)(&motorAngle) + 3)};
    return mg6010e_send_cmd(mg6010e_handle, cmd_data);
}

/**
 * @brief 获取领控6010E电机状态数据
 *
 * @param motor_id 电机ID（1-32）
 * @param status 电机状态数据指针
 * @return uint8_t 错误码，0表示成功，4表示未初始化
 */
uint8_t mg6010e_get_motor_status(uint8_t motor_id, mg6010e_status_t *status)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);
    if (mg6010e_handle == NULL)
    {
        return MG6010E_ERROR_NOT_INITIALIZED;
    }
    memcpy(status, &mg6010e_handle->status, sizeof(mg6010e_status_t));
    return MG6010E_SUCCESS;
}

/**
 * @brief 获取领控6010E电机编码器参数数据
 *
 * @param motor_id 电机ID（1-32）
 * @param encoder_data 电机编码器参数数据指针
 * @return uint8_t 错误码，0表示成功，4表示未初始化
 */
uint8_t mg6010e_get_motor_encoder_data(uint8_t motor_id, mg6010e_encoder_data_t *encoder_data)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);
    if (mg6010e_handle == NULL)
    {
        return MG6010E_ERROR_NOT_INITIALIZED;
    }
    memcpy(encoder_data, &mg6010e_handle->encoder_data, sizeof(mg6010e_encoder_data_t));
    return MG6010E_SUCCESS;
}

/**
 * @brief 获取领控6010E电机控制参数数据
 *
 * @param motor_id 电机ID（1-32）
 * @param control_params 电机控制参数数据指针
 * @return uint8_t 错误码，0表示成功，4表示未初始化
 */
uint8_t mg6010e_get_motor_control_params(uint8_t motor_id, mg6010e_control_params_t *control_params)
{
    mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);
    if (mg6010e_handle == NULL)
    {
        return MG6010E_ERROR_NOT_INITIALIZED;
    }
    memcpy(control_params, &mg6010e_handle->control_params, sizeof(mg6010e_control_params_t));
    return MG6010E_SUCCESS;
}

/**
 * @brief 领控6010E电机CAN接收回调钩子函数
 * @param rx_header CAN接收报文头指针
 * @param rx_data CAN接收数据指针
 * @note 您需要自行处理CAN接收与数据，然后将接受到的数据传入本函数，请将该函数注册在CAN总线回调函数中。依赖HAL库。
 */
void mg6010e_can_rx_callback_hook(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    if (rx_header->StdId > MG6010E_CAN_FEEDBACK_BASE_ID && rx_header->StdId <= MG6010E_CAN_FEEDBACK_BASE_ID + 32)
    {
        uint8_t motor_id = MG6010E_CAN_GET_MOTOR_ID(rx_header->StdId);
        mg6010e_handle_t *mg6010e_handle = mg6010e_get_handle_by_id(motor_id);
        if (mg6010e_handle != NULL && mg6010e_handle->initialized)
        {
            // 处理接收到的数据
            switch (rx_data[0])
            {
            case 0x9A: // 读取状态1反馈
                mg6010e_handle->status.temperature = rx_data[1];
                mg6010e_handle->status.voltage = (uint16_t)rx_data[2] | ((uint16_t)rx_data[3] << 8);
                mg6010e_handle->status.current = (int16_t)(((uint16_t)rx_data[4]) | ((uint16_t)rx_data[5] << 8));
                mg6010e_handle->status.motorState = rx_data[6];
                mg6010e_handle->status.errorState = rx_data[7];
                break;
            case 0x9C: // 读取状态2反馈
            case 0xA1:
            case 0xA2:
            case 0xA3:
            case 0xA4:
            case 0xA5:
            case 0xA6:
            case 0xA7:
            case 0xA8:
                mg6010e_handle->status.temperature = rx_data[1];
                mg6010e_handle->status.iqActual = (int16_t)(((uint16_t)rx_data[2]) | ((uint16_t)rx_data[3] << 8));
                mg6010e_handle->status.speed = (int16_t)(((uint16_t)rx_data[4]) | ((uint16_t)rx_data[5] << 8));
                mg6010e_handle->status.encoder = (uint16_t)rx_data[6] | ((uint16_t)rx_data[7] << 8);
                break;
            case 0x9D: // 读取状态3反馈
                mg6010e_handle->status.temperature = rx_data[1];
                mg6010e_handle->status.iA = (int16_t)(((uint16_t)rx_data[2]) | ((uint16_t)rx_data[3] << 8));
                mg6010e_handle->status.iB = (int16_t)(((uint16_t)rx_data[4]) | ((uint16_t)rx_data[5] << 8));
                mg6010e_handle->status.iC = (int16_t)(((uint16_t)rx_data[6]) | ((uint16_t)rx_data[7] << 8));
                break;
            case 0x8C: // 抱闸器状态反馈
                mg6010e_handle->status.brakeStatus = rx_data[1];
                break;
            case 0xC0: // 读取控制参数反馈
            case 0xC1:
                switch (rx_data[1])
                {
                case 0x0A: // 角度环PID参数
                    mg6010e_handle->control_params.anglekp = (int16_t)(((uint16_t)rx_data[2]) | ((uint16_t)rx_data[3] << 8));
                    mg6010e_handle->control_params.angleki = (int16_t)(((uint16_t)rx_data[4]) | ((uint16_t)rx_data[5] << 8));
                    mg6010e_handle->control_params.anglekd = (int16_t)(((uint16_t)rx_data[6]) | ((uint16_t)rx_data[7] << 8));
                    break;
                case 0x0B: // 速度环PID参数
                    mg6010e_handle->control_params.speedkp = (int16_t)(((uint16_t)rx_data[2]) | ((uint16_t)rx_data[3] << 8));
                    mg6010e_handle->control_params.speedki = (int16_t)(((uint16_t)rx_data[4]) | ((uint16_t)rx_data[5] << 8));
                    mg6010e_handle->control_params.speedkd = (int16_t)(((uint16_t)rx_data[6]) | ((uint16_t)rx_data[7] << 8));
                    break;
                case 0x0C: // 电流环PID参数
                    mg6010e_handle->control_params.currentkp = (int16_t)(((uint16_t)rx_data[2]) | ((uint16_t)rx_data[3] << 8));
                    mg6010e_handle->control_params.currentki = (int16_t)(((uint16_t)rx_data[4]) | ((uint16_t)rx_data[5] << 8));
                    mg6010e_handle->control_params.currentkd = (int16_t)(((uint16_t)rx_data[6]) | ((uint16_t)rx_data[7] << 8));
                    break;
                case 0x1E: // 转矩电流限制参数
                    mg6010e_handle->control_params.inputTorqueLimit = (int16_t)(((uint16_t)rx_data[4]) | ((uint16_t)rx_data[5] << 8));
                    break;
                case 0x20: // 最大速度参数
                    mg6010e_handle->control_params.inputSpeedLimit = (int32_t)(((uint32_t)rx_data[4]) | ((uint32_t)rx_data[5] << 8) | ((uint32_t)rx_data[6] << 16) | ((uint32_t)rx_data[7] << 24));
                    break;
                case 0x22: // 角度限制参数
                    mg6010e_handle->control_params.inputAngleLimit = (int32_t)(((uint32_t)rx_data[4]) | ((uint32_t)rx_data[5] << 8) | ((uint32_t)rx_data[6] << 16) | ((uint32_t)rx_data[7] << 24));
                    break;
                case 0x24: // 电流斜率参数
                    mg6010e_handle->control_params.inputCurrentRamp = (int32_t)(((uint32_t)rx_data[4]) | ((uint32_t)rx_data[5] << 8) | ((uint32_t)rx_data[6] << 16) | ((uint32_t)rx_data[7] << 24));
                    break;
                case 0x26: // 速度斜率参数
                    mg6010e_handle->control_params.inputSpeedRamp = (int32_t)(((uint32_t)rx_data[4]) | ((uint32_t)rx_data[5] << 8) | ((uint32_t)rx_data[6] << 16) | ((uint32_t)rx_data[7] << 24));
                    break;
                default:
                    break;
                }
                break;
            case 0x90: // 读取编码器反馈
                mg6010e_handle->encoder_data.encoder = (uint16_t)rx_data[2] | ((uint16_t)rx_data[3] << 8);
                mg6010e_handle->encoder_data.encoderRaw = (uint16_t)rx_data[4] | ((uint16_t)rx_data[5] << 8);
                mg6010e_handle->encoder_data.encoderOffset = (uint16_t)rx_data[6] | ((uint16_t)rx_data[7] << 8);
                break;
            case 0x19:
                mg6010e_handle->encoder_data.encoderOffset = (uint16_t)rx_data[6] | ((uint16_t)rx_data[7] << 8);
                break;
            case 0x92: // 读取多圈角度反馈
                mg6010e_handle->status.angle = (int64_t)(((uint64_t)rx_data[1]) | ((uint64_t)rx_data[2] << 8) | ((uint64_t)rx_data[3] << 16) | ((uint64_t)rx_data[4] << 24) | ((uint64_t)rx_data[5] << 32) | ((uint64_t)rx_data[6] << 40) | ((uint64_t)rx_data[7] << 48));
                break;
            case 0x94: // 读取单圈角度反馈
                mg6010e_handle->status.single_angle = (int32_t)(((uint32_t)rx_data[4]) | ((uint32_t)rx_data[5] << 8) | ((uint32_t)rx_data[6] << 16) | ((uint32_t)rx_data[7] << 24));
                break;
            case 0x95: // 设置当前位置反馈
                mg6010e_handle->status.angle = (int32_t)(((uint32_t)rx_data[4]) | ((uint32_t)rx_data[5] << 8) | ((uint32_t)rx_data[6] << 16) | ((uint32_t)rx_data[7] << 24));
                break;
            default:
                break;
            }
        }
    }
}