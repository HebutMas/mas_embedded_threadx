/*
 * @Author: abbhhh 804433588@qq.com
 * @Date: 2026-09-15 20:36:42
 * @LastEditors: abbhhh 804433588@qq.com
 * @LastEditTime: 2026-09-27 15:48:59
 * @FilePath: \mas_embedded_threadx\apps\darts\single_board\gimbal_func\gimbal_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Description: 云台功能: yaw = ZDT 闭环步进 (CAN2 扩展帧, 位置模式 + INS 世界角闭环),
 *               pitch = 3508 (CAN1 id 2, 自己编码器 + LQR)
 */

#include "gimbal_func.h"

#include "module_ins.h"
#include "module_offline.h"
#include "motor_dji.h"
#include "user_lib.h"
#include "zdt_can.h"

#define LOG_TAG "app_gimbal"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static DJI_Motor_t *pitch_motor = NULL;
static const Ins_t *ins         = NULL;

void gimbal_init(void)
{
    ins = Module_INS_get();
    if (ins == NULL)
    {
        LOG_E("ins is null");
        return;
    }

    /* yaw: ZDT 闭环步进, CAN2 扩展帧直发 (CAN2 1M, 模块 CAN 速率设 1M, addr=1), 只发不收 */
    zdt_can_init(&hcan2, 1);

    /* pitch: 3508, 除 LQR 外与左右拉簧轴相同 */
    Motor_Init_Config_s pitch_config = {
        .offline_init_config =
            {
                .name       = "pitch_3508",
                .timeout_ms = 100,
                .beep_times = 2,
                .enable     = 1,
            },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan  = BSP_CAN_HANDLE1,
                .tx_id = 2,
            },
        .controller_init_config =
            {
                .lqr_init =
                    {
                        .K         = {GIMBAL_PITCH_LQR_K0, GIMBAL_PITCH_LQR_K1},
                        .state_dim = 2,
                    },
            },
        .setting_init_config =
            {
                .angle_feedback_source = 0,
                .speed_feedback_source = 0,
                .loop_type             = ANGLE_LOOP,
                .feedback_reverse_flag = 0,
                .algorithm_type        = CONTROL_LQR,
            },
        .motor_init_info = {.motor_type = M3508, .gear_ratio = 19, .max_torque = 6, .torque_constant = 0.016f},
    };
    pitch_motor = Motor_DJI_Init(&pitch_config);
    if (pitch_motor == NULL)
    {
        LOG_E("pitch_motor init failed");
        return;
    }

    LOG_I("darts gimbal init success");
}

void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd)
{
    static bool  active;
    static float yaw_last;
    (void)yaw_ecd; /* ZDT 没有反馈, darts 也没底盘跟随 */

    /* 遥控器/pitch 掉线时先失能，不等待回包处理。 */
    if (gimbal_cmd == NULL || gimbal_cmd->gimbal_mode == gimbal_zero_force || pitch_motor == NULL ||
        Module_Offline_get_device_status(pitch_motor->base.offline_dev) == STATE_OFFLINE)
    {
        Motor_Stop((Motor_Base *)pitch_motor);
        zdt_can_set_enable(false);
        active = false;
        return;
    }

    if (!active)
    {
        /* 手动复位后，以当前 pitch 和 yaw 脉冲起点继续，不追赶掉线前的目标。 */
        gimbal_cmd->pitch = -pitch_motor->base.measure.total_angle / (DEGREE_2_RAD * pitch_motor->base.info.gear_ratio);
        gimbal_cmd->yaw   = 0.0f;
        yaw_last          = 0.0f;
        active            = true;
    }

    /* 读 ZDT 模块回包 → Ozone 看 zdt_dbg_reply_status/count */
    zdt_can_poll_reply();

    /* gimbal_gyro_mode = 手动跟随 (gimbal_auto_mode 暂按这个走, TODO: 自动瞄准)。
     * yaw 角闭环暂时去掉 (TODO: 恢复 = 用 ins->YawTotalAngle_rad 算剩余误差):
     * 摇杆在 robot_func 里已按 GIMBAL_YAW_RATE_DPS 定速积分成目标角, 这里只把
     * 目标角增量换成相对脉冲发出去, 回中增量即 0 → 停。 */
    zdt_can_set_enable(true);

    float dyaw    = gimbal_cmd->yaw - yaw_last;
    yaw_last      = gimbal_cmd->yaw;
    float inc_max = GIMBAL_YAW_SPEED_RPM / 60.0f * GIMBAL_YAW_PULSES_PER_REV * 0.002f;
    float inc     = dyaw * GIMBAL_YAW_PULSES_PER_REV / 360.0f;
    VAL_LIMIT(inc, -inc_max, inc_max);
    if ((int32_t)inc != 0) zdt_can_send_position((int32_t)inc, (uint16_t)GIMBAL_YAW_SPEED_RPM);

    /* 负号 = pitch 方向取反 (电机正方向和云台抬头方向相反), 反馈标志位保持 0 */
    Motor_SetRef((Motor_Base *)pitch_motor, -gimbal_cmd->pitch * DEGREE_2_RAD * pitch_motor->base.info.gear_ratio);
    Motor_Start((Motor_Base *)pitch_motor);
}
