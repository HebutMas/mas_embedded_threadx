/*
 * @Author: abbhhh 804433588@qq.com
 * @Date: 2026-09-15 20:36:42
 * @LastEditors: abbhhh 804433588@qq.com
 * @LastEditTime: 2026-09-25 10:52:09
 * @FilePath: \mas_embedded_threadx\apps\darts\single_board\shoot_func\shoot_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "shoot_func.h"

#define LOG_TAG "app_shoot_func"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"
#include "module_offline.h"
#include "motor_dji.h"
#include "motor_damiao.h"
#include "motor_servo.h"
#include "shoot_mode.h"
#include "bsp_dwt.h"
#include "tim.h"
#include "user_lib.h"

#include <math.h>

static DJI_Motor_t   *left_friction_motor;
static DJI_Motor_t   *right_friction_motor;
static DM_Motor_t    *reload_motor;
static Servo_Motor_t *trigger = NULL;
static bool           trigger_pwm_enabled;

/* 扳机 PWM 启停由应用负责，复用原舵机模块的角度到脉宽更新。 */
static bool trigger_set_enable(bool enable)
{
    if (trigger == NULL) return false;
    Motor_Base *base = (Motor_Base *)trigger;
    if (!enable)
    {
        Motor_Stop(base);
        if (BSP_PWM_Stop(trigger->pwm_dev) != 0) return false;
        trigger_pwm_enabled = false;
        return true;
    }

    Motor_Start(base);
    if (trigger_pwm_enabled) return true;
    base->Apply(base); /* PWM 关闭期间先写入当前目标，避免输出旧脉宽。 */
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    htim8.Instance->EGR = TIM_EVENTSOURCE_UPDATE; /* 将预装载脉宽立即生效。 */
    if (BSP_PWM_Start(trigger->pwm_dev, NULL, 0) != 0)
    {
        Motor_Stop(base);
        return false;
    }
    trigger_pwm_enabled = true;
    return true;
}

static float    home_l, home_r;         /* 每发开始时记录的待机位 (转子侧 rad) */
static float    out_l, out_r;           /* 本发首次下拉目标: 第1发去工作位，第2~4发去独立下拉位 (转子侧 rad) */
static float    work_l, work_r;         /* 工作位 = home ± FRICTION_TRAVEL_MM 行程 (转子侧 rad) */
static float    reload_l, reload_r;     /* reload 停位 = home ± FRICTION_RELOAD_MM 行程 (转子侧 rad) */
static float    target_l, target_r;     /* 当前段最终目标，LQR 参考位置逐周期靠近它 (转子侧 rad) */
static float    loader_target_rad;      /* 拨盘目标角 (电机自身零点 + 偏置, rad), 每转 +ONE_BULLET_DELTA_ANGLE */
static uint64_t trigger_ready_start_us; /* 工作位上膛等待的开始时刻 */
static uint16_t startup_cnt;            /* 上电先等过 offline 检测窗口, 再要求两轴有反馈 */
static bool     inited;                 /* 上电零点已记 */
static uint8_t  shot_cnt;               /* 一轮里已打完的发数 (拨杆触发, 4 发一轮循环) */
static uint8_t  trigger_last = 1;   /* 上线后先离开下档，再接受下拨边沿 */

typedef enum
{
    axis_idle = 0,    /* 待机位泄力, 等 CH5 下拨边沿 */
    axis_out,         /* 出发 → 本发首次下拉目标 */
    axis_out_done,    /* 首次下拉到位: 第1发上膛等待; 第2~4发下发拨盘第1转 */
    axis_turn_wait,   /* 停在独立下拉位等拨盘第1转 (转完才走小回程) */
    axis_back_reload, /* 回程第一段 → reload 停位 (到位下发拨盘第 2 转) */
    axis_reload_wait, /* 停在 reload 位等拨盘 */
    axis_back_work,   /* 顶回工作位，到位后上膛等待 */
    axis_ret,         /* 正常回程: 到 home 泄力 + 开火 */
    axis_work_wait,   /* 保持工作位，等待舵机上膛后再回程 */
} axis_state_e;

static axis_state_e axis_state;

/* 拨盘增益表 [第几发-2][第几转]: 换弹只在第 2~4 发, 每发转 2 次,
 * 转动惯量在两次之间 (弹被取走) 变, 所以两次用不同增益 */
static const float loader_lqr[3][2][2] = {
    {{LOADER_LQR_K0_2_1, LOADER_LQR_K1_2_1}, {LOADER_LQR_K0_2_2, LOADER_LQR_K1_2_2}},
    {{LOADER_LQR_K0_3_1, LOADER_LQR_K1_3_1}, {LOADER_LQR_K0_3_2, LOADER_LQR_K1_3_2}},
    {{LOADER_LQR_K0_4_1, LOADER_LQR_K1_4_1}, {LOADER_LQR_K0_4_2, LOADER_LQR_K1_4_2}},
};

/* 切拨盘增益: 只在盘停下来之后调, 运动中途换 K 会有力矩跳变 */
static void loader_set_gain(uint8_t shot, uint8_t turn)
{
    uint8_t row  = (shot >= 1) ? (uint8_t)(shot - 1) : 0; /* shot: 1..3 → 第 2/3/4 发 */
    uint8_t rows = (uint8_t)(sizeof(loader_lqr) / sizeof(loader_lqr[0]));

    if (row >= rows) row = (uint8_t)(rows - 1);
    if (turn > 1) turn = 1;

    reload_motor->base.controller.lqr.K[0] = loader_lqr[row][turn][0];
    reload_motor->base.controller.lqr.K[1] = loader_lqr[row][turn][1];
}

void shoot_init(void)
{
    Motor_Init_Config_s friction_config = {
        .offline_init_config =
            {
                .name       = "3508_1", // 设备名称
                .timeout_ms = 100,      // 超时时间
                .beep_times = 1,        // 蜂鸣次数
                .enable     = 1,        // 是否启用离线管理
            },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan = BSP_CAN_HANDLE1,
            },
        .controller_init_config =
            {
                .lqr_init =
                    {
                        .K         = {FRICTION_LQR_K0, FRICTION_LQR_K1},
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
    // 左拉簧轴
    friction_config.transport_config.can.tx_id = 1;
    left_friction_motor                        = Motor_DJI_Init(&friction_config);
    if (left_friction_motor == NULL)
    {
        LOG_E("friction_l init failed");
        return;
    }
    // 右拉簧轴, 只改 tx_id, 镜像由 FRICTION_RIGHT_SIGN 处理
    friction_config.transport_config.can.tx_id     = 4;
    friction_config.offline_init_config.name       = "3508_2";
    friction_config.offline_init_config.beep_times = 4;
    right_friction_motor                           = Motor_DJI_Init(&friction_config);
    if (right_friction_motor == NULL)
    {
        LOG_E("friction_r init failed");
        return;
    }

    Motor_Init_Config_s servo_config = {
        .offline_init_config =
            {
                .name       = "trigger",
                .timeout_ms = 100,
                .beep_times = 0,
                .enable     = 0,
            },
        .transport = MOTOR_TRANSPORT_PWM,
        .transport_config.pwm =
            {
                .htim    = &htim8,
                .Channel = TIM_CHANNEL_1,
                .dutyx10 = 0,
                .Mode    = PWM_MODE_BLOCKING,
            },
        .controller_init_config = {0},
        .setting_init_config =
            {
                .loop_type      = ANGLE_LOOP,
                .algorithm_type = CONTROL_PID,
                .enableflag     = 0,
            },
        .motor_init_info =
            {
                .motor_type = SERVO_GENERIC,
                .gear_ratio = 1.0f,
            },
    };
    /* 原模块初始化会启动 PWM，先清除板级默认脉宽，避免未上线时舵机动作。 */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    htim8.Instance->EGR = TIM_EVENTSOURCE_UPDATE;
    trigger             = Motor_Servo_Init(&servo_config, 50, 250, 0, 90);
    if (trigger == NULL)
    {
        LOG_E("trigger init failed");
        return;
    }

    trigger_set_enable(false);
    Motor_SetRef((Motor_Base *)trigger, TRIGGER_FIRE_DEG);

    Motor_Init_Config_s loader_config = {
        .offline_init_config =
            {
                .name       = "dm4310",
                .timeout_ms = 100,
                .beep_times = 2,
                .enable     = 1,
            },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan  = BSP_CAN_HANDLE1,
                .tx_id = 0x23,
                .rx_id = 0x63,
            },
        .controller_init_config =
            {
                .lqr_init =
                    {
                        .K         = {LOADER_LQR_K0_2_1, LOADER_LQR_K1_2_1}, /* 上电 = 第 2 发转 1 那组 */
                        .state_dim = 2,
                    },
            },
        .setting_init_config =
            {
                .loop_type      = ANGLE_LOOP,
                .algorithm_type = CONTROL_LQR,
            },
        .motor_init_info = {.motor_type = DM4310, .gear_ratio = 1, .max_torque = 10, .torque_constant = 0.093f},
    };
    reload_motor = Motor_DM_Init(&loader_config, DM_MIT_MODE);
    if (reload_motor == NULL)
    {
        LOG_E("reload_motor init failed");
        return;
    }

    /* 同步兵 DM：应用只做软件启停，初始化期间由模块持续发送零力矩帧。 */
    Motor_Stop((Motor_Base *)reload_motor);

    /* 3508 在 shoot_func() 里等首帧反馈、记下待机位零点之后才使能 */

    LOG_I("darts shoot init success");
}

/* 同步带直线行程 mm → 转子侧 rad (Motor_SetRef 单位) */
static float friction_rad(const DJI_Motor_t *motor, float mm) { return mm / FRICTION_PULLEY_RADIUS_MM * motor->base.info.gear_ratio; }

/* 本轴带行程 (mm): 正 = 去程方向 (右轴镜像, 去程为负) */
static float friction_x_mm(const DJI_Motor_t *motor, float home)
{
    return (motor->base.measure.total_angle - home) * FRICTION_PULLEY_RADIUS_MM / motor->base.info.gear_ratio;
}

/* 每发从当前待机位置重新建立相对坐标，吸收停机后的机械回差和带位移。 */
static void friction_rebase_home(void)
{
    float out_mm = (shot_cnt == 0) ? FRICTION_TRAVEL_MM : FRICTION_FIRST_PULL_MM;
    home_l       = left_friction_motor->base.measure.total_angle;
    home_r       = right_friction_motor->base.measure.total_angle;
    out_l        = home_l + friction_rad(left_friction_motor, out_mm);
    out_r        = home_r + FRICTION_RIGHT_SIGN * friction_rad(right_friction_motor, out_mm);
    work_l       = home_l + friction_rad(left_friction_motor, FRICTION_TRAVEL_MM);
    work_r       = home_r + FRICTION_RIGHT_SIGN * friction_rad(right_friction_motor, FRICTION_TRAVEL_MM);
    reload_l     = home_l + friction_rad(left_friction_motor, FRICTION_RELOAD_MM);
    reload_r     = home_r + FRICTION_RIGHT_SIGN * friction_rad(right_friction_motor, FRICTION_RELOAD_MM);
}

/* 只切换最终目标，由每周期的参考位置推进限制所有阶段的运动速度。 */
static void friction_set_targets(float left_target, float right_target)
{
    target_l = left_target;
    target_r = right_target;
}

/* 首次出发或掉线恢复时，从实际位置继续推进，避免追赶停机前的旧参考位置。 */
static void friction_sync_refs(void)
{
    Motor_SetRef((Motor_Base *)left_friction_motor, left_friction_motor->base.measure.total_angle);
    Motor_SetRef((Motor_Base *)right_friction_motor, right_friction_motor->base.measure.total_angle);
}

static void friction_update_targets(void)
{
    DJI_Motor_t *motors[2]  = {left_friction_motor, right_friction_motor};
    float        targets[2] = {target_l, target_r};
    float        refs[2];
    for (uint8_t i = 0; i < 2; ++i)
    {
        float ref  = motors[i]->base.controller.ref;
        float step = friction_rad(motors[i], fmaxf(FRICTION_MOVE_STEP, 0.0f));
        refs[i]    = fminf(fmaxf(targets[i], ref - step), ref + step);
    }
    Motor_SetRef((Motor_Base *)left_friction_motor, refs[0]);
    Motor_SetRef((Motor_Base *)right_friction_motor, refs[1]);
}

static bool friction_axes_arrived(float left_target, float right_target)
{
    /* 按两轴实际位置和速度判断到位；调速用的中间参考位置不参与状态切换。 */
    return shoot_mode_3508_is_position_arrived(left_friction_motor, left_target) &&
           shoot_mode_3508_is_position_arrived(right_friction_motor, right_target) && shoot_mode_3508_is_stopped(left_friction_motor) &&
           shoot_mode_3508_is_stopped(right_friction_motor);
}

/* home 单独配置位置和速度容差，两轴实际反馈均满足后才泄力。 */
static bool friction_axes_home_arrived(void)
{
    return fabsf(friction_x_mm(left_friction_motor, home_l)) <= FRICTION_HOME_POSITION_TOLERANCE_MM &&
           fabsf(friction_x_mm(right_friction_motor, home_r)) <= FRICTION_HOME_POSITION_TOLERANCE_MM &&
           fabsf(left_friction_motor->base.measure.speed_rad) * FRICTION_PULLEY_RADIUS_MM / left_friction_motor->base.info.gear_ratio <
               FRICTION_HOME_SPEED_TOLERANCE_MM_S &&
           fabsf(right_friction_motor->base.measure.speed_rad) * FRICTION_PULLEY_RADIUS_MM / right_friction_motor->base.info.gear_ratio <
               FRICTION_HOME_SPEED_TOLERANCE_MM_S;
}

/* 从待机位出发: ref 先回 home，再使能两轴，使能瞬间位置误差为 0。 */
static void friction_axis_engage(void)
{
    friction_set_targets(home_l, home_r);
    friction_sync_refs();
    Motor_Start((Motor_Base *)left_friction_motor);
    Motor_Start((Motor_Base *)right_friction_motor);
}

static float friction_spring_x_mm(const DJI_Motor_t *motor, float home, float direction)
{
    float x = direction * friction_x_mm(motor, home) - FRICTION_SPRING_START_MM;
    return x > 0.0f ? direction * x : 0.0f;
}

/* 拉簧前馈始终按有效带位移计算，回程退入空行程时归零。 */
static void friction_spring_compensate(void)
{
#if 1 /* 拉簧轴带载运行需要补偿弹簧力 */
    left_friction_motor->base.controller.feedforward_torque = FRICTION_FF_K * friction_spring_x_mm(left_friction_motor, home_l, 1.0f);
    right_friction_motor->base.controller.feedforward_torque =
        FRICTION_FF_K * friction_spring_x_mm(right_friction_motor, home_r, FRICTION_RIGHT_SIGN);
#endif
}

/* 回程两轴位置到位且停稳后泄力。 */
static void friction_release_at_home(void)
{
    Motor_Stop((Motor_Base *)left_friction_motor);
    Motor_Stop((Motor_Base *)right_friction_motor);
}

/* 拨盘 reload 完成判据: 到本发目标角且停住 (电机自身零点累计值) */
static bool reload_done(void) { return shoot_mode_dm_is_arrived(reload_motor, loader_target_rad); }

void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd, bool remote_online)
{
    /* 遥控器掉线只停止输出，保留发数、步骤、零点及目标，上线后继续当前步骤。
     * DM 只关闭软件输出，硬件使能由电机模块负责。

     */
    if (!remote_online || shoot_cmd == NULL || left_friction_motor == NULL || right_friction_motor == NULL || reload_motor == NULL || trigger == NULL)
    {
        Motor_Stop((Motor_Base *)left_friction_motor);
        Motor_Stop((Motor_Base *)right_friction_motor);
        Motor_Stop((Motor_Base *)reload_motor);
        trigger_set_enable(false);
        trigger_last = 1; /* 待机时保持下档重连不触发新一发。 */
        if (!inited) startup_cnt = 0;
        Motor_SetForwardTorque((Motor_Base *)left_friction_motor, 0.0f);
        Motor_SetForwardTorque((Motor_Base *)right_friction_motor, 0.0f);
        return;
    }

    /* 首次初始化的采零等待期间，持续确认 PWM 已关闭。 */
    if (!inited && !trigger_set_enable(false)) return;

    /* 掉线门 (同别的兵种: 每周期判定, 不锁存): 任一掉线 → 三台全停;
     * 恢复在线 → 软件重新使能: DM 保持软件使能, 3508
     * 仅序列进行中
     * (非待机位) 使能 (泄力铁律) */
    if (Module_Offline_get_device_status(left_friction_motor->base.offline_dev) == STATE_OFFLINE ||
        Module_Offline_get_device_status(right_friction_motor->base.offline_dev) == STATE_OFFLINE ||
        Module_Offline_get_device_status(reload_motor->base.offline_dev) == STATE_OFFLINE)
    {
        Motor_Stop((Motor_Base *)left_friction_motor);
        Motor_Stop((Motor_Base *)right_friction_motor);
        Motor_Stop((Motor_Base *)reload_motor);
        return;
    }
    /* 首次初始化: 等待反馈后记录同步带位置，并设置拨盘绝对基准。
     * 注: offline 设备注册后头 100ms 恒报 ONLINE, 所以先等过
     * 100ms 窗口 (75 × 2ms = 150ms) 再判状态 */
    if (!inited)
    {
        if (++startup_cnt < 75) return;

        if (Module_Offline_get_device_status(left_friction_motor->base.offline_dev) != STATE_ONLINE ||
            Module_Offline_get_device_status(right_friction_motor->base.offline_dev) != STATE_ONLINE)
            return;

        friction_rebase_home();
        friction_set_targets(home_l, home_r);
        friction_sync_refs();
        loader_target_rad = LOADER_ZERO_OFFSET_DEG * PI / 180.0f;
        loader_set_gain(0, 0);
        Motor_SetRef((Motor_Base *)reload_motor, loader_target_rad);
        Motor_SetRef((Motor_Base *)trigger, TRIGGER_FIRE_DEG);
        inited = true; /* 到 home 泄力: 上电不使能, 等拨杆触发时由 axis_idle engage */
    }

    Motor_Start((Motor_Base *)reload_motor);
    if (!trigger_set_enable(true)) return;
    if (axis_state != axis_idle)
    {
        if (!left_friction_motor->base.setting.enableflag || !right_friction_motor->base.setting.enableflag)
        {
            friction_sync_refs();
            /* 等待期间中断输出后，恢复时给舵机完整的动作时间。 */
            if (axis_state == axis_work_wait) trigger_ready_start_us = BSP_DWT_GetTimeline_us();
        }
        Motor_Start((Motor_Base *)left_friction_motor);
        Motor_Start((Motor_Base *)right_friction_motor);
    }

    /* 采零完成后才接收边沿：保持下档上线不会开火。 */
    uint8_t fire_now  = (shoot_cmd->shoot_mode == shoot_on);
    uint8_t fire_edge = fire_now && trigger_last == 0;
    trigger_last      = fire_now;

    /* CH5 下拨边沿触发, 4 发一轮循环:
     * 第1发: home → 工作位上膛并等待 → 回 home 泄力 + 开火;
     * 第2~4发: home → 独立下拉位等拨盘第1转 → reload
     * 位等拨盘第2转
     * → 工作位上膛并等待 → 回 home 泄力 + 开火。所有同步带行程使用同一调速宏。 */
    switch (axis_state)
    {
    case axis_idle:
        if (shot_cnt >= 4) shot_cnt = 0; /* 一轮四发打完刷新, 一直循环 */
        if (fire_edge == 0) break;       /* 等下一次下拨 */

        friction_rebase_home();
        friction_axis_engage();
        friction_set_targets(out_l, out_r);
        axis_state = axis_out;
        break;

    case axis_out:
        if (friction_axes_arrived(out_l, out_r)) axis_state = axis_out_done;
        break;

    case axis_out_done: /* 本发首次下拉到位 */
        if (shot_cnt == 0)
        { /* 第1发不换弹：上膛并保持工作位，等待舵机动作。 */
            Motor_SetRef((Motor_Base *)trigger, TRIGGER_READY_DEG);
            trigger_ready_start_us = BSP_DWT_GetTimeline_us();
            axis_state             = axis_work_wait;
            break;
        }

        /* 第 2~4 发: 盘第 1 转 (弹取走前), 转完再走小回程 */
        loader_set_gain(shot_cnt, 0);
        loader_target_rad += ONE_BULLET_DELTA_ANGLE * PI / 180.0f;
        Motor_SetRef((Motor_Base *)reload_motor, loader_target_rad);
        axis_state = axis_turn_wait;
        break;

    case axis_turn_wait: /* 独立下拉位上等盘转完 */
        if (reload_done())
        {
            friction_set_targets(reload_l, reload_r);
            axis_state = axis_back_reload;
        }
        break;

    case axis_back_reload:
        if (friction_axes_arrived(reload_l, reload_r))
        {
            loader_set_gain(shot_cnt, 1);                              /* 弹已取走, 盘是停着的, 在这里换第 2 转的增益 */
            loader_target_rad += ONE_BULLET_DELTA_ANGLE * PI / 180.0f; /* 拨盘放一发 */
            Motor_SetRef((Motor_Base *)reload_motor, loader_target_rad);
            axis_state = axis_reload_wait;
        }
        break;

    case axis_reload_wait: /* 扳机保持开火位，同步带保持 reload 位等待拨盘 */
        if (reload_done())
        {
            friction_set_targets(work_l, work_r);
            axis_state = axis_back_work;
        }
        break;

    case axis_back_work: /* 第2~4发回到工作位后上膛并等待 */
        if (friction_axes_arrived(work_l, work_r))
        {
            Motor_SetRef((Motor_Base *)trigger, TRIGGER_READY_DEG);
            trigger_ready_start_us = BSP_DWT_GetTimeline_us();
            axis_state             = axis_work_wait;
        }
        break;

    case axis_work_wait:
        if (BSP_DWT_GetTimeline_us() - trigger_ready_start_us >= (uint64_t)TRIGGER_READY_WAIT_MS * 1000ULL)
        {
            friction_set_targets(home_l, home_r);
            axis_state = axis_ret;
        }
        break;

    case axis_ret: /* 保持 LQR 和按位移计算的前馈，回到 home 且停稳后泄力 */
        if (friction_axes_home_arrived())
        {
            friction_release_at_home();                            /* 泄力 */
            Motor_SetRef((Motor_Base *)trigger, TRIGGER_FIRE_DEG); /* 开火, 这一发最后一步 */
            shot_cnt++;
            axis_state = axis_idle;
        }
        break;
    }
    if (axis_state != axis_idle) friction_update_targets();
    friction_spring_compensate();
}
