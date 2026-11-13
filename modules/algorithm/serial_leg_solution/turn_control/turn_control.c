/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-10-20
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-10-20
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\turn_control\turn_control.c
 * @Description: 转向控制 (mode0 单环, 对齐 luntui1 x_observe.c turn_control)
 */

#include "turn_control.h"
#include "observer.h"
#include "car_state.h"

static const float TURN_K11 = 22.3607f * 2.2f; /* 49.19, luntui1 原值 */
static const float TURN_K12 = 4.6885f * 1.4f;  /* 6.56,  luntui1 原值 */
static const float TURN_MAX = 3.5f;            /* 限幅,  luntui1 原值 */

static turn_control_t turn_ctrl;

void turn_control_init(void)
{
    turn_ctrl.delta_t = 0.0f;
}

void turn_control_update(float turn_set_angle)
{
    const observer *obs = observer_get();
    if (obs == NULL) return;

    /* 调姿期间不更新转向: 摆腿时轮子锁定, 遥控转向不攒 (对齐 luntui1 摆腿跳过 turn_control) */
    if (car_state_get() == STATE_POSE_ADJUST)
    {
        turn_ctrl.delta_t = 0.0f;
        return;
    }

    /* mode0 单环: 设定角前馈 + 偏航角速度阻尼
     * (对齐 luntui1 x_observe.c turn_control 的 mode0 分支) */
    turn_ctrl.delta_t = TURN_K11 * (-turn_set_angle) + TURN_K12 * obs->dyaw;

    if (turn_ctrl.delta_t > TURN_MAX) {
        turn_ctrl.delta_t = TURN_MAX;
    } else if (turn_ctrl.delta_t < -TURN_MAX) {
        turn_ctrl.delta_t = -TURN_MAX;
    }
}

const turn_control_t *turn_control_get(void)
{
    return &turn_ctrl;
}

void turn_control_reset(void)
{
    turn_ctrl.delta_t = 0.0f;
}
