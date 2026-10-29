#include "car_state.h"
#include "kinematics.h"
#include <math.h>

static car_state_e car_state = STATE_POSE_ADJUST;
static uint16_t    stable_cnt = 0;

#define CAR_DONE_THETA   0.30f  /* 两腿 |θ|<17° 判起立完成 */
#define CAR_DONE_CYCLES  1      /* 无防抖: 瞬时满足即切 */

static inline uint8_t leg_in_window(float theta)
{
    return (theta > CAR_POSE_THETA_LO && theta <= CAR_POSE_THETA_HI);
}

void car_state_init(void)
{
    car_state = STATE_POSE_ADJUST;
    stable_cnt = 0;
}

car_state_e car_state_get(void)
{
    return car_state;
}

void car_state_update(void)
{
    const fk *l = kinematics_get_left();
    const fk *r = kinematics_get_right();
    if (l == NULL || r == NULL)
        return;

    switch (car_state)
    {
    case STATE_POSE_ADJUST:
        /* 两腿都摆进到位区间 → 起立 (每帧判断, 无防抖) */
        if (leg_in_window(l->theta) && leg_in_window(r->theta))
        {
            car_state = STATE_STANDUP;
        }
        break;

    case STATE_STANDUP:
        if (fabsf(l->theta) < CAR_DONE_THETA && fabsf(r->theta) < CAR_DONE_THETA)
        {
            if (++stable_cnt >= CAR_DONE_CYCLES)
            {
                car_state = STATE_BALANCE;
                stable_cnt = 0; /* 切换后清零, 防溢出兜底 */
            }
        }
        else
        {
            stable_cnt = 0; /* 任一腿超窗, 重新攒 */
            /* 回退边: 起立中腿甩出到位区间 → 回调姿重摆 (起立失败自动重试) */
            if (!leg_in_window(l->theta) || !leg_in_window(r->theta))
            {
                car_state = STATE_POSE_ADJUST;
            }
        }
        break;

    case STATE_BALANCE:
    case STATE_FALLEN:
    default:
        /* latch 不回退 (掉线用 car_state_reset); 后续 JUMP/STEP 在此扩展转移 */
        break;
    }
}

void car_state_reset(void)
{
    car_state = STATE_POSE_ADJUST; /* 掉线: 回调姿 (顺带清 FALLEN) */
    stable_cnt = 0;
}

void car_state_set_fallen(void)
{
    car_state = STATE_FALLEN;
}
