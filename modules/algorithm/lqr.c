#include "lqr.h"
#if defined(STM32F407xx) || defined(STM32H723xx)
#include <arm_math.h>
#endif
#include <string.h>

void LQRInit(LQRInstance *lqr, LQR_Init_Config_s *config)
{
    if (lqr == NULL || config == NULL) return;

    memset(lqr, 0, sizeof(LQRInstance));

    lqr->state_dim = config->state_dim;
    if (lqr->state_dim < 1 || lqr->state_dim > 2) return;

    for (uint8_t i = 0; i < lqr->state_dim; i++) lqr->K[i] = config->K[i];
}

float LQRCalculate(LQRInstance *lqr, float rad_degree, float rad_angular_velocity, float rad_ref)
{
    if (lqr == NULL || lqr->state_dim < 1 || lqr->state_dim > 2) return 0.0f;

    /* 计算 LQR 输出 */
    if (lqr->state_dim == 1)
    {
        lqr->measure = rad_angular_velocity;
        lqr->ref     = rad_ref;
        float err    = lqr->measure - lqr->ref;
        lqr->output  = -(lqr->K[0] * err);
    }
    else /* state_dim == 2 */
    {
        lqr->measure = rad_degree;
        lqr->ref     = rad_ref;
        float err    = lqr->measure - lqr->ref;
        lqr->output  = -(lqr->K[0] * err) - lqr->K[1] * rad_angular_velocity;
    }

    return lqr->output;
}
