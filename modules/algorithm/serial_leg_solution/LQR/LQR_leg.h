#ifndef LQR_LEG_H
#define LQR_LEG_H

#include "solution_def.h"

void LQR_K_calc(float *K, float L_0);
void LQR_calc(float target_x, float target_dx);
const LQR *LQR_get(void);

#endif // LQR_LEG_H