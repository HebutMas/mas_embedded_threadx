#include "motor_servo.h"
#include "bsp_def.h"
#include "module_offline.h"
#include "user_lib.h"
#include <string.h>

#define LOG_TAG "motor_servo"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static inline uint32_t angle_to_pulse(Servo_Motor_t *s, float angle_deg)
{
    float ratio = (angle_deg - s->min_angle_deg) / (s->max_angle_deg - s->min_angle_deg);
    VAL_LIMIT(ratio, 0.0f, 1.0f);
    return (uint32_t)(s->min_pulse + ratio * (s->max_pulse - s->min_pulse));
}

static void servo_apply(Motor_Base *base)
{
    Servo_Motor_t *s = MOTOR_GET_DERIVED(base, Servo_Motor_t);

    if (!base->setting.enableflag) return;

    float    target_deg = base->controller.ref;
    uint32_t pulse      = angle_to_pulse(s, target_deg);
    BSP_PWM_SetPulse(s->pwm_dev, pulse);
}

Servo_Motor_t *Motor_Servo_Init(Motor_Init_Config_s *config, uint16_t min_pulse, uint16_t max_pulse, float min_angle, float max_angle)
{
    Servo_Motor_t *motor = NULL;
    BSP_MEM_ALLOC_WAIT(motor, sizeof(Servo_Motor_t), TX_NO_WAIT);
    if (!motor)
    {
        LOG_E("Failed to allocate memory for Servo motor");
        return NULL;
    }
    memset(motor, 0, sizeof(Servo_Motor_t));

    motor->base.type           = SERVO_GENERIC;
    motor->base.transport      = MOTOR_TRANSPORT_PWM;
    motor->base.info           = config->motor_init_info;
    motor->base.setting        = config->setting_init_config;
    motor->base.setting.loop_type = OPEN_LOOP; /* 舵机为开环控制 (角度直通) */
    motor->min_pulse           = min_pulse;
    motor->max_pulse           = max_pulse;
    motor->min_angle_deg       = min_angle;
    motor->max_angle_deg       = max_angle;

    PWM_Device *pwm_dev = BSP_PWM_Device_Init(&config->transport_config.pwm);
    if (!pwm_dev)
    {
        LOG_E("Failed to initialize PWM device");
        BSP_MEM_FREE(motor);
        return NULL;
    }
    motor->pwm_dev            = pwm_dev;
    motor->base.transport_dev = pwm_dev;

    motor->base.offline_dev = Module_Offline_register(&config->offline_init_config);

    BSP_PWM_Start(pwm_dev, NULL, 0);

    /* 绑定两阶段调度, 注册到全局链表 */
    motor->base.Apply   = servo_apply;
    Motor_Register(&motor->base);

    LOG_I("Servo init (pulse:%d-%d angle:%.0f-%.0f deg)", min_pulse, max_pulse, min_angle, max_angle);
    return motor;
}
