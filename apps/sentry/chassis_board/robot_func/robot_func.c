#include "robot_func.h"
#include "module_referee.h"
#include "referee_protocol.h"

void handle_referee_data(ChassisToGimbal_referee_t *referee_data)
{
    const robot_status_t    *robot_status    = (robot_status_t *)Module_Referee_Get_cmd_data(CMD_ID_ROBOT_STATUS);
    const allowed_bullet_t  *allowed_bullet  = (allowed_bullet_t *)Module_Referee_Get_cmd_data(CMD_ID_ALLOWED_BULLET);
    const game_status_t     *game_status     = (game_status_t *)Module_Referee_Get_cmd_data(CMD_ID_GAME_STATUS);
    const power_heat_data_t *power_heat_data = (power_heat_data_t *)Module_Referee_Get_cmd_data(CMD_ID_POWER_HEAT_DATA);
    if (!robot_status || !allowed_bullet || !game_status || !power_heat_data) return;

    // 红方id为1-7，蓝方id为101-107
    if (robot_status->robot_id >= 100)
    {
        referee_data->robot_color = 1; // 蓝方
    }
    else
    {
        referee_data->robot_color = 0; // 红方（默认）
    }
    referee_data->bullet_allow     = allowed_bullet->bullet_17mm_allowed;
    referee_data->current_hp       = robot_status->current_hp;
    referee_data->game_progress    = game_status->type_progress.game_progress;
    referee_data->shooter_heat_pct = power_heat_data->shooter_17mm_heat;
}
