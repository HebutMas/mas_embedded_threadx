/*
 * @Description: 发射机构功能模板
 */

#ifndef _SHOOT_FUNC_H_
#define _SHOOT_FUNC_H_

#include "<robot>_def.h" /* TODO: 改为你的 def 文件 */

int shoot_init(void);

void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd);

#endif // _SHOOT_FUNC_H_
