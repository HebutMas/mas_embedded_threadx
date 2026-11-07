/*
 * @Author: abbhhh 804433588@qq.com
 * @Date: 2026-09-15 20:36:42
 * @LastEditors: abbhhh 804433588@qq.com
 * @LastEditTime: 2026-09-15 20:45:43
 * @FilePath: \mas_embedded_threadx\apps\darts\single_board\shoot_func\shoot_func.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Description: 发射机构功能模板
 */

#ifndef _SHOOT_FUNC_H_
#define _SHOOT_FUNC_H_

#include "darts_def.h" /* TODO: 改为你的 def 文件 */
#include <stdbool.h>

void shoot_init(void);

void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd, bool remote_online);

#endif // _SHOOT_FUNC_H_
