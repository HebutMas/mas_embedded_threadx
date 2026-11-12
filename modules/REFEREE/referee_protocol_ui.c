#include "referee_protocol_ui.h"
#include "module_referee.h"
#include "tx_api.h"
#include <string.h>

// 填充 figure_name + cfg1/cfg2 通用字段
static void fig_base(ui_figure_t *fig, const char name[3], uint8_t op, uint8_t type, uint8_t layer, uint8_t color, uint16_t width, uint16_t sx,
                     uint16_t sy, uint16_t da, uint16_t db)
{
    memset(fig, 0, sizeof(ui_figure_t));
    fig->figure_name[0]    = (uint8_t)name[0];
    fig->figure_name[1]    = (uint8_t)name[1];
    fig->figure_name[2]    = (uint8_t)name[2];
    fig->cfg1.operate_type = op;
    fig->cfg1.figure_type  = type;
    fig->cfg1.layer        = layer;
    fig->cfg1.color        = color;
    fig->cfg1.details_a    = da;
    fig->cfg1.details_b    = db;
    fig->cfg2.width        = width;
    fig->cfg2.start_x      = sx;
    fig->cfg2.start_y      = sy;
}

void ui_figure_set_name(ui_figure_t *fig, const char name[3])
{
    if (fig == NULL) return;
    fig->figure_name[0] = (uint8_t)name[0];
    fig->figure_name[1] = (uint8_t)name[1];
    fig->figure_name[2] = (uint8_t)name[2];
}

void ui_draw_line(ui_figure_t *fig, const char name[3], uint8_t op, uint8_t layer, uint8_t color, uint16_t width, uint16_t x1, uint16_t y1,
                  uint16_t x2, uint16_t y2)
{
    if (fig == NULL) return;
    fig_base(fig, name, op, UI_GRAPH_TYPE_LINE, layer, color, width, x1, y1, 0, 0);
    fig->cfg3.details_d = x2;
    fig->cfg3.details_e = y2;
}

void ui_draw_rect(ui_figure_t *fig, const char name[3], uint8_t op, uint8_t layer, uint8_t color, uint16_t width, uint16_t x1, uint16_t y1,
                  uint16_t x2, uint16_t y2)
{
    if (fig == NULL) return;
    fig_base(fig, name, op, UI_GRAPH_TYPE_RECT, layer, color, width, x1, y1, 0, 0);
    fig->cfg3.details_d = x2;
    fig->cfg3.details_e = y2;
}

void ui_draw_circle(ui_figure_t *fig, const char name[3], uint8_t op, uint8_t layer, uint8_t color, uint16_t width, uint16_t cx, uint16_t cy,
                    uint16_t radius)
{
    if (fig == NULL) return;
    fig_base(fig, name, op, UI_GRAPH_TYPE_CIRCLE, layer, color, width, cx, cy, 0, 0);
    fig->cfg3.details_c = radius;
}

void ui_draw_oval(ui_figure_t *fig, const char name[3], uint8_t op, uint8_t layer, uint8_t color, uint16_t width, uint16_t cx, uint16_t cy,
                  uint16_t rx, uint16_t ry)
{
    if (fig == NULL) return;
    fig_base(fig, name, op, UI_GRAPH_TYPE_OVAL, layer, color, width, cx, cy, 0, 0);
    fig->cfg3.details_d = rx;
    fig->cfg3.details_e = ry;
}

void ui_draw_arc(ui_figure_t *fig, const char name[3], uint8_t op, uint8_t layer, uint8_t color, uint16_t width, uint16_t cx, uint16_t cy,
                 uint16_t start_angle, uint16_t end_angle, uint16_t rx, uint16_t ry)
{
    if (fig == NULL) return;
    fig_base(fig, name, op, UI_GRAPH_TYPE_ARC, layer, color, width, cx, cy, start_angle, end_angle);
    fig->cfg3.details_d = rx;
    fig->cfg3.details_e = ry;
}

void ui_draw_float(ui_figure_t *fig, const char name[3], uint8_t op, uint8_t layer, uint8_t color, uint16_t width, uint16_t font_size, uint8_t digit,
                   uint16_t x, uint16_t y, int32_t value)
{
    if (fig == NULL) return;
    fig_base(fig, name, op, UI_GRAPH_TYPE_FLOAT, layer, color, width, x, y, font_size, digit);
    /* value 分拆写入 details_c/d/e：共 10+11+11=32bit */
    uint32_t uv         = (uint32_t)value;
    fig->cfg3.details_c = (uv) & 0x3FFu;
    fig->cfg3.details_d = (uv >> 10) & 0x7FFu;
    fig->cfg3.details_e = (uv >> 21) & 0x7FFu;
}

void ui_draw_int(ui_figure_t *fig, const char name[3], uint8_t op, uint8_t layer, uint8_t color, uint16_t width, uint16_t font_size, uint16_t x,
                 uint16_t y, int32_t value)
{
    if (fig == NULL) return;
    fig_base(fig, name, op, UI_GRAPH_TYPE_INT, layer, color, width, x, y, font_size, 0);
    uint32_t uv         = (uint32_t)value;
    fig->cfg3.details_c = (uv) & 0x3FFu;
    fig->cfg3.details_d = (uv >> 10) & 0x7FFu;
    fig->cfg3.details_e = (uv >> 21) & 0x7FFu;
}

void ui_draw_string(ui_string_t *str_fig, const char name[3], uint8_t op, uint8_t layer, uint8_t color, uint16_t width, uint16_t font_size,
                    uint16_t x, uint16_t y, const char *text)
{
    if (str_fig == NULL || text == NULL) return;
    memset(str_fig, 0, sizeof(ui_string_t));
    uint8_t len = (uint8_t)strlen(text);
    if (len > 30u) len = 30u;
    ui_figure_t *fig = &str_fig->figure_config;
    fig_base(fig, name, op, UI_GRAPH_TYPE_CHAR, layer, color, width, x, y, font_size, len);
    memcpy(str_fig->text, text, len);
}

void ui_string_set_text(ui_string_t *str_fig, const char *text, uint8_t str_len)
{
    if (str_fig == NULL || text == NULL) return;
    uint8_t copy_len = (str_len > 30u) ? 30u : str_len;
    size_t  text_len = strlen(text);
    if (copy_len > text_len) copy_len = (uint8_t)text_len; /* 防止读取超出字符串实际长度 */
    memset(str_fig->text, 0, sizeof(str_fig->text));
    memcpy(str_fig->text, text, copy_len);
    /* 同步更新 details_b 中存储的字符长度 */
    str_fig->figure_config.cfg1.details_b = copy_len;
}

void ui_delete_layer(ui_delete_layer_t *del, uint8_t delete_type, uint8_t layer)
{
    if (del == NULL) return;
    del->delete_type = delete_type;
    del->layer       = layer;
}

void ui_figure_set_op(ui_figure_t *fig, uint8_t op)
{
    if (fig == NULL) return;
    fig->cfg1.operate_type = op;
}

void ui_figure_set_int_value(ui_figure_t *fig, int32_t value)
{
    if (fig == NULL) return;
    uint32_t uv         = (uint32_t)value;
    fig->cfg3.details_c = (uv) & 0x3FFu;
    fig->cfg3.details_d = (uv >> 10) & 0x7FFu;
    fig->cfg3.details_e = (uv >> 21) & 0x7FFu;
}

void ui_send_figure_1(const ui_figure_t *fig, uint32_t timeout) { Module_Referee_Send_Interaction(UI_CMD_DRAW_1, fig, sizeof(ui_figure_t), timeout); }

void ui_send_figure_2(const ui_figure_2_t *figs, uint32_t timeout)
{
    Module_Referee_Send_Interaction(UI_CMD_DRAW_2, figs, sizeof(ui_figure_2_t), timeout);
}

void ui_send_figure_5(const ui_figure_5_t *figs, uint32_t timeout)
{
    Module_Referee_Send_Interaction(UI_CMD_DRAW_5, figs, sizeof(ui_figure_5_t), timeout);
}

void ui_send_figure_7(const ui_figure_7_t *figs, uint32_t timeout)
{
    Module_Referee_Send_Interaction(UI_CMD_DRAW_7, figs, sizeof(ui_figure_7_t), timeout);
}

void ui_send_string(const ui_string_t *str_fig, uint32_t timeout)
{
    Module_Referee_Send_Interaction(UI_CMD_DRAW_STRING, str_fig, sizeof(ui_string_t), timeout);
}

void ui_send_delete_layer(const ui_delete_layer_t *del, uint32_t timeout)
{
    Module_Referee_Send_Interaction(UI_CMD_DEL_LAYER, del, sizeof(ui_delete_layer_t), timeout);
}
