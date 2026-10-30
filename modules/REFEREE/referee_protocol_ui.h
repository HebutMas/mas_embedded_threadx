/*
 * @Author: laladuduqq 17503181697@163.com
 * @Date: 2026-03-12 15:37:27
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-23 12:19:02
 * @FilePath: /mas_embedded_threadx/modules/REFEREE/referee_protocol_ui.h
 * @Description: RoboMaster 2026 V1.2.0 UI绘制协议封装
 */
#ifndef _REFEREE_PROTOCOL_UI_H_
#define _REFEREE_PROTOCOL_UI_H_

#include <stdint.h>
#include <stdbool.h>

#include "referee_protocol.h"

// 宏定义

// UI 图层
#define REFEREE_UI_LAYER_MAX    9

// UI 颜色
#define REFEREE_UI_COLOR_MAIN   0 // 己方颜色（红/蓝）
#define REFEREE_UI_COLOR_YELLOW 1
#define REFEREE_UI_COLOR_GREEN  2
#define REFEREE_UI_COLOR_ORANGE 3
#define REFEREE_UI_COLOR_PURPLE 4
#define REFEREE_UI_COLOR_PINK   5
#define REFEREE_UI_COLOR_CYAN   6
#define REFEREE_UI_COLOR_BLACK  7
#define REFEREE_UI_COLOR_WHITE  8

// UI 图形类型
#define UI_GRAPH_TYPE_LINE      0 // 直线
#define UI_GRAPH_TYPE_RECT      1 // 矩形
#define UI_GRAPH_TYPE_CIRCLE    2 // 正圆
#define UI_GRAPH_TYPE_OVAL      3 // 椭圆
#define UI_GRAPH_TYPE_ARC       4 // 圆弧
#define UI_GRAPH_TYPE_FLOAT     5 // 浮点数
#define UI_GRAPH_TYPE_INT       6 // 整型数
#define UI_GRAPH_TYPE_CHAR      7 // 字符

// UI 操作类型
#define UI_OP_NOOP              0 // 空操作
#define UI_OP_ADD               1 // 增加
#define UI_OP_MODIFY            2 // 修改
#define UI_OP_DELETE            3 // 删除

// 删除图层操作类型
#define UI_DEL_NOOP             0 // 空操作
#define UI_DEL_LAYER            1 // 删除图层
#define UI_DEL_ALL              2 // 删除所有

// 屏幕分辨率参考
#define UI_SCREEN_WIDTH         1920
#define UI_SCREEN_HEIGHT        1080

// 子内容ID
#define UI_CMD_DEL_LAYER        INTERACTION_ID_DELETE_LAYER  // 0x0100
#define UI_CMD_DRAW_1           INTERACTION_ID_DRAW_1_FIGURE // 0x0101
#define UI_CMD_DRAW_2           INTERACTION_ID_DRAW_2_FIGURE // 0x0102
#define UI_CMD_DRAW_5           INTERACTION_ID_DRAW_5_FIGURE // 0x0103
#define UI_CMD_DRAW_7           INTERACTION_ID_DRAW_7_FIGURE // 0x0104
#define UI_CMD_DRAW_STRING      INTERACTION_ID_DRAW_STRING   // 0x0110

// 数据结构定义

/**
 * 图形配置字1
 * bit[2:0]   operate_type  操作类型
 * bit[5:3]   figure_type   图形类型
 * bit[9:6]   layer         图层 0~9
 * bit[13:10] color         颜色
 * bit[22:14] details_a     详情a（圆弧起始角度 / 浮点/整型字体大小）
 * bit[31:23] details_b     详情b（圆弧终止角度 / 浮点小数位 / 字符长度）
 */
typedef union __attribute__((packed))
{
    struct __attribute__((packed))
    {
        uint32_t operate_type : 3;
        uint32_t figure_type  : 3;
        uint32_t layer        : 4;
        uint32_t color        : 4;
        uint32_t details_a    : 9;
        uint32_t details_b    : 9;
    };
    uint32_t cfg1;
} ui_figure_cfg1_t;

/**
 * 图形配置字2
 * bit[9:0]   width    线宽 / 字体大小建议比例 1:10
 * bit[20:10] start_x  起点/圆心 x 坐标
 * bit[31:21] start_y  起点/圆心 y 坐标
 */
typedef union __attribute__((packed))
{
    struct __attribute__((packed))
    {
        uint32_t width   : 10;
        uint32_t start_x : 11;
        uint32_t start_y : 11;
    };
    uint32_t cfg2;
} ui_figure_cfg2_t;

/**
 * 图形配置字3
 * bit[9:0]   details_c  正圆半径 / 浮点、整型数值低 10bit
 * bit[20:10] details_d  直线/矩形终点（对角顶点）x / 椭圆、圆弧 x 半轴 / 浮点、整型数值中 11bit
 * bit[31:21] details_e  直线/矩形终点（对角顶点）y / 椭圆、圆弧 y 半轴 / 浮点、整型数值高 11bit
 */
typedef union __attribute__((packed))
{
    struct __attribute__((packed))
    {
        uint32_t details_c : 10;
        uint32_t details_d : 11;
        uint32_t details_e : 11;
    };
    uint32_t cfg3;
} ui_figure_cfg3_t;

/**
 * 单个图形元素，对应子内容ID 0x0101~0x0104
 */
typedef struct __attribute__((packed))
{
    uint8_t          figure_name[3]; /* 图形名称（3字节ASCII，逆序存储） */
    ui_figure_cfg1_t cfg1;           /* 操作/类型/图层/颜色/details_a/b */
    ui_figure_cfg2_t cfg2;           /* 线宽/起点坐标 */
    ui_figure_cfg3_t cfg3;           /* details_c/d/e */
} ui_figure_t;

// 删除图层 (2字节)，子内容ID 0x0100
typedef struct __attribute__((packed))
{
    uint8_t delete_type; // UI_DEL_xxx
    uint8_t layer;       // 图层号 0~9
} ui_delete_layer_t;

// 绘制2个图形，子内容ID 0x0102
typedef struct __attribute__((packed))
{
    ui_figure_t figure[2];
} ui_figure_2_t;

// 绘制5个图形，子内容ID 0x0103
typedef struct __attribute__((packed))
{
    ui_figure_t figure[5];
} ui_figure_5_t;

// 绘制7个图形，子内容ID 0x0104
typedef struct __attribute__((packed))
{
    ui_figure_t figure[7];
} ui_figure_7_t;

// 绘制字符串，子内容ID 0x0110
typedef struct __attribute__((packed))
{
    ui_figure_t figure_config; // 图形配置（figure_type=7）
    char        text[30];      // 字符内容（ASCII，未使用字节填0）
} ui_string_t;

// API 函数声明

/**
 * @brief 在线修改操作类型（不重建整帧）
 */
void ui_figure_set_op(ui_figure_t *fig, uint8_t op);

/**
 * @brief 在线修改整型/浮点数字段（不重建整帧）
 */
void ui_figure_set_int_value(ui_figure_t *fig, int32_t value);

/**
 * @brief 初始化图形结构体名称（3字节ASCII）
 * @param fig    目标图形结构体
 * @param name   3字节名称，如 "a01"
 */
void ui_figure_set_name(ui_figure_t *fig, const char name[3]);

/**
 * @brief 绘制直线
 * @param fig        图形结构体
 * @param name       图形名
 * @param op         操作类型 UI_OP_xxx
 * @param layer      图层 0~9
 * @param color      颜色 REFEREE_UI_COLOR_xxx
 * @param width      线宽
 * @param x1,y1      起点
 * @param x2,y2      终点
 */
void ui_draw_line(ui_figure_t *fig, const char name[3], uint8_t op, uint8_t layer, uint8_t color, uint16_t width, uint16_t x1, uint16_t y1,
                  uint16_t x2, uint16_t y2);

/**
 * @brief 绘制矩形
 * @param x1,y1  左下角
 * @param x2,y2  右上角（对角顶点）
 */
void ui_draw_rect(ui_figure_t *fig, const char name[3], uint8_t op, uint8_t layer, uint8_t color, uint16_t width, uint16_t x1, uint16_t y1,
                  uint16_t x2, uint16_t y2);

/**
 * @brief 绘制正圆
 * @param cx,cy  圆心
 * @param radius 半径（pixels）
 */
void ui_draw_circle(ui_figure_t *fig, const char name[3], uint8_t op, uint8_t layer, uint8_t color, uint16_t width, uint16_t cx, uint16_t cy,
                    uint16_t radius);

/**
 * @brief 绘制椭圆
 * @param cx,cy  圆心
 * @param rx,ry  X/Y半轴
 */
void ui_draw_oval(ui_figure_t *fig, const char name[3], uint8_t op, uint8_t layer, uint8_t color, uint16_t width, uint16_t cx, uint16_t cy,
                  uint16_t rx, uint16_t ry);

/**
 * @brief 绘制圆弧
 * @param cx,cy         圆心
 * @param start_angle   起始角度（0=12点钟，顺时针）
 * @param end_angle     终止角度
 * @param rx,ry         X/Y半轴
 */
void ui_draw_arc(ui_figure_t *fig, const char name[3], uint8_t op, uint8_t layer, uint8_t color, uint16_t width, uint16_t cx, uint16_t cy,
                 uint16_t start_angle, uint16_t end_angle, uint16_t rx, uint16_t ry);

/**
 * @brief 绘制浮点数（选手端实际显示值 = value/1000）
 * @param font_size   字体大小
 * @param digit       小数位数
 * @param value       显示值×1000（如1234显示1.234）
 */
void ui_draw_float(ui_figure_t *fig, const char name[3], uint8_t op, uint8_t layer, uint8_t color, uint16_t width, uint16_t font_size, uint8_t digit,
                   uint16_t x, uint16_t y, int32_t value);

/**
 * @brief 绘制整型数
 * @param font_size   字体大小
 * @param value       整型值（int32_t）
 */
void ui_draw_int(ui_figure_t *fig, const char name[3], uint8_t op, uint8_t layer, uint8_t color, uint16_t width, uint16_t font_size, uint16_t x,
                 uint16_t y, int32_t value);

/**
 * @brief 绘制字符串
 * @param str_fig     字符串图形结构体
 * @param name        图形名
 * @param font_size   字体大小
 * @param text        字符内容
 */
void ui_draw_string(ui_string_t *str_fig, const char name[3], uint8_t op, uint8_t layer, uint8_t color, uint16_t width, uint16_t font_size,
                    uint16_t x, uint16_t y, const char *text);

/**
 * @brief 修改字符串内容（不重建整帧）
 * @param str_fig     字符串图形结构体
 * @param text        新内容
 * @param str_len     字符长度（最多30）
 */
void ui_string_set_text(ui_string_t *str_fig, const char *text, uint8_t str_len);

/**
 * @brief 填充删除图层结构体
 * @param del          删除图层结构体
 * @param delete_type  删除操作类型 UI_DEL_xxx
 * @param layer        图层号
 */
void ui_delete_layer(ui_delete_layer_t *del, uint8_t delete_type, uint8_t layer);

/**
 * @brief 打包并发送单个图形（子内容ID 0x0101）
 * @param fig   图形结构体
 * @param timeout 超时时间
 */
void ui_send_figure_1(const ui_figure_t *fig, uint32_t timeout);

/**
 * @brief 打包并发送2个图形（子内容ID 0x0102）
 */
void ui_send_figure_2(const ui_figure_2_t *figs, uint32_t timeout);

/**
 * @brief 打包并发送5个图形（子内容ID 0x0103）
 */
void ui_send_figure_5(const ui_figure_5_t *figs, uint32_t timeout);

/**
 * @brief 打包并发送7个图形（子内容ID 0x0104）
 */
void ui_send_figure_7(const ui_figure_7_t *figs, uint32_t timeout);

/**
 * @brief 打包并发送字符串图形（子内容ID 0x0110）
 */
void ui_send_string(const ui_string_t *str_fig, uint32_t timeout);

/**
 * @brief 打包并发送删除图层指令（子内容ID 0x0100）
 */
void ui_send_delete_layer(const ui_delete_layer_t *del, uint32_t timeout);

#endif // _REFEREE_PROTOCOL_UI_H_
