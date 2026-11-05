/**
 * @file    lv_port.c
 * @brief   LVGL 9.5 移植层：接到 ILI9488(FSMC 并口) + XPT2046(软件 SPI)。
 *
 * LVGL 只问三件事，这里就是答这三件事：
 *   1. 现在几点   -> lv_tick_set_cb(HAL_GetTick)
 *   2. 像素画到哪 -> disp_flush() 里调 LCD_Color_Fill()
 *   3. 手在哪     -> indev_read() 里读 tp_dev
 *
 * 本文件不创建界面、不改 ui_task，纯粹的驱动对接。
 */

#include "lv_port.h"

#include "stm32f4xx_hal.h"
#include "tftlcd.h"
#include "touch.h"

/* 横屏 480x320，必须和 main.c 里的 LCD_Display_Dir(1) 保持一致。
 * 方向不对的话，flush 的坐标会和实际显存错位，表现为画面横竖颠倒或花屏。 */
#define DISP_W          480u
#define DISP_H          320u

/* 画缓冲行数。6 行 = 480*6*2 = 5.76KB，配合 PARTIAL 模式分块刷新。
 * 调大能减少 flush 次数但吃 RAM；F407 刷 FSMC 很快，6 行足够。 */
#define DISP_BUF_LINES  6u

/* 触摸坐标修正。
 *
 * 24C02 里存的校准系数是在竖屏 320x480 时标定的，而显示现在是横屏 480x320，
 * 所以原始坐标要先换轴才能和显示对上。竖屏 x∈[0,319] y∈[0,479]，
 * 交换后正好是横屏的 x∈[0,479] y∈[0,319]，不需要缩放。
 *
 * 方向不对时按下面的实测结果微调，不用改逻辑：
 *   左上角原始值约 0,0      右下角约 479,319  -> SWAP 改 0
 *   左上角原始值约 0,0      右下角约 319,479  -> SWAP 保持 1
 *   左上角原始值约 319,0 或 0,479            -> 配合 INV_X / INV_Y 翻向 */
#define TOUCH_SWAP_XY   1       /* 1 = 交换 x 与 y */
#define TOUCH_INV_X     0       /* 1 = 左右镜像 */
#define TOUCH_INV_Y     1       /* 1 = 上下镜像。实测按下 y_raw≈44 时报成 44，
                                 *     但手指实际在屏幕底部（应≈275），320-44=276 正好对上，
                                 *     所以竖屏标定的 y 方向和横屏是反的 */

/* 自己判滑动，不用 LVGL 自带的手势检测。
 *
 * 原因：LVGL 的 gesture_sum 是逐帧累加的，只要有一帧位移小于 min_velocity
 * （电阻屏轻按时 TP_Read_XY2 会判采样失败、坐标不更新，此时位移就是 0），
 * 累积量立刻被整体清零，手指滑到头也触发不了。
 *
 * 这里改成：按下时记起点，按住期间跟踪"到达过的最大水平位移"，松手时用峰值判定。
 * 采样失败的帧坐标保持不变，差值不变，所以不会打断判定。
 * 好处是完全不动 touch.c —— ERR_RANGE 仍是原厂 50，坐标精度一分不丢。 */
#define SWIPE_MIN_PX    50      /* 水平位移超过这么多才算一次滑动 */

static lv_coord_t s_press_x  = 0;   /* 本次按下的起点 x */
static lv_coord_t s_max_dx   = 0;   /* 按住期间达到过的最大水平位移，带符号 */
static uint8_t    s_was_down = 0;   /* 上一帧是否是按下状态，用来识别"刚按下""刚松开" */
static uint8_t    s_swipe    = 0;   /* 待取走的滑动结果：0=无 1=左滑 2=右滑 */

static lv_color_t    s_disp_buf[DISP_W * DISP_BUF_LINES];
static lv_display_t *s_disp;
static lv_indev_t   *s_indev;

/* ------------------------------------------------------------------ 显示 */

/**
 * LVGL 画好一块矩形后交给我们，这里整块甩给 FSMC。
 *
 * LCD_Color_Fill 一次写一个矩形区域，比逐点写快一个数量级，
 * 正好匹配 LVGL 的"按块刷"模型 —— 这是整个移植里唯一需要动脑的地方，
 * 剩下的都是填参数。
 *
 * 最后那句 lv_display_flush_ready() 不能漏：LVGL 靠它确认"这块画完了"，
 * 漏掉的话第一屏之后就永远停在那里等，表现为画面冻住。
 */
static void disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    LCD_Color_Fill((u16)area->x1, (u16)area->y1,
                   (u16)area->x2, (u16)area->y2,
                   (u16 *)px_map);

    lv_display_flush_ready(disp);
}

/* ------------------------------------------------------------------ 输入 */

/**
 * LVGL 每 LV_DEF_REFR_PERIOD（33ms）左右问一次"手指在哪、按没按"。
 *
 * tp_dev.scan(0) 是阻塞的（软件 SPI 读 XPT2046，几个毫秒），
 * 所以别在外面再频繁调它，交给 LVGL 的节奏就好。
 *
 * 松手时要回"上一次的坐标"而不是 (0,0)，否则 LVGL 会认为手指瞬移到左上角，
 * 导致按钮的点击区域判断错乱。
 */
static void indev_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    static lv_coord_t last_x = 0;
    static lv_coord_t last_y = 0;
    uint16_t          tx;
    uint16_t          ty;
    lv_coord_t        dx;
    uint8_t           retry;

    tp_dev.scan(0);                              /* 0 = 返回屏幕坐标 */

    /* 电阻屏按下首帧常常采样失败：XPT2046 还没稳定，TP_Read_XY2 连续几次都
     * 判超差就放弃，TP_Scan 于是把 x[0]/y[0] 留成 0xFFFF。
     *
     * 这个脏值一旦送进 LVGL，会被越界钳位成屏幕右下角；而 LVGL 是在"按下那一帧"
     * 锁定命中对象的，后面坐标变正常也不会改投，这一次点击就彻底丢了。
     * 实测大约每三次按下就有一次，所以先重采几下把坐标拿回来。 */
    for (retry = 0; retry < 3u; retry++) {
        if ((tp_dev.sta & TP_PRES_DOWN) == 0) {
            break;                               /* 没按住，不用再采 */
        }
        if (tp_dev.x[0] != 0xFFFFu && tp_dev.y[0] != 0xFFFFu) {
            break;                               /* 已经是有效坐标 */
        }
        tp_dev.scan(0);
    }

    /* ---- 这一帧没有拿到有效按下 ---- */
    if (((tp_dev.sta & TP_PRES_DOWN) == 0) ||
        (tp_dev.x[0] == 0xFFFFu) || (tp_dev.y[0] == 0xFFFFu)) {

        /* 手指还按着，只是这帧采样没成功：沿用上次坐标继续报按下，
         * 否则拖动会被误判成松手，滑动的起点也跟着丢。 */
        if ((tp_dev.sta & TP_PRES_DOWN) && s_was_down) {
            data->state = LV_INDEV_STATE_PRESSED;
            data->point.x = last_x;
            data->point.y = last_y;
            return;
        }

        /* 真松手（或按下第一帧就没采到）：结算滑动后报释放 */
        if (s_was_down) {
            if (LV_ABS(s_max_dx) >= SWIPE_MIN_PX) {
                s_swipe = (s_max_dx > 0) ? 2u : 1u;
            }
            s_was_down = 0;
        }
        data->state = LV_INDEV_STATE_RELEASED;
        data->point.x = last_x;
        data->point.y = last_y;
        return;
    }

    /* ---- 有效按下 ---- */
    tx = tp_dev.x[0];
    ty = tp_dev.y[0];

#if TOUCH_SWAP_XY
    {
        uint16_t tmp = tx;

        tx = ty;
        ty = tmp;
    }
#endif
#if TOUCH_INV_X
    tx = (uint16_t)((DISP_W - 1u) - tx);
#endif
#if TOUCH_INV_Y
    ty = (uint16_t)((DISP_H - 1u) - ty);
#endif

    /* 越界钳位：坐标跑到屏幕外，LVGL 找不到命中对象，手势和点击都会失效 */
    if (tx >= DISP_W) {
        tx = (uint16_t)(DISP_W - 1u);
    }
    if (ty >= DISP_H) {
        ty = (uint16_t)(DISP_H - 1u);
    }

    if (!s_was_down) {                           /* 刚按下这一帧，记起点 */
        s_press_x = (lv_coord_t)tx;
        s_max_dx  = 0;
    }

    /* 全程跟踪相对起点的最大水平位移。用峰值而不是"松手点 - 起点"，
     * 是因为松手前如果连续几帧采样失败，坐标会停在中途，算出来的位移会偏短。 */
    dx = (lv_coord_t)tx - s_press_x;
    if (LV_ABS(dx) > LV_ABS(s_max_dx)) {
        s_max_dx = dx;
    }

    s_was_down = 1;

    last_x = (lv_coord_t)tx;
    last_y = (lv_coord_t)ty;
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = last_x;
    data->point.y = last_y;
}

/**
 * 取走一次滑动结果并清零。
 *
 * 用"取走"而不是"查询"，是为了让 ui_task 每 5ms 轮询时不会重复触发切屏。
 *
 * @return 0 = 本次没有滑动，1 = 往左滑，2 = 往右滑
 */
uint8_t lv_port_take_swipe(void)
{
    uint8_t sw = s_swipe;

    s_swipe = 0;
    return sw;
}

/* -------------------------------------------------------------- 初始化 */

void lv_port_init(void)
{
    lv_init();

    /* FreeRTOS 已经占用了 SysTick，LVGL 不自己计时，直接读 HAL 的毫秒计数。
     * 漏掉这一步的典型症状：界面能画出来，但永远不动、点了没反应。 */
    lv_tick_set_cb(HAL_GetTick);

    /* ---- 显示 ---- */
    s_disp = lv_display_create((int32_t)DISP_W, (int32_t)DISP_H);
    lv_display_set_flush_cb(s_disp, disp_flush);
    lv_display_set_buffers(s_disp, s_disp_buf, NULL,
                           (uint32_t)sizeof(s_disp_buf),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    /* ---- 输入 ---- */
    s_indev = lv_indev_create();
    lv_indev_set_type(s_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(s_indev, indev_read);

    /* XPT2046 一帧位移常常只有 1~2 像素，而手势判定里只要每帧位移小于
     * min_velocity 就会把 gesture_sum 清零（见 indev/lv_indev.c 的 indev_gesture）。
     * 默认值 3 会让手指滑过整个屏幕也攒不够手势，所以降到 1。
     * 距离阈值默认 50，降到 40 让滑动稍微好触发一点。 */
    lv_indev_set_gesture_min_velocity(s_indev, 1);
    lv_indev_set_gesture_min_distance(s_indev, 40);
}
