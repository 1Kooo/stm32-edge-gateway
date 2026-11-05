/**
 * @file    app_ui.c
 * @brief   LVGL 界面：主界面 / WiFi 页 / 切屏手势 / ui_task。
 *
 * 从 app_freertos.c 整体搬来，代码原样；只把数据源从直接读
 * app_freertos.c 的私有变量换成四个只读 getter（见 app_ui.h）。
 * 后续配网弹层（屏上输入 WiFi 账号）也加在这个文件里。
 */

#include <stdio.h>
#include <string.h>      /* strlen：配网弹层校验输入长度用 */

#include "FreeRTOS.h"
#include "task.h"

#include "lv_port.h"         /* lvgl.h 由此带进来 */
#include "key.h"             /* KEY_Scan / KEY1_PRESS / KEY2_PRESS */
#include "touch.h"           /* 触摸坐标调试显示用：tp_dev */
#include "net_if.h"          /* net_if_state_str() */
#include "app_ota.h"         /* app_ota_request() */
#include "app_config.h"      /* APP_VERSION / WIFI_SSID / BEMFA_* */
#include "app_wifi_cfg.h"    /* 配网凭据的存取 */
#include "app_ui.h"

/* 日志函数定义在 app_freertos.c，全项目惯例：用到的文件自己 extern 一句 */
extern void app_log(const char *fmt, ...);

/* 界面统一字体：思源黑体简体 14px，ASCII 和汉字都含在内，中英混排不用切换。
 * 换字号或重做字库只改这一行。
 * 28px 的温度/湿度大数字仍用 Montserrat —— 纯数字它更好看，而且 28px
 * 中文单字要 392 字节，字库收不下也没必要。
 * 键盘、按钮、输入框没显式设字体，走 LV_FONT_DEFAULT（Montserrat 14），
 * 键盘上的 ✓ ← → 图标就靠它提供。 */
#define FONT_UI  (&lv_font_cn_14)

/* ================= UI：LVGL 界面（横屏 480x320） ================= */

/* 界面控件句柄：建一次，之后只改内容，不重建对象 */
static lv_obj_t *s_scr_main;        /* 主界面 */
static lv_obj_t *s_scr_wifi;        /* WiFi 信息页 */
static uint8_t   s_on_wifi = 0;     /* 当前是否停在 WiFi 页 */

static lv_obj_t *s_wifi_state;      /* WiFi 页：网络状态 */
static lv_obj_t *s_wifi_mqtt;       /* WiFi 页：MQTT 状态 */
static lv_obj_t *s_wifi_ssid;       /* WiFi 页：当前生效的 SSID（动态刷新） */
static lv_obj_t *s_wifi_cfg_src;    /* WiFi 页：凭据来源 default / custom */

static lv_obj_t *s_ui_temp;         /* 温度数值 */
static lv_obj_t *s_ui_temp_bar;     /* 温度进度条 */
static lv_obj_t *s_ui_light_pct;    /* 光照百分数 */
static lv_obj_t *s_ui_light_bar;    /* 光照进度条 */
static lv_obj_t *s_ui_net_text;     /* 顶栏右侧：online / offline */
static lv_obj_t *s_ui_seq;          /* 采样序号 */
static lv_obj_t *s_ui_sent;         /* 已上报条数 */
static lv_obj_t *s_ui_backlog;      /* 待补传条数 */
static lv_obj_t *s_ui_uptime;       /* 运行时间 */
static lv_obj_t *s_ui_heap;         /* 剩余堆 */

/* 触摸坐标调试显示：验证坐标变换对不对时打开，确认后改 0 就完全消失。
 * 显示的是变换前的原始值，正好用来判断该开 SWAP / INV 哪个开关。 */
#define UI_SHOW_TOUCH_DEBUG  0
#if UI_SHOW_TOUCH_DEBUG
static lv_obj_t *s_ui_touch;
#endif

/* 建一个标签：字体和颜色统一在这里设，省得每处重复写样式 */
static lv_obj_t *ui_label(lv_obj_t *parent, int32_t x, int32_t y,
                          const lv_font_t *font, lv_color_t color)
{
    lv_obj_t *lb = lv_label_create(parent);

    lv_obj_set_pos(lb, x, y);
    lv_obj_set_style_text_font(lb, font, 0);
    lv_obj_set_style_text_color(lb, color, 0);

    return lb;
}

/* 建一张白底卡片：统一边框、圆角、背景色，省得每处重复设样式 */
static lv_obj_t *ui_card(lv_obj_t *parent, int32_t x, int32_t y,
                         int32_t w, int32_t h)
{
    lv_obj_t *card = lv_obj_create(parent);

    lv_obj_set_size(card, w, h);
    lv_obj_set_pos(card, x, y);
    lv_obj_set_style_bg_color(card, lv_color_white(), 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_border_color(card, lv_color_hex(0xC9D8E8), 0);
    lv_obj_set_style_radius(card, 6, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    /* 卡片上的滑动要能传到屏幕，否则手势判定会停在卡片这一层。
     * lv_obj 构造时对"有父对象的对象"默认就会加 GESTURE_BUBBLE，
     * 这里显式写一遍是为了让意图清楚，也不受库默认值变化影响。
     * 注意用的是 GESTURE_BUBBLE，不是 EVENT_BUBBLE，两个标志不是一回事。 */
    lv_obj_add_flag(card, LV_OBJ_FLAG_GESTURE_BUBBLE);

    return card;
}

/* 建主界面。之后只改文字和数值，不重建对象 */
static void ui_build_main(lv_obj_t *scr)
{
    lv_obj_t *card;
    lv_obj_t *lb;

    /* 浅灰蓝底 + 白卡片，才有层次感；纯白底会糊成一片 */
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xE8EEF5), 0);
    lv_obj_set_style_text_color(scr, lv_color_hex(0x2D3748), 0);

    /* ---- 顶栏：左边标题，右边链路状态 ---- */
    lb = ui_label(scr, 12, 10, FONT_UI, lv_color_hex(0x2D3748));
    lv_label_set_text(lb, "GW " APP_VERSION);

    s_ui_net_text = ui_label(scr, 386, 10, FONT_UI, lv_color_hex(0x999999));
    lv_label_set_text(s_ui_net_text, "离线");

    /* ---- 左卡片：温度 ---- */
    card = ui_card(scr, 10, 44, 220, 140);

    lb = ui_label(card, 12, 8, FONT_UI, lv_color_hex(0xE53E3E));
    lv_label_set_text(lb, "温度");

    /* 大数字仍用 Montserrat 28：纯数字比中文好看，且省字库空间。
     * 28 号字体的 cmap 里有 °（U+00B0）没有 ℃（U+2103），
     * 所以单位写 ° 而不是 ℃，否则这里会出现方块。 */
    s_ui_temp = ui_label(card, 12, 32, &lv_font_montserrat_28, lv_color_hex(0xE53E3E));
    lv_label_set_text(s_ui_temp, "--.-°");

    s_ui_temp_bar = lv_bar_create(card);
    lv_obj_set_size(s_ui_temp_bar, 196, 12);
    lv_obj_set_pos(s_ui_temp_bar, 12, 112);
    lv_bar_set_range(s_ui_temp_bar, -100, 600);          /* -10.0 ~ 60.0 摄氏度 */
    lv_bar_set_value(s_ui_temp_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_ui_temp_bar, lv_color_hex(0xE2E8F0), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_ui_temp_bar, lv_color_hex(0xE53E3E), LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_ui_temp_bar, 4, 0);

    /* ---- 右卡片：光照 ---- */
    card = ui_card(scr, 250, 44, 220, 140);

    lb = ui_label(card, 12, 8, FONT_UI, lv_color_hex(0x319795));
    lv_label_set_text(lb, "光照");

    s_ui_light_pct = ui_label(card, 12, 32, &lv_font_montserrat_28, lv_color_hex(0x319795));
    lv_label_set_text(s_ui_light_pct, "0 %");

    s_ui_light_bar = lv_bar_create(card);
    lv_obj_set_size(s_ui_light_bar, 196, 12);
    lv_obj_set_pos(s_ui_light_bar, 12, 112);
    lv_bar_set_range(s_ui_light_bar, 0, 100);
    lv_bar_set_value(s_ui_light_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_ui_light_bar, lv_color_hex(0xE2E8F0), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_ui_light_bar, lv_color_hex(0x319795), LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_ui_light_bar, 4, 0);

    /* ---- 统计条：三列横排 ---- */
    card = ui_card(scr, 10, 194, 460, 48);

    s_ui_seq = ui_label(card, 14, 16, FONT_UI, lv_color_hex(0x2D3748));
    lv_label_set_text(s_ui_seq, "序号 0");

    s_ui_sent = ui_label(card, 170, 16, FONT_UI, lv_color_hex(0x2D3748));
    lv_label_set_text(s_ui_sent, "已发 0");

    s_ui_backlog = ui_label(card, 330, 16, FONT_UI, lv_color_hex(0x2D3748));
    lv_label_set_text(s_ui_backlog, "待发 0");

    /* ---- 底栏：深色条，视觉上收口 ---- */
    card = lv_obj_create(scr);
    lv_obj_set_size(card, 480, 58);
    lv_obj_set_pos(card, 0, 262);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x1A2233), 0);
    lv_obj_set_style_border_width(card, 0, 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    s_ui_uptime = ui_label(card, 14, 20, FONT_UI, lv_color_hex(0xC9D8E8));
    lv_label_set_text(s_ui_uptime, "运行 00:00:00");

    s_ui_heap = ui_label(card, 300, 20, FONT_UI, lv_color_hex(0xC9D8E8));
    lv_label_set_text(s_ui_heap, "内存 0 B");

#if UI_SHOW_TOUCH_DEBUG
    s_ui_touch = ui_label(card, 180, 20, FONT_UI, lv_color_hex(0x8899BB));
    lv_label_set_text(s_ui_touch, "T ----,----");
#endif
}

/* ================= WiFi 配网弹层 =================
 *
 * 点 WiFi 页的 CONFIG 按钮弹出：遮罩 + 面板（SSID/PASS 两个输入框）
 * + 键盘。点哪个框键盘就往哪个框打字，OK 保存、CANCEL 放弃。
 *
 * 凭据的走向（本文件只管前半段）：
 *   ui_cfg_ok_cb → wifi_cfg_request（只写 RAM 待写缓冲）
 *   net_task     → wifi_cfg_save（擦写外部 Flash）+ net_if_set_wifi + 重连
 * 界面代码不碰 Flash，原因见 app_wifi_cfg.h 的线程约束。 */
static lv_obj_t *s_cfg_layer;      /* 弹层根对象：NULL = 没开 */
static lv_obj_t *s_cfg_ta_ssid;    /* SSID 输入框 */
static lv_obj_t *s_cfg_ta_pass;    /* 密码输入框 */
static lv_obj_t *s_cfg_hint;       /* 底部提示行：校验失败原因写这里 */

/* 扫描选网列表。挂在配网弹层上面（s_cfg_layer 的子对象，弹层一关跟着没）。
 * 结果拷贝在 UI 侧而不是直接用 net_task 的缓冲：列表行的 user_data
 * 指向这些字符串，net_task 重新扫描覆写时不能把行上的指针拆了。 */
static lv_obj_t *s_scan_layer;         /* 列表层根对象：NULL = 没开 */
static lv_obj_t *s_scan_status;        /* "扫描中..." / 结果提示 */
static uint8_t   s_scan_pending_ui;    /* 已发起扫描、还没拿到结果 */
static char      s_scan_ssid_ui[8][33];
static int8_t    s_scan_rssi_ui[8];

/* 当前生效的 SSID / 凭据来源：
 * 直接问 wifi_cfg —— 它开机时已经把 Flash 里的记录读进 RAM，
 * 这里每次刷新只是读内存，不碰 SPI。 */
static const char *ui_wifi_ssid(void)
{
    return wifi_cfg_ssid();
}

static const char *ui_wifi_cfg_src(void)
{
    return wifi_cfg_is_custom() ? "自定义" : "默认";
}

/* 网络状态码转中文。
 *
 * net_if_state_str() 返回的是协议层的 "down"/"ready"/"connecting"，
 * 属于 port 层的对外状态码，不该在里面塞界面文案 —— 别的调用方
 * （比如日志）还要用它做判断。所以转译放在 UI 这一层做。 */
static const char *ui_net_state_cn(void)
{
    const char *s = net_if_state_str();

    if (strcmp(s, "ready") == 0) {
        return "已就绪";
    }
    if (strcmp(s, "connecting") == 0) {
        return "连接中";
    }
    return "断开";
}

/* ---------- 弹层：关闭 ----------
 * 删根对象即可，子对象（文本框/键盘/按钮）跟着一起释放。
 * 句柄必须同步清空，否则按键屏蔽那几处还在拿旧指针比较。 */
static void ui_cfg_close(void)
{
    if (s_cfg_layer == NULL) {
        return;
    }
    lv_obj_delete(s_cfg_layer);
    s_cfg_layer    = NULL;
    s_cfg_ta_ssid  = NULL;
    s_cfg_ta_pass  = NULL;
    s_cfg_hint     = NULL;
    /* 扫描列表层是弹层的子对象，删弹层时跟着没了，但句柄要在这里清，
     * 否则拿着野指针判断"层开没开"必出事。 */
    s_scan_layer      = NULL;
    s_scan_status     = NULL;
    s_scan_pending_ui = 0;
}

static void ui_cfg_cancel_cb(lv_event_t *e)
{
    (void)e;
    ui_cfg_close();
}

/* ---------- 弹层：OK ----------
 * 先校验再动作：SSID 空 / 超长都不关弹层，把原因写在提示行上，
 * 用户改完再按一次 OK。 */
static void ui_cfg_ok_cb(lv_event_t *e)
{
    const char *ssid;
    const char *pass;

    (void)e;
    ssid = lv_textarea_get_text(s_cfg_ta_ssid);
    pass = lv_textarea_get_text(s_cfg_ta_pass);

    if (ssid[0] == '\0') {
        lv_label_set_text(s_cfg_hint, "SSID 不能为空");
        return;
    }
    if (strlen(ssid) > 32u) {
        lv_label_set_text(s_cfg_hint, "SSID 最多 32 字");
        return;
    }
    if (strlen(pass) > 64u) {
        lv_label_set_text(s_cfg_hint, "密码最多 64 位");
        return;
    }

    /* 只受理不落盘：真正擦 Flash 的是 net_task（见 app_freertos.c），
     * 这里返回 0 就说明凭据已经进了待写缓冲，net_task 下一轮会断链、
     * 存盘、再用新凭据重连。
     * 页面上的 SSID 和 cfg 来源不用在这里改 —— 下面 ui_task 的周期刷新
     * 调的就是 ui_wifi_ssid() / ui_wifi_cfg_src()，存好之后自动变。 */
    if (wifi_cfg_request(ssid, pass) != 0) {
        lv_label_set_text(s_cfg_hint, "保存失败");
        return;
    }
    app_log("[ui] wifi cfg: ssid='%s' pass_len=%u\r\n",
            ssid, (unsigned)strlen(pass));

    ui_cfg_close();
}

/* ---------- 弹层：输入框焦点切换 ----------
 * 键盘同时只往一个框打字。点哪个框，就把键盘目标切过去。
 * user_data 塞的是键盘句柄，省一个全局。 */
static void ui_cfg_ta_focus_cb(lv_event_t *e)
{
    lv_obj_t *ta = lv_event_get_target(e);
    lv_obj_t *kb = lv_event_get_user_data(e);

    lv_keyboard_set_textarea(kb, ta);
}

/* ---------- 弹层：密码显/隐切换 ---------- */
static void ui_cfg_eye_cb(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_t *ta  = lv_event_get_user_data(e);

    /* lv_button 的第 0 个子对象就是它自己的 label */
    if (lv_textarea_get_password_mode(ta)) {
        lv_textarea_set_password_mode(ta, false);
        lv_label_set_text(lv_obj_get_child(btn, 0), "隐藏");
    } else {
        lv_textarea_set_password_mode(ta, true);
        lv_label_set_text(lv_obj_get_child(btn, 0), "显示");
    }
}

/* ---------- 扫描列表层 ----------
 * 挂在弹层上（子对象，弹层关它也关）。扫描请求发给 net_task，
 * 结果轮询回来后建行；点行把 SSID 填回输入框。 */

static void ui_scan_close(void)
{
    if (s_scan_layer == NULL) {
        return;
    }
    lv_obj_delete(s_scan_layer);
    s_scan_layer  = NULL;
    s_scan_status = NULL;
}

/* 返回按钮：什么都不选，回配网弹层继续手输 */
static void ui_scan_back_cb(lv_event_t *e)
{
    (void)e;
    ui_scan_close();
}

/* 点了列表里的一条：SSID 填进输入框，关列表回弹层。
 * user_data 是 s_scan_ssid_ui 里那一条的指针。 */
static void ui_scan_pick_cb(lv_event_t *e)
{
    const char *ssid = lv_event_get_user_data(e);

    lv_textarea_set_text(s_cfg_ta_ssid, ssid);
    /* 填完焦点挪到密码框：SSID 已经有了，下一步自然是打密码 */
    ui_scan_close();
}

/* 扫描结果到了（ui_task 轮询发现 state==2）才建行。
 * 扫描发起时用户在屏上看到的是"扫描中..."。 */
static void ui_scan_build_rows(void)
{
    int      cnt;
    int      i;
    char     buf[48];
    lv_obj_t *btn;
    lv_obj_t *lb;

    if (s_scan_layer == NULL) {
        return;                          /* 等结果期间用户按了返回 */
    }

    cnt = app_wifi_scan_get(s_scan_ssid_ui, s_scan_rssi_ui, 8);

    if (cnt <= 0) {
        lv_label_set_text(s_scan_status, (cnt == 0) ? "无网络" : "扫描失败");
        return;
    }

    for (i = 0; i < cnt; i++) {
        btn = lv_button_create(s_scan_layer);
        lv_obj_set_pos(btn, 10, 44 + i * 36);
        lv_obj_set_size(btn, 460, 32);
        lv_obj_set_style_bg_color(btn, lv_color_white(), 0);
        lv_obj_set_style_border_color(btn, lv_color_hex(0xC9D8E8), 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_radius(btn, 5, 0);
        lv_obj_set_style_pad_all(btn, 2, 0);

        lb = lv_label_create(btn);
        lv_obj_set_pos(lb, 8, 9);
        lv_obj_set_width(lb, 340);
        /* 名字太长时中间打点省略，不折行（行高 32 放不下两行字） */
        lv_label_set_long_mode(lb, LV_LABEL_LONG_DOT);
        lv_label_set_text(lb, s_scan_ssid_ui[i]);
        lv_obj_set_style_text_font(lb, FONT_UI, 0);      /* 中文网络名 */
        lv_obj_set_style_text_color(lb, lv_color_hex(0x2D3748), 0);

        lb = lv_label_create(btn);
        lv_obj_set_pos(lb, 356, 9);
        sprintf(buf, "%d dBm", (int)s_scan_rssi_ui[i]);
        lv_label_set_text(lb, buf);
        lv_obj_set_style_text_font(lb, FONT_UI, 0);
        lv_obj_set_style_text_color(lb, lv_color_hex(0x999999), 0);

        lv_obj_add_event_cb(btn, ui_scan_pick_cb, LV_EVENT_CLICKED,
                            s_scan_ssid_ui[i]);
    }

    sprintf(buf, "找到 %d 个网络", cnt);
    lv_label_set_text(s_scan_status, buf);
}

/* 配网弹层里按「扫描」：发请求 + 建列表层。
 * 层挂在 s_cfg_layer 下面（后建的子对象画在上面，天然盖住弹层）。 */
static void ui_scan_open_cb(lv_event_t *e)
{
    lv_obj_t *btn;
    lv_obj_t *lb;

    (void)e;
    if (s_scan_layer != NULL) {
        return;                          /* 已经开着 */
    }

    app_wifi_scan_request();
    s_scan_pending_ui = 1;

    /* 全屏白底：这层要滚动，半透明会看串下面的行 */
    s_scan_layer = lv_obj_create(s_cfg_layer);
    lv_obj_set_pos(s_scan_layer, 0, 0);
    lv_obj_set_size(s_scan_layer, 480, 320);
    lv_obj_set_style_bg_color(s_scan_layer, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(s_scan_layer, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_scan_layer, 0, 0);
    lv_obj_set_style_radius(s_scan_layer, 0, 0);
    lv_obj_set_style_pad_all(s_scan_layer, 0, 0);
    /* 故意保留 SCROLLABLE：网络多于 7 个时列表要能往下滚 */

    lb = ui_label(s_scan_layer, 16, 12, FONT_UI, lv_color_hex(0x2D3748));
    lv_label_set_text(lb, "选择网络");

    s_scan_status = ui_label(s_scan_layer, 120, 12, FONT_UI,
                             lv_color_hex(0x999999));
    lv_label_set_text(s_scan_status, "扫描中...");

    btn = lv_button_create(s_scan_layer);
    lv_obj_set_pos(btn, 388, 6);
    lv_obj_set_size(btn, 82, 28);
    lv_obj_set_style_bg_color(btn, lv_color_white(), 0);
    lv_obj_set_style_border_color(btn, lv_color_hex(0xB4B2A9), 0);
    lv_obj_set_style_border_width(btn, 1, 0);
    lv_obj_set_style_radius(btn, 5, 0);
    lv_obj_set_style_pad_all(btn, 2, 0);
    lb = lv_label_create(btn);
    lv_label_set_text(lb, "返回");
    lv_obj_set_style_text_font(lb, FONT_UI, 0);          /* 按钮 label 要自己设字体 */
    lv_obj_set_style_text_color(lb, lv_color_hex(0x2D3748), 0);
    lv_obj_center(lb);
    lv_obj_add_event_cb(btn, ui_scan_back_cb, LV_EVENT_CLICKED, NULL);
}

/* ---------- 弹层：构建 ----------
 * 弹层挂在 WiFi 页上（CONFIG 按钮只出现在这页），全屏 480x320。
 * 上半是白色面板（标题 + 两个输入框 + 按钮），下半是键盘。 */
static void ui_cfg_open(void)
{
    lv_obj_t *panel;
    lv_obj_t *kb;
    lv_obj_t *btn;
    lv_obj_t *lb;
    lv_obj_t *ta;

    if (s_cfg_layer != NULL) {
        return;                                   /* 已经开着，别重复建 */
    }
    app_log("[cfg] open\r\n");

    /* 遮罩：半透明深色，盖住下面的 WiFi 页。
     * 加 CLICKABLE 吃掉点透；清 GESTURE_BUBBLE 挡住滑动冒泡，
     * 弹层开着时滑动不能切屏。 */
    s_cfg_layer = lv_obj_create(s_scr_wifi);
    lv_obj_set_pos(s_cfg_layer, 0, 0);
    lv_obj_set_size(s_cfg_layer, 480, 320);
    lv_obj_add_flag(s_cfg_layer, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(s_cfg_layer, lv_color_hex(0x2D3748), 0);
    lv_obj_set_style_bg_opa(s_cfg_layer, LV_OPA_40, 0);
    lv_obj_set_style_border_width(s_cfg_layer, 0, 0);
    lv_obj_set_style_radius(s_cfg_layer, 0, 0);
    /* lv_obj_create 默认套用主题的 card 样式，自带 16px 内边距。
     * 不清掉的话，所有子对象的实际位置都会比设定值偏右下 16px，
     * 键盘底边会被推出屏幕外裁掉。 */
    lv_obj_set_style_pad_all(s_cfg_layer, 0, 0);
    lv_obj_clear_flag(s_cfg_layer, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_cfg_layer, LV_OBJ_FLAG_GESTURE_BUBBLE);

    /* 键盘先建：输入框的焦点回调要用它当 user_data。
     *
     * 注意这里必须先清掉 align：lv_keyboard 的构造函数内部做了
     * lv_obj_align(obj, LV_ALIGN_BOTTOM_MID, 0, 0)，而 align 的优先级高于
     * set_pos（set_pos 只是往本地样式写 X/Y，有 align 时布局根本不看 X/Y）。
     * 两个同时存在时，最终位置 = 贴底对齐再叠加 (10,168) 的偏移，
     * 算出来 y = 320 - 150 + 168 = 338，整个键盘被顶到屏幕外，看着就是“没显示”。
     * 清成 LV_ALIGN_DEFAULT 之后，set_pos 才算数。 */
    kb = lv_keyboard_create(s_cfg_layer);
    lv_obj_set_style_align(kb, LV_ALIGN_DEFAULT, 0);
    lv_obj_set_pos(kb, 10, 168);
    lv_obj_set_size(kb, 460, 150);

    /* 白底不透明：键盘下面的遮罩是半透明的，不铺一层白底会透出 WiFi 页的内容 */
    lv_obj_set_style_bg_color(kb, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(kb, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(kb, 0, 0);

    /* 白色面板：标题 + 两行输入 + 按钮 + 提示行 */
    panel = lv_obj_create(s_cfg_layer);
    lv_obj_set_pos(panel, 10, 10);
    lv_obj_set_size(panel, 460, 150);
    lv_obj_set_style_bg_color(panel, lv_color_white(), 0);
    lv_obj_set_style_border_color(panel, lv_color_hex(0xC9D8E8), 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_radius(panel, 8, 0);
    lv_obj_set_style_pad_all(panel, 0, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    lb = ui_label(panel, 16, 8, FONT_UI, lv_color_hex(0x2D3748));
    lv_label_set_text(lb, "WIFI 配网");

    /* 扫描选网：列出周围 WiFi 点选，不用手打名字（中文网络名也能选）。
     * 标题只占 ~90px，右上这块是空的。 */
    btn = lv_button_create(panel);
    lv_obj_set_pos(btn, 374, 6);
    lv_obj_set_size(btn, 76, 24);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0xE8EEF5), 0);
    lv_obj_set_style_radius(btn, 5, 0);
    lv_obj_set_style_pad_all(btn, 2, 0);
    lb = lv_label_create(btn);
    lv_label_set_text(lb, "扫描");
    lv_obj_set_style_text_font(lb, FONT_UI, 0);          /* 按钮 label 要自己设字体 */
    lv_obj_set_style_text_color(lb, lv_color_hex(0x2D3748), 0);
    lv_obj_center(lb);
    lv_obj_add_event_cb(btn, ui_scan_open_cb, LV_EVENT_CLICKED, NULL);

    /* 第一行：SSID。单行输入，上限 32（和 wifi_cfg_t 的缓冲一致） */
    lb = ui_label(panel, 16, 38, FONT_UI, lv_color_hex(0x777777));
    lv_label_set_text(lb, "SSID");
    ta = lv_textarea_create(panel);
    lv_obj_set_pos(ta, 78, 32);
    lv_obj_set_size(ta, 300, 28);
    /* 输入框单独设中文字体：SSID 允许中文，用默认的 Montserrat 只会显示方块。
     * 这个字体 ASCII 也含在内，所以英文 SSID 一样能打。 */
    lv_obj_set_style_text_font(ta, FONT_UI, 0);
    lv_textarea_set_one_line(ta, true);
    lv_textarea_set_max_length(ta, 32);
    lv_obj_add_event_cb(ta, ui_cfg_ta_focus_cb, LV_EVENT_FOCUSED, kb);
    s_cfg_ta_ssid = ta;

    /* 第二行：密码。默认圆点显示，上限 64；空 = 开放网络（允许） */
    lb = ui_label(panel, 16, 74, FONT_UI, lv_color_hex(0x777777));
    lv_label_set_text(lb, "密码");
    ta = lv_textarea_create(panel);
    lv_obj_set_pos(ta, 78, 68);
    lv_obj_set_size(ta, 300, 28);
    lv_obj_set_style_text_font(ta, FONT_UI, 0);   /* 同上，中文密码也能显示 */
    lv_textarea_set_one_line(ta, true);
    lv_textarea_set_max_length(ta, 64);
    lv_textarea_set_password_mode(ta, true);
    lv_obj_add_event_cb(ta, ui_cfg_ta_focus_cb, LV_EVENT_FOCUSED, kb);
    s_cfg_ta_pass = ta;

    /* 密码显/隐小按钮：放密码框右边。
     * 宽度要够：主题给按钮的 pad_hor 是 16px，56 宽只剩 24px 可用，
     * "显示"(2 个汉字 28px) 会折行、超出高度后被整块裁掉 —— 看着就是个空框。
     * 60 宽配 pad 2 后可用 56px，够放。 */
    btn = lv_button_create(panel);
    lv_obj_set_pos(btn, 380, 68);
    lv_obj_set_size(btn, 60, 28);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0xE8EEF5), 0);
    lv_obj_set_style_radius(btn, 5, 0);
    lv_obj_set_style_pad_all(btn, 2, 0);              /* 收掉默认 16px 内边距 */
    lb = lv_label_create(btn);
    lv_label_set_text(lb, "显示");
    /* 按钮内的 label 是单独创建的对象，不会自动拿到 ui_label 里的字体。
     * 不显式设置就走屏幕默认的 Montserrat，中文会变成方块。 */
    lv_obj_set_style_text_font(lb, FONT_UI, 0);
    lv_obj_set_style_text_color(lb, lv_color_hex(0x2D3748), 0);
    lv_obj_center(lb);
    lv_obj_add_event_cb(btn, ui_cfg_eye_cb, LV_EVENT_CLICKED, s_cfg_ta_pass);

    /* 提示行（左）+ CANCEL / OK（右）。校验失败的原因写提示行 */
    s_cfg_hint = ui_label(panel, 16, 116, FONT_UI,
                          lv_color_hex(0xDD3333));
    lv_label_set_text(s_cfg_hint, "");

    /* 取消按钮：78 宽配 pad 2，放得下 2 个汉字（28px），不会折行 */
    btn = lv_button_create(panel);
    lv_obj_set_pos(btn, 304, 110);
    lv_obj_set_size(btn, 78, 30);
    lv_obj_set_style_bg_color(btn, lv_color_white(), 0);
    lv_obj_set_style_border_color(btn, lv_color_hex(0xB4B2A9), 0);
    lv_obj_set_style_border_width(btn, 1, 0);
    lv_obj_set_style_radius(btn, 5, 0);
    lv_obj_set_style_pad_all(btn, 2, 0);
    lb = lv_label_create(btn);
    lv_label_set_text(lb, "取消");
    lv_obj_set_style_text_font(lb, FONT_UI, 0);      /* 同上：按钮 label 要自己设字体 */
    lv_obj_set_style_text_color(lb, lv_color_hex(0x2D3748), 0);
    lv_obj_center(lb);
    lv_obj_add_event_cb(btn, ui_cfg_cancel_cb, LV_EVENT_CLICKED, NULL);

    btn = lv_button_create(panel);
    lv_obj_set_pos(btn, 390, 110);
    lv_obj_set_size(btn, 52, 30);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x0F6E56), 0);
    lv_obj_set_style_radius(btn, 5, 0);
    lv_obj_set_style_pad_all(btn, 2, 0);
    lb = lv_label_create(btn);
    lv_label_set_text(lb, "确定");
    lv_obj_set_style_text_font(lb, FONT_UI, 0);      /* 同上 */
    lv_obj_set_style_text_color(lb, lv_color_white(), 0);   /* 绿底配白字 */
    lv_obj_center(lb);
    lv_obj_add_event_cb(btn, ui_cfg_ok_cb, LV_EVENT_CLICKED, NULL);

    /* 默认焦点在 SSID 框：键盘一开就能打字 */
    lv_keyboard_set_textarea(kb, s_cfg_ta_ssid);
}

static void ui_cfg_open_cb(lv_event_t *e)
{
    (void)e;
    ui_cfg_open();
}

/* WiFi 信息页：展示连接信息，底部 CONFIG 按钮弹出配网键盘。 */
static void ui_build_wifi(lv_obj_t *scr)
{
    lv_obj_t *card;
    lv_obj_t *lb;
    lv_obj_t *btn;

    lv_obj_set_style_bg_color(scr, lv_color_hex(0xE8EEF5), 0);
    lv_obj_set_style_text_color(scr, lv_color_hex(0x2D3748), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    /* ---- 顶栏 ---- */
    lb = ui_label(scr, 12, 10, FONT_UI, lv_color_hex(0x2D3748));
    lv_label_set_text(lb, "WIFI");

    lb = ui_label(scr, 336, 10, FONT_UI, lv_color_hex(0x777777));
    lv_label_set_text(lb, "KEY2 返回");

    /* ---- 连接信息卡片 ---- */
    card = ui_card(scr, 10, 44, 460, 190);

    lb = ui_label(card, 16, 14, FONT_UI, lv_color_hex(0x777777));
    lv_label_set_text(lb, "SSID");
    s_wifi_ssid = ui_label(card, 150, 14, FONT_UI,
                           lv_color_hex(0x2D3748));
    lv_label_set_text(s_wifi_ssid, ui_wifi_ssid());

    lb = ui_label(card, 16, 46, FONT_UI, lv_color_hex(0x777777));
    lv_label_set_text(lb, "NET");
    s_wifi_state = ui_label(card, 150, 46, FONT_UI, lv_color_hex(0x2D3748));
    lv_label_set_text(s_wifi_state, ui_net_state_cn());

    lb = ui_label(card, 16, 78, FONT_UI, lv_color_hex(0x777777));
    lv_label_set_text(lb, "MQTT");
    s_wifi_mqtt = ui_label(card, 150, 78, FONT_UI, lv_color_hex(0x2D3748));
    lv_label_set_text(s_wifi_mqtt, "离线");

    /* 分隔线用一串短横，比 lv_line 省事 */
    lb = ui_label(card, 16, 108, FONT_UI, lv_color_hex(0xD0D8E4));
    lv_label_set_text(lb, "-------------------------------------------");

    lb = ui_label(card, 16, 122, FONT_UI, lv_color_hex(0x777777));
    lv_label_set_text(lb, "HOST");
    lb = ui_label(card, 150, 122, FONT_UI, lv_color_hex(0x2D3748));
    lv_label_set_text(lb, BEMFA_HOST);

    lb = ui_label(card, 16, 154, FONT_UI, lv_color_hex(0x777777));
    lv_label_set_text(lb, "TOPIC");
    lb = ui_label(card, 150, 154, FONT_UI, lv_color_hex(0x2D3748));
    lv_label_set_text(lb, BEMFA_TOPIC);

    /* ---- 操作卡片：凭据来源 + CONFIG 按钮 ---- */
    card = ui_card(scr, 10, 244, 460, 62);

    lb = ui_label(card, 16, 10, FONT_UI, lv_color_hex(0x777777));
    lv_label_set_text(lb, "配置:");
    /* 值标签从 50 挪到 62：中文"配置:"比 "cfg:" 宽约 20px，
     * 留在 50 会和这三个字叠在一起。 */
    s_wifi_cfg_src = ui_label(card, 62, 10, FONT_UI,
                              lv_color_hex(0x0F6E56));
    lv_label_set_text(s_wifi_cfg_src, ui_wifi_cfg_src());

    lb = ui_label(card, 16, 34, FONT_UI, lv_color_hex(0x999999));
    lv_label_set_text(lb, "已存外部Flash，断电不丢");

    /* CONFIG 按钮：弹出配网键盘 */
    btn = lv_button_create(card);
    lv_obj_set_pos(btn, 360, 16);
    lv_obj_set_size(btn, 86, 30);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x185FA5), 0);
    lv_obj_set_style_radius(btn, 5, 0);
    lb = lv_label_create(btn);
    lv_label_set_text(lb, "设置");
    lv_obj_set_style_text_font(lb, FONT_UI, 0);      /* 同上 */
    lv_obj_center(lb);
    lv_obj_add_event_cb(btn, ui_cfg_open_cb, LV_EVENT_CLICKED, NULL);
}

static uint32_t s_last_sw = 0;      /* 上次切屏的时刻，用来防连滑 */

/* 统一的切屏入口：KEY2 和触摸手势都走这里，避免两处各写一套 */
static void ui_switch_to(uint8_t to_wifi)
{
    if (s_on_wifi == to_wifi) {
        return;                                                     /* 已经在目标页 */
    }
    if ((xTaskGetTickCount() - s_last_sw) < pdMS_TO_TICKS(300)) {
        return;                                                     /* 动画还没走完，忽略连滑 */
    }

    s_last_sw = xTaskGetTickCount();
    s_on_wifi = to_wifi;

    if (to_wifi) {
        lv_screen_load_anim(s_scr_wifi, LV_SCREEN_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    } else {
        lv_screen_load_anim(s_scr_main, LV_SCREEN_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
    }
}

/* 手势回调：两个屏幕都注册，按滑动方向切页。
 * 方向语义是"手指往哪滑"：往左滑 = 想看右边的内容 = 进 WiFi 页。 */
static void ui_gesture_cb(lv_event_t *e)
{
    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());

    if (dir == LV_DIR_LEFT) {
        ui_switch_to(1);
    } else if (dir == LV_DIR_RIGHT) {
        ui_switch_to(0);
    }
}

/* 建两个屏幕：上电先显示主界面 */
static void ui_create(void)
{
    s_scr_main = lv_obj_create(NULL);
    s_scr_wifi = lv_obj_create(NULL);

    /* 屏幕必须清掉 SCROLLABLE。lv_obj 构造时默认会给所有对象加这个标志，
     * 而手势判定开头就是 "有对象在滚动就直接返回"，屏幕一旦可滚动，
     * 手指在空白处划动会被当成滚动，手势永远触发不了。 */
    lv_obj_clear_flag(s_scr_main, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_scr_wifi, LV_OBJ_FLAG_SCROLLABLE);

    ui_build_main(s_scr_main);
    ui_build_wifi(s_scr_wifi);

    /* 屏幕本身也要能收手势：手指按在空白处时，命中的对象就是 screen */
    lv_obj_add_event_cb(s_scr_main, ui_gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_add_event_cb(s_scr_wifi, ui_gesture_cb, LV_EVENT_GESTURE, NULL);

    lv_screen_load(s_scr_main);
}

/* LVGL 每 5ms 处理一次（刷新/触摸/动画）；
 * 数据源变化慢，每 100 次循环（约 500ms）才更新一次文字。 */
void ui_task(void *args)
{
    ui_snapshot_t snap;
    char          buf[48];
    uint32_t      last_upd = 0;
    uint32_t      up_s;
    uint8_t       key_cnt = 0;
    (void)args;

    ui_create();

    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(5));

        /* 每 4 次循环（约 20ms）扫一次按键。
         * KEY2 = 主界面 / WiFi 页切换，KEY1 = 请求 OTA（8.2c 会换成界面按钮） */
        /* 弹层开着时物理键全部让位：KEY1 会触发 OTA、KEY2 会切屏，
         * 两者都会把弹层晾在半路。关掉弹层再恢复。 */
        if (s_cfg_layer == NULL && ++key_cnt >= 4u) {
            uint8_t k;

            key_cnt = 0;
            k = KEY_Scan(0);

            if (k == KEY2_PRESS) {
                ui_switch_to((uint8_t)(!s_on_wifi));
            } else if (k == KEY1_PRESS) {
                /* 这里只置标志，真正的下载在 net_task 里做。
                 * 绝不能直接调 app_ota_run —— 那是要阻塞一两分钟的操作，
                 * ui 任务一卡住，屏幕就冻在最后一帧，还会触发看门狗式的假死。 */
                app_log("[ui] KEY1 -> OTA request\r\n");
                app_ota_request();
            }
        }

        /* 取走移植层判出来的滑动。LVGL 自带的 gesture 偶尔也能触发，
         * 两条路并存没关系：ui_switch_to 里已经在目标页就直接 return。
         * 弹层开着时不响应滑动切屏。 */
        if (s_cfg_layer == NULL) {
            uint8_t sw = lv_port_take_swipe();

            if (sw == 1u) {
                ui_switch_to(1);            /* 往左滑 → 进 WiFi 页 */
            } else if (sw == 2u) {
                ui_switch_to(0);            /* 往右滑 → 回主界面 */
            }
        }

        /* 扫描结果轮询：扫描由 net_task 执行（3~6 秒），完成就建列表行。
         * 放在 500ms 刷新闸的外面 —— 列表越早出现越好。 */
        if (s_scan_pending_ui && app_wifi_scan_state() == 2u) {
            s_scan_pending_ui = 0;
            ui_scan_build_rows();
        }

        /* 按真实时间判断，不受 lv_timer_handler 耗时影响。
         * 之前用循环次数算（100 次 × 5ms），但首帧刷全屏要几百毫秒，
         * 100 次循环可能变成几十秒，数据迟迟不更新。 */
        if ((xTaskGetTickCount() - last_upd) < pdMS_TO_TICKS(500)) {
            continue;
        }
        last_upd = xTaskGetTickCount();

        /* 原来直接在临界区里拷 s_latest（app_freertos.c 的私有变量），
         * 拆文件后走 getter —— 临界区逻辑原样，只是挪进了对方那边。 */
        app_data_get_snapshot(&snap);

        /* 温度：数值和进度条一起更新 */
        if (snap.temp_x10 == -999) {
            lv_label_set_text(s_ui_temp, "--.-°");
            lv_bar_set_value(s_ui_temp_bar, -100, LV_ANIM_OFF);
        } else {
            sprintf(buf, "%d.%d°", snap.temp_x10 / 10, snap.temp_x10 % 10);
            lv_label_set_text(s_ui_temp, buf);
            lv_bar_set_value(s_ui_temp_bar, snap.temp_x10, LV_ANIM_OFF);
        }

        /* 光照 */
        lv_bar_set_value(s_ui_light_bar, snap.light_pct, LV_ANIM_OFF);
        sprintf(buf, "%u %%", (unsigned)snap.light_pct);
        lv_label_set_text(s_ui_light_pct, buf);

        /* 采样序号 / 已上报 / 待补传 */
        sprintf(buf, "序号 %lu", (unsigned long)snap.seq);
        lv_label_set_text(s_ui_seq, buf);

        sprintf(buf, "已发 %lu", (unsigned long)app_net_pub_cnt());
        lv_label_set_text(s_ui_sent, buf);

        sprintf(buf, "待发 %lu", (unsigned long)app_store_log_pending());
        lv_label_set_text(s_ui_backlog, buf);

        /* 链路状态：有 MQTT 就绿，否则红 */
        if (app_net_mqtt_up()) {
            lv_label_set_text(s_ui_net_text, "在线");
            lv_obj_set_style_text_color(s_ui_net_text, lv_color_hex(0x22AA22), 0);
        } else {
            lv_label_set_text(s_ui_net_text, "离线");
            lv_obj_set_style_text_color(s_ui_net_text, lv_color_hex(0xDD3333), 0);
        }

        /* 运行时间 秒 -> 时:分:秒 */
        up_s = (uint32_t)(xTaskGetTickCount() / configTICK_RATE_HZ);
        sprintf(buf, "运行 %02lu:%02lu:%02lu",
                (unsigned long)(up_s / 3600u),
                (unsigned long)((up_s / 60u) % 60u),
                (unsigned long)(up_s % 60u));
        lv_label_set_text(s_ui_uptime, buf);

        sprintf(buf, "内存 %u B", (unsigned)xPortGetFreeHeapSize());
        lv_label_set_text(s_ui_heap, buf);

#if UI_SHOW_TOUCH_DEBUG
        /* 显示变换前的原始坐标：手指点四角，看范围就知道 SWAP / INV 该怎么开 */
        {
            uint16_t dx;
            uint16_t dy;

            if (tp_dev.sta & TP_PRES_DOWN) {
                dx = tp_dev.x[0];
                dy = tp_dev.y[0];
            } else {
                dx = 0xFFFFu;
                dy = 0xFFFFu;
            }

            sprintf(buf, "T %u,%u", (unsigned)dx, (unsigned)dy);
            lv_label_set_text(s_ui_touch, buf);
        }
#endif

        /* WiFi 页的状态一起刷新，切过去就是最新的。
         * SSID 和凭据来源也跟着刷：配置保存后不用重进页面就能看到新值。 */
        lv_label_set_text(s_wifi_state, ui_net_state_cn());
        lv_label_set_text(s_wifi_ssid, ui_wifi_ssid());
        lv_label_set_text(s_wifi_cfg_src, ui_wifi_cfg_src());

        if (app_net_mqtt_up()) {
            lv_label_set_text(s_wifi_mqtt, "在线");
            lv_obj_set_style_text_color(s_wifi_mqtt, lv_color_hex(0x22AA22), 0);
        } else {
            lv_label_set_text(s_wifi_mqtt, "离线");
            lv_obj_set_style_text_color(s_wifi_mqtt, lv_color_hex(0xDD3333), 0);
        }
    }
}
