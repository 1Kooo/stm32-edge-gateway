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
#include "drv_rtc.h"         /* drv_rtc_unix / drv_rtc_split：顶栏实时时间 */
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

/* ---- 深色玻璃主题配色 ----
 * 一处定义全站用，换肤只改这里。
 * 卡片是半透明白叠在纯色底上：重绘只是混一次色，开销很小；
 * 以后真要加背景图，这几行不用动，但那时要重新评估刷新性能
 * （半透明叠图片 = 每个脏区都要重读背景再混合）。 */
#define C_BG          lv_color_hex(0x16232F)   /* 屏幕底色 */
#define C_TEXT        lv_color_white()          /* 主文字 */
#define C_TEXT_DIM    lv_color_hex(0x9AA7B4)    /* 次要文字 */
#define C_ACCENT      lv_color_hex(0x85B7EB)    /* 选中 / 强调 */
#define C_OK          lv_color_hex(0x5DCAA5)    /* 正常态 */
#define C_WARN        lv_color_hex(0xE24B4A)    /* 异常态 */
#define OPA_CARD      25u                       /* 卡片填充 25/255 ≈ 10% 白 */
#define OPA_BORDER    90u                       /* 卡片描边 ≈ 35% 白 */
#define OPA_PILL      18u                       /* 统计条 ≈ 7% 白 */
#define OPA_TRACK     60u                       /* 进度条轨道 */

/* ================= UI：LVGL 界面（横屏 480x320） ================= */

/* 界面控件句柄：建一次，之后只改内容，不重建对象 */
static lv_obj_t *s_scr_main;        /* 首页：数据仪表盘 */
static lv_obj_t *s_scr_wifi;        /* 设置页：网络信息 + 配网入口 */
static lv_obj_t *s_scr_about;       /* 关于页：版本 / 服务器 / 运行信息 */
static uint8_t   s_scr_idx = 0;     /* 当前屏：0=首页 1=设置 2=关于 */

static lv_obj_t *s_wifi_state;      /* WiFi 页：网络状态 */
static lv_obj_t *s_wifi_mqtt;       /* WiFi 页：MQTT 状态 */
static lv_obj_t *s_wifi_ssid;       /* WiFi 页：当前生效的 SSID（动态刷新） */
static lv_obj_t *s_wifi_cfg_src;    /* WiFi 页：凭据来源 default / custom */

static lv_obj_t *s_ui_temp;         /* 温度数值 */
static lv_obj_t *s_ui_temp_bar;     /* 温度进度条 */
static lv_obj_t *s_ui_humi_pct;     /* 湿度百分数 */
static lv_obj_t *s_ui_humi_bar;     /* 湿度进度条 */
static lv_obj_t *s_ui_net_text;     /* 顶栏右侧：online / offline */
static lv_obj_t *s_ui_seq;          /* 采样序号 */
static lv_obj_t *s_ui_sent;         /* 已上报条数 */
static lv_obj_t *s_ui_backlog;      /* 待补传条数 */
static lv_obj_t *s_ui_uptime;       /* 关于页：运行时长 */
static lv_obj_t *s_ui_datetime;     /* 关于页：RTC 日期时间 */
static lv_obj_t *s_ui_heap;         /* 关于页：剩余堆 */
static lv_obj_t *s_ui_clock;        /* 首页状态条：运行时长 */
static lv_obj_t *s_ui_lastpub;      /* 首页：最近一次上报距今秒数 */

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

/* 建一张玻璃卡片：半透明白底 + 细白边，叠在深色屏幕上。
 * 阴影必须清掉 —— 主题给卡片的默认阴影在深色底上会发灰，像脏了。 */
static lv_obj_t *ui_card(lv_obj_t *parent, int32_t x, int32_t y,
                         int32_t w, int32_t h)
{
    lv_obj_t *card = lv_obj_create(parent);

    lv_obj_set_size(card, w, h);
    lv_obj_set_pos(card, x, y);
    lv_obj_set_style_bg_color(card, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(card, OPA_CARD, 0);
    lv_obj_set_style_border_color(card, lv_color_white(), 0);
    lv_obj_set_style_border_opa(card, OPA_BORDER, 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_radius(card, 10, 0);
    lv_obj_set_style_shadow_width(card, 0, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    /* 卡片上的滑动要能传到屏幕，否则手势判定会停在卡片这一层。
     * lv_obj 构造时对"有父对象的对象"默认就会加 GESTURE_BUBBLE，
     * 这里显式写一遍是为了让意图清楚，也不受库默认值变化影响。
     * 注意用的是 GESTURE_BUBBLE，不是 EVENT_BUBBLE，两个标志不是一回事。 */
    lv_obj_add_flag(card, LV_OBJ_FLAG_GESTURE_BUBBLE);

    return card;
}

/* 小信息条：左边标题、右边数值。返回数值 label，调用方只改数值不用管布局。 */
static lv_obj_t *ui_pill(lv_obj_t *parent, int32_t x, int32_t y,
                         int32_t w, const char *title)
{
    lv_obj_t *pill;
    lv_obj_t *lb;

    pill = lv_obj_create(parent);
    lv_obj_set_pos(pill, x, y);
    lv_obj_set_size(pill, w, 38);
    lv_obj_set_style_bg_color(pill, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(pill, OPA_PILL, 0);
    lv_obj_set_style_border_color(pill, lv_color_white(), 0);
    lv_obj_set_style_border_opa(pill, 60, 0);
    lv_obj_set_style_border_width(pill, 1, 0);
    lv_obj_set_style_radius(pill, 9, 0);
    lv_obj_set_style_shadow_width(pill, 0, 0);
    lv_obj_set_style_pad_all(pill, 0, 0);
    lv_obj_clear_flag(pill, LV_OBJ_FLAG_SCROLLABLE);

    lb = ui_label(pill, 12, 10, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, title);

    /* 数值右对齐：位数从 1 位涨到 6 位时不会顶到标题上 */
    lb = ui_label(pill, 0, 9, FONT_UI, C_TEXT);
    lv_label_set_text(lb, "0");
    lv_obj_align(lb, LV_ALIGN_RIGHT_MID, -12, 0);

    return lb;
}

/* 建首页：顶部状态条 + 两张数据卡 + 三格统计 + 最近上报条。
 * 之后只改文字和数值，不重建对象。底部标签栏由 ui_build_tabbar 统一建。 */
static void ui_build_main(lv_obj_t *scr)
{
    lv_obj_t *card;
    lv_obj_t *lb;

    lv_obj_set_style_bg_color(scr, C_BG, 0);
    lv_obj_set_style_text_color(scr, C_TEXT, 0);

    /* ---- 顶部状态条：左版本，右链路状态 + 运行时长 ---- */
    lb = ui_label(scr, 14, 9, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "GW " APP_VERSION);

    s_ui_net_text = ui_label(scr, 312, 9, FONT_UI, C_WARN);
    lv_label_set_text(s_ui_net_text, "离线");

    s_ui_clock = ui_label(scr, 380, 9, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(s_ui_clock, "--:--:--");

    /* ---- 温度卡 ---- */
    card = ui_card(scr, 14, 32, 216, 112);

    lb = ui_label(card, 13, 10, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "温度");

    /* 大数字仍用 Montserrat 28：纯数字比中文好看，且省字库空间。
     * 28 号字体的 cmap 里有 °（U+00B0）没有 ℃（U+2103），
     * 所以单位写 ° 而不是 ℃，否则这里会出现方块。 */
    s_ui_temp = ui_label(card, 12, 26, &lv_font_montserrat_28, C_TEXT);
    lv_label_set_text(s_ui_temp, "--.-°");

    s_ui_temp_bar = lv_bar_create(card);
    lv_obj_set_pos(s_ui_temp_bar, 13, 74);
    lv_obj_set_size(s_ui_temp_bar, 190, 6);
    lv_bar_set_range(s_ui_temp_bar, -100, 600);          /* -10.0 ~ 60.0 度 */
    lv_bar_set_value(s_ui_temp_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_ui_temp_bar, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_ui_temp_bar, OPA_TRACK, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_ui_temp_bar, C_WARN, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_ui_temp_bar, 3, LV_PART_MAIN);
    lv_obj_set_style_radius(s_ui_temp_bar, 3, LV_PART_INDICATOR);

    /* 量程说明用纯 ASCII + 汉字，别写 °：14px 字库只收了 ASCII 和界面汉字，
     * 度符号在小字号里没有字形，会变成方块。 */
    lb = ui_label(card, 13, 88, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "量程 -10~60");

    /* ---- 湿度卡 ---- */
    card = ui_card(scr, 250, 32, 216, 112);

    lb = ui_label(card, 13, 10, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "湿度");

    s_ui_humi_pct = ui_label(card, 12, 26, &lv_font_montserrat_28, C_TEXT);
    lv_label_set_text(s_ui_humi_pct, "--.- %");

    s_ui_humi_bar = lv_bar_create(card);
    lv_obj_set_pos(s_ui_humi_bar, 13, 74);
    lv_obj_set_size(s_ui_humi_bar, 190, 6);
    lv_bar_set_range(s_ui_humi_bar, 0, 100);
    lv_bar_set_value(s_ui_humi_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_ui_humi_bar, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_ui_humi_bar, OPA_TRACK, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_ui_humi_bar, C_OK, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_ui_humi_bar, 3, LV_PART_MAIN);
    lv_obj_set_style_radius(s_ui_humi_bar, 3, LV_PART_INDICATOR);

    lb = ui_label(card, 13, 88, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "当前相对湿度");

    /* ---- 三格统计 ---- */
    s_ui_seq     = ui_pill(scr, 14,  152, 140, "序号");
    s_ui_sent    = ui_pill(scr, 161, 152, 140, "已发");
    s_ui_backlog = ui_pill(scr, 308, 152, 158, "待发");

    /* ---- 最近上报条：一眼判断链路是不是活的 ---- */
    card = ui_card(scr, 14, 200, 452, 34);

    lb = ui_label(card, 13, 9, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "最近上报");

    /* 右对齐：文案长度会变（"等待中" / "128 秒前 · 已确认"），
     * 写死坐标会顶出卡片，交给对齐算更稳。 */
    s_ui_lastpub = ui_label(card, 0, 9, FONT_UI, C_TEXT);
    lv_label_set_text(s_ui_lastpub, "等待中");
    lv_obj_align(s_ui_lastpub, LV_ALIGN_RIGHT_MID, -13, 0);

#if UI_SHOW_TOUCH_DEBUG
    s_ui_touch = ui_label(scr, 180, 244, FONT_UI, C_TEXT_DIM);
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

/* ================= OTA 升级弹层 =================
 *
 * KEY1 触发 OTA 时弹出：居中面板（约 70% 宽 x 60% 高），背景压暗不整屏盖死。
 * 生命周期只有一个原则：弹层退出 = 升级成功。
 *   - 成功：app_ota_done()=1 显示"升级完成"，随后板子复位，弹层随固件消失
 *   - 失败：net_task 置 app_ota_err()，红字原因 + 按 KEY2 手动关闭
 *   - 中途断电复位：固件头和 pending 都没写，设备回老固件，弹层自然不存在
 *
 * 进度数据全部来自 app_ota.h 的查询函数（net_task 写、ui_task 读），
 * 单字节 volatile 读写天然原子，不需要临界区。 */
static lv_obj_t *s_ota_layer;       /* 弹层根：NULL = 没开 */
static lv_obj_t *s_ota_state;       /* 状态行：等待网络 / 下载固件 / 失败原因 */
static lv_obj_t *s_ota_pct_lb;      /* 百分比大数字（montserrat 28，纯 ASCII） */
static lv_obj_t *s_ota_bar;         /* 进度条 */
static lv_obj_t *s_ota_ver;         /* 版本行 "V0.6 -> V0.7"（握手后才有值） */
static lv_obj_t *s_ota_hint;        /* 底部提示行 */
static lv_obj_t *s_ota_stage[5];    /* 节点行：等待/准备/下载/校验/完成 */
static uint8_t   s_ota_fail_ui;     /* 失败/超时态：置 1 才允许 KEY2 关闭 */
static uint32_t  s_ota_open_tick;   /* 弹层打开时刻，30 秒等待兜底 */

static void ui_ota_close(void)
{
    int i;

    if (s_ota_layer == NULL) {
        return;
    }
    lv_obj_delete(s_ota_layer);
    s_ota_layer  = NULL;
    s_ota_state  = NULL;
    s_ota_pct_lb = NULL;
    s_ota_bar    = NULL;
    s_ota_ver    = NULL;
    s_ota_hint   = NULL;
    for (i = 0; i < 5; i++) {
        s_ota_stage[i] = NULL;
    }
    s_ota_fail_ui = 0;
}

static void ui_ota_open(void)
{
    lv_obj_t *panel;
    lv_obj_t *scr;                  /* 挂当前屏：KEY1 任何页都能按 */
    lv_obj_t *lb;
    const char *names[5] = {"等待", "准备", "下载", "校验", "完成"};
    int i;

    if (s_ota_layer != NULL) {
        return;                     /* 已经开着，别重复建 */
    }

    scr = (s_scr_idx == 0u) ? s_scr_main :
          ((s_scr_idx == 1u) ? s_scr_wifi : s_scr_about);

    /* 遮罩：LVGL 9.5 虽然有 backdrop blur 属性，但软件实现要在 layer 缓冲里
     * 先渲染背景（480x320x2 = 300KB），板子 RAM 装不下，所以走"渐变压暗"。
     * 顶部纯黑、底部回落到界面底色，上下有明暗差，比纯色一块更有空间感。 */
    s_ota_layer = lv_obj_create(scr);
    lv_obj_set_pos(s_ota_layer, 0, 0);
    lv_obj_set_size(s_ota_layer, 480, 320);
    lv_obj_add_flag(s_ota_layer, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(s_ota_layer, lv_color_black(), 0);
    lv_obj_set_style_bg_grad_color(s_ota_layer, C_BG, 0);
    lv_obj_set_style_bg_grad_dir(s_ota_layer, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(s_ota_layer, 205, 0);
    lv_obj_set_style_border_width(s_ota_layer, 0, 0);
    lv_obj_set_style_radius(s_ota_layer, 0, 0);
    lv_obj_set_style_pad_all(s_ota_layer, 0, 0);
    lv_obj_clear_flag(s_ota_layer, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_ota_layer, LV_OBJ_FLAG_GESTURE_BUBBLE);

    /* 面板：与主界面卡片同款玻璃风，填充/描边各加浓一档，压得住遮罩 */
    panel = lv_obj_create(s_ota_layer);
    lv_obj_set_pos(panel, 72, 64);
    lv_obj_set_size(panel, 336, 192);
    lv_obj_set_style_bg_color(panel, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(panel, 33, 0);
    lv_obj_set_style_border_color(panel, lv_color_white(), 0);
    lv_obj_set_style_border_opa(panel, 115, 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_radius(panel, 16, 0);
    lv_obj_set_style_pad_all(panel, 0, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    /* 柔和阴影：面板四周一圈黑色光晕 + 轻微下投影，视觉上"浮"在暗背景上。
     * 这是替代真模糊最有效的一招（软件绘制，只在 500ms 刷新时重画，代价可接受）。 */
    lv_obj_set_style_shadow_color(panel, lv_color_black(), 0);
    lv_obj_set_style_shadow_width(panel, 28, 0);
    lv_obj_set_style_shadow_spread(panel, 2, 0);
    lv_obj_set_style_shadow_opa(panel, 120, 0);
    lv_obj_set_style_shadow_offset_y(panel, 6, 0);

    lb = ui_label(panel, 24, 14, FONT_UI, C_TEXT);
    lv_label_set_text(lb, "固件升级");

    /* 版本行：放标题右侧。握手拿到固件头之前整行隐藏 ——
     * "版本待定"没有信息量，还占地方。
     * 千万别把它对齐到右上角：那里是大百分比的专属位置，
     * 两个 TOP_RIGHT 会叠在同一坐标（这就是之前字体重叠的原因）。 */
    s_ota_ver = ui_label(panel, 110, 17, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(s_ota_ver, "");
    lv_obj_add_flag(s_ota_ver, LV_OBJ_FLAG_HIDDEN);

    s_ota_state = ui_label(panel, 24, 48, FONT_UI, C_ACCENT);
    lv_label_set_text(s_ota_state, "等待网络");

    /* 百分比用现成的 montserrat 28：纯 ASCII 零字库成本 */
    s_ota_pct_lb = ui_label(panel, 0, 40, &lv_font_montserrat_28, C_TEXT);
    lv_label_set_text(s_ota_pct_lb, "0%");
    lv_obj_align(s_ota_pct_lb, LV_ALIGN_TOP_RIGHT, -24, 0);

    s_ota_bar = lv_bar_create(panel);
    lv_obj_set_pos(s_ota_bar, 24, 92);
    lv_obj_set_size(s_ota_bar, 288, 10);
    lv_bar_set_range(s_ota_bar, 0, 100);
    lv_bar_set_value(s_ota_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_ota_bar, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_ota_bar, OPA_TRACK, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_ota_bar, C_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_ota_bar, 5, LV_PART_MAIN);
    lv_obj_set_style_radius(s_ota_bar, 5, LV_PART_INDICATOR);

    for (i = 0; i < 5; i++) {
        s_ota_stage[i] = ui_label(panel, 24 + i * 60, 124, FONT_UI, C_TEXT_DIM);
        lv_label_set_text(s_ota_stage[i], names[i]);
    }

    s_ota_hint = ui_label(panel, 24, 164, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(s_ota_hint, "请勿断开电源 完成后自动重启");

    s_ota_fail_ui   = 0;
    s_ota_open_tick = xTaskGetTickCount();
}

/* 节点行高亮：idx 那个白字 + 蓝色下划线，其余灰。idx=-1 全灰（失败态）。 */
static void ui_ota_set_stage(int idx)
{
    int i;

    for (i = 0; i < 5; i++) {
        if (i == idx) {
            lv_obj_set_style_text_color(s_ota_stage[i], C_TEXT, 0);
            lv_obj_set_style_border_color(s_ota_stage[i], C_ACCENT, 0);
            lv_obj_set_style_border_side(s_ota_stage[i], LV_BORDER_SIDE_BOTTOM, 0);
            lv_obj_set_style_border_width(s_ota_stage[i], 2, 0);
        } else {
            lv_obj_set_style_text_color(s_ota_stage[i], C_TEXT_DIM, 0);
            lv_obj_set_style_border_width(s_ota_stage[i], 0, 0);
        }
    }
}

/* 失败码 -> 屏上文案。挑覆盖 99% 场景的报，剩下的给个通用兜底。 */
static const char *ui_ota_fail_text(int8_t code)
{
    switch (code) {
    case -1: return "服务器连不上";
    case -2: return "服务器无响应";
    case -3: return "固件头不对";
    case -4: return "固件过大";
    case -5:
    case -7: return "存储写入失败";
    case -6: return "网络中断";
    case -8: return "数据校验失败";
    case -9: return "状态写入失败";
    default: return "未知错误";
    }
}

/* 弹层刷新：ui_task 的 500ms 闸里调一次。只改文字/颜色/进度值，
 * 不建不删对象 —— LVGL 属性刷新是廉价的，建删才是贵的。 */
static void ui_ota_refresh(void)
{
    char     buf[48];
    uint8_t  busy  = app_ota_busy();
    uint8_t  phase = app_ota_phase();
    uint8_t  pct   = app_ota_pct();
    uint32_t ver   = app_ota_ver();

    /* 版本行：握手拿到固件头后才显示 "V0.6 -> V0.7"，之前整行隐藏 */
    if (ver != 0u) {
        sprintf(buf, "%s -> V%lu.%lu", APP_VERSION,
                (unsigned long)(ver >> 16), (unsigned long)(ver & 0xFFFFu));
        lv_label_set_text(s_ota_ver, buf);
        lv_obj_clear_flag(s_ota_ver, LV_OBJ_FLAG_HIDDEN);
    }

    if (app_ota_done()) {
        /* 唯一的成功出口：显示完这一屏，板子就复位装新固件，弹层随固件消失 */
        lv_label_set_text(s_ota_state, "升级完成 重启中");
        lv_obj_set_style_text_color(s_ota_state, C_OK, 0);
        lv_bar_set_value(s_ota_bar, 100, LV_ANIM_OFF);
        lv_label_set_text(s_ota_pct_lb, "100%");
        ui_ota_set_stage(4);
        lv_label_set_text(s_ota_hint, "请勿断开电源");
        return;
    }

    if (busy) {
        lv_obj_set_style_text_color(s_ota_state, C_ACCENT, 0);
        lv_bar_set_value(s_ota_bar, pct, LV_ANIM_OFF);
        sprintf(buf, "%u%%", (unsigned)pct);
        lv_label_set_text(s_ota_pct_lb, buf);

        if (phase == 2u && pct >= 100u) {
            lv_label_set_text(s_ota_state, "校验固件");
            ui_ota_set_stage(3);
        } else if (phase == 2u) {
            lv_label_set_text(s_ota_state, "下载固件");
            ui_ota_set_stage(2);
        } else {
            lv_label_set_text(s_ota_state, "准备存储区");
            ui_ota_set_stage(1);
        }
        return;
    }

    /* busy=0 且没 done：要么失败了，要么还没轮到 net_task 开始 */
    if (app_ota_err() != 0) {
        sprintf(buf, "升级失败 %s", ui_ota_fail_text(app_ota_err()));
        lv_label_set_text(s_ota_state, buf);
        lv_obj_set_style_text_color(s_ota_state, C_WARN, 0);
        lv_obj_set_style_bg_color(s_ota_bar, C_WARN, LV_PART_INDICATOR);
        lv_label_set_text(s_ota_hint, "按 KEY2 关闭 可重新按 KEY1 再试");
        ui_ota_set_stage(-1);
        s_ota_fail_ui = 1;
        return;
    }

    /* 没失败也没开始：KEY1 到 net_task 真正开跑之间有几秒空窗，30 秒兜底 */
    if ((xTaskGetTickCount() - s_ota_open_tick) > pdMS_TO_TICKS(30000)) {
        lv_label_set_text(s_ota_state, "等待超时");
        lv_obj_set_style_text_color(s_ota_state, C_WARN, 0);
        lv_label_set_text(s_ota_hint, "按 KEY2 关闭");
        s_ota_fail_ui = 1;
        return;
    }

    lv_label_set_text(s_ota_state, "等待网络");
    ui_ota_set_stage(0);
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

/* 设置页：网络信息 + 配网入口。内容压到 240 以内，给底部标签栏让位。 */
static void ui_build_wifi(lv_obj_t *scr)
{
    lv_obj_t *card;
    lv_obj_t *lb;
    lv_obj_t *btn;

    lv_obj_set_style_bg_color(scr, C_BG, 0);
    lv_obj_set_style_text_color(scr, C_TEXT, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lb = ui_label(scr, 14, 9, FONT_UI, C_TEXT);
    lv_label_set_text(lb, "设置");

    lb = ui_label(scr, 400, 9, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "KEY2 换页");

    /* ---- 网络信息卡 ---- */
    card = ui_card(scr, 14, 34, 452, 128);

    lb = ui_label(card, 14, 12, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "SSID");
    s_wifi_ssid = ui_label(card, 86, 12, FONT_UI, C_TEXT);
    lv_label_set_text(s_wifi_ssid, ui_wifi_ssid());

    lb = ui_label(card, 14, 40, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "网络");
    s_wifi_state = ui_label(card, 86, 40, FONT_UI, C_TEXT);
    lv_label_set_text(s_wifi_state, ui_net_state_cn());

    lb = ui_label(card, 14, 68, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "MQTT");
    s_wifi_mqtt = ui_label(card, 86, 68, FONT_UI, C_TEXT);
    lv_label_set_text(s_wifi_mqtt, "离线");

    lb = ui_label(card, 14, 96, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "配置");
    s_wifi_cfg_src = ui_label(card, 86, 96, FONT_UI, C_ACCENT);
    lv_label_set_text(s_wifi_cfg_src, ui_wifi_cfg_src());

    /* ---- 配网卡片 ---- */
    card = ui_card(scr, 14, 170, 452, 70);

    lb = ui_label(card, 14, 12, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "已存外部 Flash · 断电不丢");

    btn = lv_button_create(card);
    lv_obj_set_pos(btn, 348, 26);
    lv_obj_set_size(btn, 86, 32);
    lv_obj_set_style_bg_color(btn, C_ACCENT, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(btn, 8, 0);
    lv_obj_set_style_shadow_width(btn, 0, 0);

    lb = lv_label_create(btn);
    lv_label_set_text(lb, "配网");
    lv_obj_set_style_text_font(lb, FONT_UI, 0);      /* 按钮 label 不继承字体 */
    lv_obj_set_style_text_color(lb, lv_color_hex(0x16232F), 0);
    lv_obj_center(lb);
    lv_obj_add_event_cb(btn, ui_cfg_open_cb, LV_EVENT_CLICKED, NULL);
}

/* 关于页：静态信息 + 两个动态值。服务器地址/主题一眼可查，现场排障用。 */
static void ui_build_about(lv_obj_t *scr)
{
    lv_obj_t *card;
    lv_obj_t *lb;

    lv_obj_set_style_bg_color(scr, C_BG, 0);
    lv_obj_set_style_text_color(scr, C_TEXT, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lb = ui_label(scr, 14, 9, FONT_UI, C_TEXT);
    lv_label_set_text(lb, "关于");

    card = ui_card(scr, 14, 34, 452, 196);

    lb = ui_label(card, 14, 12, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "版本");
    lb = ui_label(card, 86, 12, FONT_UI, C_TEXT);
    lv_label_set_text(lb, APP_VERSION);

    lb = ui_label(card, 14, 40, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "服务器");
    lb = ui_label(card, 86, 40, FONT_UI, C_TEXT);
    lv_label_set_text(lb, BEMFA_HOST);

    lb = ui_label(card, 14, 68, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "主题");
    lb = ui_label(card, 86, 68, FONT_UI, C_TEXT);
    lv_label_set_text(lb, BEMFA_TOPIC);

    /* 时间：RTC 真实日期时间。卡片高度 196，六行按 28 间距排，
     * 「时间」这一行新加，运行/内存/固件三行整体下移。 */
    lb = ui_label(card, 14, 96, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "时间");
    s_ui_datetime = ui_label(card, 86, 96, FONT_UI, C_TEXT);
    lv_label_set_text(s_ui_datetime, "--");

    lb = ui_label(card, 14, 124, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "运行");
    s_ui_uptime = ui_label(card, 86, 124, FONT_UI, C_TEXT);
    lv_label_set_text(s_ui_uptime, "00:00:00");

    lb = ui_label(card, 14, 152, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "内存");
    s_ui_heap = ui_label(card, 86, 152, FONT_UI, C_TEXT);
    lv_label_set_text(s_ui_heap, "0 B");

    lb = ui_label(card, 14, 178, FONT_UI, C_TEXT_DIM);
    lv_label_set_text(lb, "固件 V1.0 · 中文字库 1084 字形");
}

static uint32_t s_last_sw = 0;      /* 上次切屏的时刻，用来防连点 */

/* 统一的切屏入口：tab 按钮、KEY2、触摸手势都走这里，避免几处各写一套。
 *
 * 不用转场动画：SW 渲染下全屏切换要几十毫秒，还要额外申请屏幕快照缓冲；
 * 工业界面「点了就到」比「滑过去」更符合直觉，也省内存。 */
static void ui_switch_to(uint8_t idx)
{
    lv_obj_t *target;

    if (idx > 2u || idx == s_scr_idx) {
        return;                                  /* 越界 / 已经在目标页 */
    }
    if ((xTaskGetTickCount() - s_last_sw) < pdMS_TO_TICKS(250)) {
        return;                                  /* 连点保护 */
    }

    s_last_sw = xTaskGetTickCount();
    s_scr_idx = idx;

    if (idx == 0u) {
        target = s_scr_main;
    } else if (idx == 1u) {
        target = s_scr_wifi;
    } else {
        target = s_scr_about;
    }

    lv_screen_load(target);
}

/* tab 按钮回调：user_data 存的是目标页索引（整数塞 void*，不要传栈地址） */
static void ui_tab_cb(lv_event_t *e)
{
    ui_switch_to((uint8_t)(uintptr_t)lv_event_get_user_data(e));
}

/* 手势回调：按当前页索引前进 / 后退，到两头就停。
 * 方向语义是"手指往哪滑"：往左滑 = 看右边的内容 = 前进一页。
 * 这条路是 LVGL 原生手势（电阻屏上偶尔能触发），主路径其实是
 * ui_task 里的 lv_port_take_swipe；两条路逻辑保持一致，别只改一边。 */
static void ui_gesture_cb(lv_event_t *e)
{
    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());

    (void)e;

    if (dir == LV_DIR_LEFT && s_scr_idx < 2u) {
        ui_switch_to((uint8_t)(s_scr_idx + 1u));
    } else if (dir == LV_DIR_RIGHT && s_scr_idx > 0u) {
        ui_switch_to((uint8_t)(s_scr_idx - 1u));
    }
}

/* 底部标签栏。每个屏幕各建一份（各自是屏幕的子对象），
 * 切屏时整屏换掉，不用维护三份高亮状态的同步。 */
static void ui_build_tabbar(lv_obj_t *scr, uint8_t cur)
{
    static const char *names[3]   = { "首页", "设置", "关于" };
    static const char *symbols[3] = { LV_SYMBOL_HOME, LV_SYMBOL_SETTINGS,
                                      LV_SYMBOL_LIST };
    lv_obj_t *bar;
    lv_obj_t *btn;
    lv_obj_t *lb;
    lv_color_t c;
    uint8_t   i;

    bar = lv_obj_create(scr);
    lv_obj_set_pos(bar, 0, 264);
    lv_obj_set_size(bar, 480, 56);
    lv_obj_set_style_bg_color(bar, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(bar, 18, 0);
    lv_obj_set_style_border_side(bar, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_border_width(bar, 1, 0);
    lv_obj_set_style_border_color(bar, lv_color_white(), 0);
    lv_obj_set_style_border_opa(bar, 40, 0);
    lv_obj_set_style_radius(bar, 0, 0);
    lv_obj_set_style_pad_all(bar, 0, 0);
    lv_obj_set_style_shadow_width(bar, 0, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    for (i = 0; i < 3u; i++) {
        c = (i == cur) ? C_ACCENT : lv_color_hex(0x8A99A8);

        btn = lv_button_create(bar);
        lv_obj_set_size(btn, 150, 48);
        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_set_style_shadow_width(btn, 0, 0);
        lv_obj_set_style_pad_all(btn, 0, 0);
        lv_obj_add_event_cb(btn, ui_tab_cb, LV_EVENT_CLICKED,
                            (void *)(uintptr_t)i);

        /* 图标走默认字体（FontAwesome 码点），文字走中文字库 —— 两个 label 各管一段，
         * 不能合成一个 label：字体只有一个，混排会有一半变方块。 */
        lb = ui_label(btn, 0, 5, LV_FONT_DEFAULT, c);
        lv_label_set_text(lb, symbols[i]);
        lv_obj_align(lb, LV_ALIGN_TOP_MID, 0, 5);

        lb = ui_label(btn, 0, 23, FONT_UI, c);
        lv_label_set_text(lb, names[i]);
        lv_obj_align(lb, LV_ALIGN_TOP_MID, 0, 23);
    }
}

/* 建三个屏幕：上电先显示首页 */
static void ui_create(void)
{
    s_scr_main  = lv_obj_create(NULL);
    s_scr_wifi  = lv_obj_create(NULL);
    s_scr_about = lv_obj_create(NULL);

    /* 屏幕必须清掉 SCROLLABLE。lv_obj 构造时默认会给所有对象加这个标志，
     * 而手势判定开头就是 "有对象在滚动就直接返回"，屏幕一旦可滚动，
     * 手指在空白处划动会被当成滚动，手势永远触发不了。 */
    lv_obj_clear_flag(s_scr_main,  LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_scr_wifi,  LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_scr_about, LV_OBJ_FLAG_SCROLLABLE);

    ui_build_main(s_scr_main);
    ui_build_wifi(s_scr_wifi);
    ui_build_about(s_scr_about);

    ui_build_tabbar(s_scr_main,  0);
    ui_build_tabbar(s_scr_wifi,  1);
    ui_build_tabbar(s_scr_about, 2);

    /* 屏幕本身也要能收手势：手指按在空白处时，命中的对象就是 screen。
     * 三屏全注册，否则停在某一页时手势会突然失灵。 */
    lv_obj_add_event_cb(s_scr_main,  ui_gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_add_event_cb(s_scr_wifi,  ui_gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_add_event_cb(s_scr_about, ui_gesture_cb, LV_EVENT_GESTURE, NULL);

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
    int16_t       t_show;           /* 当前温度（0.1℃），负数单独处理符号 */
    int16_t       t_show_abs;
    uint8_t       key_cnt = 0;
    uint16_t      rtc_y;            /* RTC 拆出来的年月日时分秒 */
    uint8_t       rtc_mo, rtc_d, rtc_h, rtc_mi, rtc_s;
    (void)args;

    ui_create();

    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(5));

        /* 每 4 次循环（约 20ms）扫一次按键。
         * KEY2 = 下一页（三页线性，到关于页就不再前进），KEY1 = 请求 OTA。 */
        /* 弹层开着时物理键全部让位：KEY1 会触发 OTA、KEY2 会切屏，
         * 两者都会把弹层晾在半路。关掉弹层再恢复。 */
        /* OTA 弹层独占按键：只有失败/超时态才受理 KEY2 关闭，
         * 下载中/成功态任何键都不响应 —— 成功的退出方式只有复位。 */
        if (++key_cnt >= 4u) {
            uint8_t k;

            key_cnt = 0;
            k = KEY_Scan(0);

            if (s_ota_layer != NULL) {
                if (k == KEY2_PRESS && s_ota_fail_ui) {
                    ui_ota_close();
                }
            } else if (s_cfg_layer == NULL) {
                if (k == KEY2_PRESS) {
                    if (s_scr_idx < 2u) {
                        ui_switch_to((uint8_t)(s_scr_idx + 1u));
                    } else {
                        /* 关于页再按一次回到首页：按键是三页循环。
                         * 滑动切页仍是"两头停住"，两套语义并存。 */
                        ui_switch_to(0u);
                    }
                } else if (k == KEY1_PRESS) {
                    /* 这里只置标志 + 开弹层，真正的下载在 net_task 里做。
                     * 绝不能直接调 app_ota_run —— 那是要阻塞一两分钟的操作，
                     * ui 任务一卡住，屏幕就冻在最后一帧，还会触发看门狗式的假死。 */
                    app_log("[ui] KEY1 -> OTA request\r\n");
                    app_ota_request();
                    ui_ota_open();
                }
            }
        }

        /* 滑动切屏。无条件 take_swipe（取走即清零）：
         * 弹层开着时也要把结果取走丢弃 —— 否则在键盘上滑动的残留值会留在
         * 移植层，等弹层关掉后第一次循环就取出来，页面自己跳走。 */
        {
            uint8_t sw = lv_port_take_swipe();

            if (sw != 0u && s_cfg_layer == NULL && s_ota_layer == NULL) {
                if (sw == 1u) {                     /* 手指往左 = 前进一页 */
                    if (s_scr_idx < 2u) {
                        ui_switch_to((uint8_t)(s_scr_idx + 1u));
                    }
                } else {                            /* 手指往右 = 后退一页 */
                    if (s_scr_idx > 0u) {
                        ui_switch_to((uint8_t)(s_scr_idx - 1u));
                    }
                }
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

        /* OTA 弹层开着时刷进度：文案/颜色/进度值三样，500ms 一拍 */
        if (s_ota_layer != NULL) {
            ui_ota_refresh();
        }

        /* 原来直接在临界区里拷 s_latest（app_freertos.c 的私有变量），
         * 拆文件后走 getter —— 临界区逻辑原样，只是挪进了对方那边。 */
        app_data_get_snapshot(&snap);

        /* 温度：数值和进度条一起更新。
         * 负数不能直接喂 %d.%d：-53 会算成 "-5.-3°"，符号要单独提出来。 */
        if (snap.temp_x10 == -999) {
            lv_label_set_text(s_ui_temp, "--.-°");
            lv_bar_set_value(s_ui_temp_bar, -100, LV_ANIM_OFF);
        } else {
            t_show     = snap.temp_x10;
            t_show_abs = (t_show < 0) ? (int16_t)(-t_show) : t_show;

            sprintf(buf, "%s%d.%d°", (t_show < 0) ? "-" : "",
                    (int)(t_show_abs / 10), (int)(t_show_abs % 10));
            lv_label_set_text(s_ui_temp, buf);
            lv_bar_set_value(s_ui_temp_bar, snap.temp_x10, LV_ANIM_OFF);
        }

        /* 湿度：0xFFFF = 无效（读不到 / 被闸门拦下） */
        if (snap.humi_x10 == 0xFFFFu) {
            lv_label_set_text(s_ui_humi_pct, "--.- %");
            lv_bar_set_value(s_ui_humi_bar, 0, LV_ANIM_OFF);
        } else {
            /* 0.1% 分辨率，别截成整数 */
            sprintf(buf, "%u.%u %%", (unsigned)(snap.humi_x10 / 10u),
                    (unsigned)(snap.humi_x10 % 10u));
            lv_label_set_text(s_ui_humi_pct, buf);
            lv_bar_set_value(s_ui_humi_bar, snap.humi_x10 / 10u, LV_ANIM_OFF);
        }

        /* 采样序号 / 已上报 / 待补传：pill 只有数值，单位在标题上 */
        sprintf(buf, "%lu", (unsigned long)snap.seq);
        lv_label_set_text(s_ui_seq, buf);

        sprintf(buf, "%lu", (unsigned long)app_net_pub_cnt());
        lv_label_set_text(s_ui_sent, buf);

        sprintf(buf, "%lu", (unsigned long)app_store_log_pending());
        lv_label_set_text(s_ui_backlog, buf);

        /* 链路状态：有 MQTT 就绿，否则红 */
        if (app_net_mqtt_up()) {
            lv_label_set_text(s_ui_net_text, "在线");
            lv_obj_set_style_text_color(s_ui_net_text, C_OK, 0);
        } else {
            lv_label_set_text(s_ui_net_text, "离线");
            lv_obj_set_style_text_color(s_ui_net_text, C_WARN, 0);
        }

        /* 上电时长：只喂关于页「运行」那一行，语义就是"上电跑了多久" */
        up_s = (uint32_t)(xTaskGetTickCount() / configTICK_RATE_HZ);
        sprintf(buf, "%02lu:%02lu:%02lu",
                (unsigned long)(up_s / 3600u),
                (unsigned long)((up_s / 60u) % 60u),
                (unsigned long)(up_s % 60u));
        lv_label_set_text(s_ui_uptime, buf);

        /* 首页顶栏：只有校准过的时间才显示。
         * drv_rtc_ok 为假 = LSE 起不来；drv_rtc_synced 为假 = RTC 里的值还只是
         * 编译时刻兜底（断电丢了备份域、尚未联网对时），那个点数是假的，
         * 与其让人拿它看数据时间，不如空着显示 --:--:--。 */
        if (drv_rtc_ok() && drv_rtc_synced()) {
            uint32_t now = drv_rtc_unix();

            drv_rtc_split(now, &rtc_y, &rtc_mo, &rtc_d, &rtc_h, &rtc_mi, &rtc_s);

            sprintf(buf, "%02u:%02u:%02u",
                    (unsigned)rtc_h, (unsigned)rtc_mi, (unsigned)rtc_s);
            lv_label_set_text(s_ui_clock, buf);

            /* 关于页「时间」：完整日期时间，一眼能看出跨没跨天 */
            sprintf(buf, "%04u-%02u-%02u %02u:%02u:%02u",
                    (unsigned)rtc_y, (unsigned)rtc_mo, (unsigned)rtc_d,
                    (unsigned)rtc_h, (unsigned)rtc_mi, (unsigned)rtc_s);
            lv_label_set_text(s_ui_datetime, buf);
        } else {
            lv_label_set_text(s_ui_clock, "--:--:--");
            lv_label_set_text(s_ui_datetime, "--:--");
        }

        sprintf(buf, "%u B", (unsigned)xPortGetFreeHeapSize());
        lv_label_set_text(s_ui_heap, buf);

        /* OTA 下载优先占这条横幅：下载期间 MQTT 本来就断着，
         * "等待中"停在原地没信息量，进度才是当下最想知道的事。
         * 只在 busy 时显示 —— 失败后 busy 清 0，这条自动回到常规显示。 */
        if (app_ota_busy()) {
            if (app_ota_phase() == 2u) {
                sprintf(buf, "OTA 下载中 %u%%", (unsigned)app_ota_pct());
            } else {
                sprintf(buf, "OTA 准备中 %u%%", (unsigned)app_ota_pct());
            }
            lv_label_set_text(s_ui_lastpub, buf);
        }
        /* 最近上报：靠发布计数变化判断，不用改 net_task。
         * 计数不动就说明没有新数据确认过 —— 断网时自然停在"等待中"，
         * 一眼能看出链路死活（比只看"在线"两字可靠）。 */
        else
        {
            static uint32_t last_cnt  = 0;
            static uint32_t last_tick = 0;
            uint32_t        cnt       = app_net_pub_cnt();

            if (cnt != last_cnt) {
                last_cnt  = cnt;
                last_tick = xTaskGetTickCount();
            }

            if (cnt == 0u) {
                lv_label_set_text(s_ui_lastpub, "等待中");
            } else {
                /* 文案里别用中点符号：14px 字库只收了 ASCII 和界面汉字，
                 * 生僻标点没有字形，会显示成方块。空格分隔最稳。 */
                sprintf(buf, "%lu 秒前 已确认",
                        (unsigned long)((xTaskGetTickCount() - last_tick)
                                        / configTICK_RATE_HZ));
                lv_label_set_text(s_ui_lastpub, buf);
            }
        }

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

        /* 设置页的状态一起刷新，切过去就是最新的。 */
        lv_label_set_text(s_wifi_state, ui_net_state_cn());
        lv_label_set_text(s_wifi_ssid, ui_wifi_ssid());
        lv_label_set_text(s_wifi_cfg_src, ui_wifi_cfg_src());

        if (app_net_mqtt_up()) {
            lv_label_set_text(s_wifi_mqtt, "在线");
            lv_obj_set_style_text_color(s_wifi_mqtt, C_OK, 0);
        } else {
            lv_label_set_text(s_wifi_mqtt, "离线");
            lv_obj_set_style_text_color(s_wifi_mqtt, C_WARN, 0);
        }
    }
}
