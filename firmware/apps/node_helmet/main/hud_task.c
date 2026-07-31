/* Heads-up display: ST7789 panel driven through esp_lcd + esp_lvgl_port.
 *
 * ARCHITECTURE.md calls for twin transparent micro-OLEDs; those are not a
 * commodity part, so the bench build targets a 240x240 ST7789 whose driver
 * ships inside IDF. Swapping in the real panel is confined to panel_init().
 *
 * The safety banner is derived from the mirrored suit state, never from
 * anything the cloud sends — advisories can add warnings to the list, but they
 * cannot make the banner claim the suit is safe. */
#include "helmet_tasks.h"
#include "board_helmet.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "powersuit_proto/wire.h"

#include <stdio.h>

static const char *TAG = "node_helmet";

#define HUD_REFRESH_MS 100   /* 10 Hz is plenty for a status overlay */

static lv_disp_t *s_disp;
static lv_obj_t *s_banner;
static lv_obj_t *s_battery_bar;
static lv_obj_t *s_battery_label;
static lv_obj_t *s_warn_list;
static lv_obj_t *s_link_label;
static lv_obj_t *s_speed_label;

static esp_err_t panel_init(esp_lcd_panel_handle_t *out_panel,
                            esp_lcd_panel_io_handle_t *out_io)
{
    gpio_config_t bl = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << HELMET_LCD_BL_GPIO,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&bl), TAG, "backlight gpio");

    spi_bus_config_t buscfg = {
        .sclk_io_num = HELMET_LCD_SCLK_GPIO,
        .mosi_io_num = HELMET_LCD_MOSI_GPIO,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = HELMET_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(HELMET_LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO),
                        TAG, "spi bus");

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = HELMET_LCD_DC_GPIO,
        .cs_gpio_num = HELMET_LCD_CS_GPIO,
        .pclk_hz = HELMET_LCD_CLK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)HELMET_LCD_SPI_HOST, &io_cfg, out_io),
        TAG, "panel io");

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = HELMET_LCD_RST_GPIO,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(*out_io, &panel_cfg, out_panel),
                        TAG, "st7789");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(*out_panel), TAG, "reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(*out_panel), TAG, "init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(*out_panel, true), TAG, "invert");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(*out_panel, true), TAG, "display on");
    gpio_set_level(HELMET_LCD_BL_GPIO, 1);
    return ESP_OK;
}

static void build_ui(void)
{
    lvgl_port_lock(0);
    lv_obj_t *scr = lv_disp_get_scr_act(s_disp);
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    /* Battery bar across the top. */
    s_battery_bar = lv_bar_create(scr);
    lv_obj_set_size(s_battery_bar, 180, 12);
    lv_obj_align(s_battery_bar, LV_ALIGN_TOP_MID, 0, 8);
    lv_bar_set_range(s_battery_bar, 0, 100);

    s_battery_label = lv_label_create(scr);
    lv_obj_align(s_battery_label, LV_ALIGN_TOP_RIGHT, -6, 6);
    lv_obj_set_style_text_color(s_battery_label, lv_color_white(), 0);
    lv_label_set_text(s_battery_label, "--%");

    /* Centre reticle. */
    lv_obj_t *reticle = lv_arc_create(scr);
    lv_obj_set_size(reticle, 90, 90);
    lv_obj_center(reticle);
    lv_arc_set_rotation(reticle, 270);
    lv_arc_set_bg_angles(reticle, 0, 360);
    lv_arc_set_value(reticle, 0);
    lv_obj_remove_style(reticle, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(reticle, LV_OBJ_FLAG_CLICKABLE);

    /* Safety banner, left. */
    s_banner = lv_label_create(scr);
    lv_obj_align(s_banner, LV_ALIGN_LEFT_MID, 6, -60);
    lv_label_set_text(s_banner, "BOOT");

    /* Warning list, right. */
    s_warn_list = lv_label_create(scr);
    lv_obj_set_width(s_warn_list, 120);
    lv_obj_align(s_warn_list, LV_ALIGN_RIGHT_MID, -4, 20);
    lv_obj_set_style_text_color(s_warn_list, lv_palette_main(LV_PALETTE_AMBER), 0);
    lv_label_set_text(s_warn_list, "");

    /* Link + airspeed along the bottom. */
    s_link_label = lv_label_create(scr);
    lv_obj_align(s_link_label, LV_ALIGN_BOTTOM_LEFT, 6, -6);
    lv_obj_set_style_text_color(s_link_label, lv_color_white(), 0);
    lv_label_set_text(s_link_label, "CLOUD --");

    s_speed_label = lv_label_create(scr);
    lv_obj_align(s_speed_label, LV_ALIGN_BOTTOM_RIGHT, -6, -6);
    lv_obj_set_style_text_color(s_speed_label, lv_color_white(), 0);
    lv_label_set_text(s_speed_label, "0 m/s");

    lvgl_port_unlock();
}

static lv_color_t colour_for_state(uint8_t state)
{
    switch (state) {
    case PS_STATE_OPERATIONAL: return lv_palette_main(LV_PALETTE_GREEN);
    case PS_STATE_STANDBY:     return lv_palette_main(LV_PALETTE_AMBER);
    case PS_STATE_PASSIVE:     return lv_palette_main(LV_PALETTE_ORANGE);
    case PS_STATE_ESTOP:       return lv_palette_main(LV_PALETTE_RED);
    case PS_STATE_FAULT:       return lv_palette_main(LV_PALETTE_RED);
    default:                   return lv_palette_main(LV_PALETTE_BLUE_GREY);
    }
}

static const char *name_for_state(uint8_t state)
{
    switch (state) {
    case PS_STATE_BOOT:        return "BOOT";
    case PS_STATE_STANDBY:     return "STANDBY";
    case PS_STATE_OPERATIONAL: return "ACTIVE";
    case PS_STATE_PASSIVE:     return "PASSIVE";
    case PS_STATE_ESTOP:       return "E-STOP";
    case PS_STATE_FAULT:       return "FAULT";
    default:                   return "?";
    }
}

static void hud_task(void *arg)
{
    (void)arg;
    char buf[HELMET_MAX_WARNINGS * HELMET_WARNING_LEN + 8];

    while (true) {
        hud_model_t m;
        helmet_hud_snapshot(&m);

        lvgl_port_lock(0);
        lv_label_set_text(s_banner, name_for_state(m.safety_state));
        lv_obj_set_style_text_color(s_banner, colour_for_state(m.safety_state), 0);

        lv_bar_set_value(s_battery_bar, m.battery_valid ? m.battery_pct : 0, LV_ANIM_OFF);
        if (m.battery_valid) {
            snprintf(buf, sizeof(buf), "%u%%", (unsigned)m.battery_pct);
        } else {
            snprintf(buf, sizeof(buf), "--%%");
        }
        lv_label_set_text(s_battery_label, buf);

        buf[0] = '\0';
        for (int i = 0; i < m.warning_count; i++) {
            strncat(buf, m.warnings[i], sizeof(buf) - strlen(buf) - 2);
            strncat(buf, "\n", sizeof(buf) - strlen(buf) - 1);
        }
        lv_label_set_text(s_warn_list, buf);

        snprintf(buf, sizeof(buf), "CLOUD %s", m.cloud_up ? "UP" : "--");
        lv_label_set_text(s_link_label, buf);
        snprintf(buf, sizeof(buf), "%.0f m/s", (double)m.ias_ms);
        lv_label_set_text(s_speed_label, buf);
        lvgl_port_unlock();

        vTaskDelay(pdMS_TO_TICKS(HUD_REFRESH_MS));
    }
}

esp_err_t helmet_hud_start(void)
{
    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_io_handle_t io = NULL;
    ESP_RETURN_ON_ERROR(panel_init(&panel, &io), TAG, "panel");

    const lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_RETURN_ON_ERROR(lvgl_port_init(&port_cfg), TAG, "lvgl port");

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io,
        .panel_handle = panel,
        .buffer_size = HELMET_LCD_H_RES * 40,
        .double_buffer = true,
        .hres = HELMET_LCD_H_RES,
        .vres = HELMET_LCD_V_RES,
        .monochrome = false,
        .rotation = { .swap_xy = false, .mirror_x = false, .mirror_y = false },
    };
    s_disp = lvgl_port_add_disp(&disp_cfg);
    if (s_disp == NULL) {
        return ESP_FAIL;
    }
    build_ui();

    /* Core 1 per ARCHITECTURE.md: rendering must not share the comms core. */
    if (xTaskCreatePinnedToCore(hud_task, "helmet_hud", 4096, NULL, 4, NULL, 1) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "HUD up: ST7789 %dx%d", HELMET_LCD_H_RES, HELMET_LCD_V_RES);
    return ESP_OK;
}
