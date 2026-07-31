/* The HUD's shared model. Written by comms, safety_glue and ESP-NOW; read by
 * hud_task. One mutex, snapshot-on-read, so rendering never tears. */
#include "helmet_tasks.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <string.h>

static hud_model_t s_model;
static SemaphoreHandle_t s_lock;
static StaticSemaphore_t s_lock_buf;

static void ensure_lock(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
    }
}

void helmet_hud_snapshot(hud_model_t *out)
{
    ensure_lock();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_model;
    xSemaphoreGive(s_lock);
}

void helmet_hud_set_state(uint8_t safety_state)
{
    ensure_lock();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_model.safety_state = safety_state;
    xSemaphoreGive(s_lock);
}

void helmet_hud_set_battery(uint8_t pct)
{
    ensure_lock();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_model.battery_pct = pct;
    s_model.battery_valid = true;
    xSemaphoreGive(s_lock);
}

void helmet_hud_set_cloud(bool up)
{
    ensure_lock();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_model.cloud_up = up;
    xSemaphoreGive(s_lock);
}

void helmet_hud_push_warning(const char *text)
{
    if (text == NULL) {
        return;
    }
    ensure_lock();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_model.warning_count >= HELMET_MAX_WARNINGS) {
        /* Drop the oldest: a full list must not hide the newest problem. */
        memmove(&s_model.warnings[0], &s_model.warnings[1],
                (HELMET_MAX_WARNINGS - 1) * HELMET_WARNING_LEN);
        s_model.warning_count = HELMET_MAX_WARNINGS - 1;
    }
    strncpy(s_model.warnings[s_model.warning_count], text, HELMET_WARNING_LEN - 1);
    s_model.warnings[s_model.warning_count][HELMET_WARNING_LEN - 1] = '\0';
    s_model.warning_count++;
    xSemaphoreGive(s_lock);
}

void helmet_hud_clear_warnings(void)
{
    ensure_lock();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_model.warning_count = 0;
    xSemaphoreGive(s_lock);
}
