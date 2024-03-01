/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"

#include "driver/uart.h"
#include "esp_vfs_dev.h"

#include "lvgl.h"

#include "esp_lcd_ili9341.h"

static const char *TAG = "myapp";

// Using SPI2 in the example
#define LCD_HOST  SPI2_HOST

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ILI9341_LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)
#define ILI9341_LCD_BK_LIGHT_ON_LEVEL  1
#define ILI9341_LCD_BK_LIGHT_OFF_LEVEL !ILI9341_LCD_BK_LIGHT_ON_LEVEL
#define ILI9341_PIN_NUM_SCLK           18
#define ILI9341_PIN_NUM_MOSI           19
#define ILI9341_PIN_NUM_MISO           21
#define ILI9341_PIN_NUM_LCD_DC         5
#define ILI9341_PIN_NUM_LCD_RST        3
#define ILI9341_PIN_NUM_LCD_CS         4
#define ILI9341_PIN_NUM_BK_LIGHT       2

#define ILI9341_PIN_NUM_TOUCH_CS       15
#define ILI9341_PIN_NUM_TOUCH_IRQ      10

// The pixel number in horizontal and vertical
#define ILI9341_LCD_H_RES              240
#define ILI9341_LCD_V_RES              320
// Bit number used to represent command and parameter
#define ILI9341_LCD_CMD_BITS           8
#define ILI9341_LCD_PARAM_BITS         8

#define ILI9341_LVGL_TICK_PERIOD_MS    2
#define ILI9341_LVGL_TASK_MAX_DELAY_MS 500
#define ILI9341_LVGL_TASK_MIN_DELAY_MS 1
#define ILI9341_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define ILI9341_LVGL_TASK_PRIORITY     2

#define CANVAS_WIDTH  220
#define CANVAS_HEIGHT 260

#define UART_NUM                UART_NUM_1
#define UART1_REMAP_TXD_PIN     15
#define UART1_REMAP_RXD_PIN     16
#define UART1_BUFFER_SIZE       512
#define UART1_BAND_RATE         9600


static SemaphoreHandle_t lvgl_mux = NULL;



static bool ILI9341_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void ILI9341_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static void ILI9341_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(ILI9341_LVGL_TICK_PERIOD_MS);
}

static void my_draw_triangle_direction(lv_obj_t * canvas, int angle, lv_color_t color)
{
    lv_point_t points[3];
    int base_length = 8; // 三角形的底边长度
    int side_length = 15; // 三角形的两边长度
    int height = (int)(sqrt(side_length * side_length - base_length * base_length / 4)); // 三角形的高


// 计算三角形的三个顶点的位置
    points[0].x = CANVAS_WIDTH / 2;
    points[0].y = CANVAS_HEIGHT / 2 - height / 2;
    points[1].x = CANVAS_WIDTH / 2 - base_length / 2;
    points[1].y = CANVAS_HEIGHT / 2 + height / 2;
    points[2].x = CANVAS_WIDTH / 2 + base_length / 2;
    points[2].y = CANVAS_HEIGHT / 2 + height / 2;

// 旋转顶点
    for(int i = 0; i < 3; i++) {
        int dx = points[i].x - CANVAS_WIDTH / 2;
        int dy = points[i].y - CANVAS_HEIGHT / 2;
        points[i].x = dx * cos(angle * M_PI / 180) - dy * sin(angle * M_PI / 180) + CANVAS_WIDTH / 2;
        points[i].y = dx * sin(angle * M_PI / 180) + dy * cos(angle * M_PI / 180) + CANVAS_HEIGHT / 2;
    }

    lv_draw_rect_dsc_t draw_dsc;
    lv_draw_rect_dsc_init(&draw_dsc);
    draw_dsc.bg_color = color; // 设置颜色
    draw_dsc.bg_opa = LV_OPA_COVER;

    lv_canvas_draw_polygon(canvas, points, 3, &draw_dsc);

}

void ILI9341_lvgl_main_task(lv_disp_t *disp)
{
    lv_obj_t *scr = lv_disp_get_scr_act(disp);

    lv_style_t style;
    lv_style_init(&style);

    lv_style_set_text_font(&style, &lv_font_montserrat_16);

    lv_obj_t *canvas = lv_canvas_create(scr);
    static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR(CANVAS_WIDTH, CANVAS_HEIGHT)];
    lv_canvas_set_buffer(canvas, cbuf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(canvas, LV_ALIGN_TOP_MID, 0, 10);
    lv_canvas_fill_bg(canvas, lv_palette_lighten(LV_PALETTE_BLUE,1), LV_OPA_COVER);

    lv_color_t color = lv_palette_main(LV_PALETTE_RED); // 设置颜色
    for(int x = 0; x < CANVAS_WIDTH; x += 5) 
    {
        lv_canvas_set_px(canvas, x, CANVAS_HEIGHT/2, color);
    }

    for(int y = 0; y < CANVAS_HEIGHT; y += 5) 
    {
        lv_canvas_set_px(canvas, CANVAS_WIDTH/2, y, color);
    }

    my_draw_triangle_direction(canvas,0,lv_palette_main(LV_PALETTE_YELLOW));


}

bool ILI9341_lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void ILI9341_lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void ILI9341_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = ILI9341_LVGL_TASK_MAX_DELAY_MS;

    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (ILI9341_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            ILI9341_lvgl_unlock();
        }
        if (task_delay_ms > ILI9341_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = ILI9341_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < ILI9341_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = ILI9341_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}



void app_main(void)
{
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << ILI9341_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = ILI9341_PIN_NUM_SCLK,
        .mosi_io_num = ILI9341_PIN_NUM_MOSI,
        .miso_io_num = ILI9341_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ILI9341_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = ILI9341_PIN_NUM_LCD_DC,
        .cs_gpio_num = ILI9341_PIN_NUM_LCD_CS,
        .pclk_hz = ILI9341_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = ILI9341_LCD_CMD_BITS,
        .lcd_param_bits = ILI9341_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = ILI9341_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = ILI9341_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_LOGI(TAG, "Install ILI9341 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(ILI9341_PIN_NUM_BK_LIGHT, ILI9341_LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(ILI9341_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(ILI9341_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, ILI9341_LCD_H_RES * 20);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = ILI9341_LCD_H_RES;
    disp_drv.ver_res = ILI9341_LCD_V_RES;
    disp_drv.flush_cb = ILI9341_lvgl_flush_cb;

//当LVGL参数被修改时，需要调用lvgl_port_update_callback函数来更新LVGL参数，在本程序中调用这个函数，是因为：
//我们按下了rotate按钮，这样我们就需要旋转屏幕，这就把LVGL的参数修改了，所以要调用这个函数来更新LVGL参数。
//最新的程序中，我们不需要调用这个函数，因为我们不会修改LVGL的参数。
//    disp_drv.drv_update_cb = ILI9341_lvgl_port_update_callback;

    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

///////////////////////////////
// 至此，我们已经完成了LCD的驱动安装和LVGL的初始化，接下来我们需要创建一个任务来运行LVGL。
///////////////////////////////


// 初始化UART1，并将TXD和RXD引脚重映射，来和GPS模块通信
    ESP_LOGI(TAG, "Initialize UART1, Remap TXD and RXD pins");
    uart_config_t uart_config = {
        .baud_rate = UART1_BAND_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART1_REMAP_TXD_PIN, UART1_REMAP_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, UART1_BUFFER_SIZE, 0, 0, NULL, 0);




    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &ILI9341_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, ILI9341_LVGL_TICK_PERIOD_MS * 1000));

    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);
    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(ILI9341_lvgl_port_task, "LVGL", ILI9341_LVGL_TASK_STACK_SIZE, NULL, ILI9341_LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL Meter Widget");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (ILI9341_lvgl_lock(-1)) {
        ILI9341_lvgl_main_task(disp);
        // Release the mutex
        ILI9341_lvgl_unlock();
    }
    while(1);
}
