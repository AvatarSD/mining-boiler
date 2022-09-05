/* Simple HTTP Server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <driver/gpio.h>
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/queue.h>
#include <nvs_flash.h>
#include <sys/param.h>

#include "bdc_motor.h"
#include "boiler.h"
#include "esp32/rom/gpio.h"
#include "esp_netif.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_tls_crypto.h"
#include "freertos/FreeRTOS.h"
#include "fw_config.h"
#include "hw_ow.h"
#include "nvs_flash.h"
#include "ow_temp.h"
#include "protocol_examples_common.h"
#include "ssd1306.h"

#define MAIN_PRIO (1)

/**
 * @brief return uptime string
 *
 * @param uptime
 * @return const char*
 */
const char *esp_uptime_str(uint64_t uptime) {
    static char uptime_str[64];

    snprintf(uptime_str, sizeof(uptime_str), "%llud %lluh %llum %llus",
             uptime / (1000000llu * 3600llu * 24llu),
             (uptime % (1000000llu * 3600llu * 24llu)) / (1000000llu * 3600llu),
             (uptime % (1000000llu * 3600llu)) / (1000000llu * 60llu),
             (uptime % (1000000llu * 60llu)) / (1000000llu));

    return uptime_str;
}

/* An HTTP GET handler */
static esp_err_t main_page_handler(httpd_req_t *req) {
    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Custom-Header-1", "avatarsd");
    httpd_resp_set_hdr(req, "Custom-Header-2", "by S.D.");

    /** @brief Send response with custom headers and body set as the
     * string passed in user context
     * @todo add locks */
    boiler_t boiler_ctx_copy;
    memcpy(&boiler_ctx_copy, req->user_ctx, sizeof(boiler_t));

    // curr is 217bytes
    char resp_str[256+128] = {0};
    uint64_t uptime = esp_timer_get_time();
    snprintf(resp_str, sizeof(resp_str),
             "Worker0 IN: %03.2f`C, OUT: %03.2f`C, Flow: %fliter/s;\t\t\t"
             "Coolant IN: %03.2f`C, OUT: %03.2f`C, Flow: %fliter/s\t\t\t"
             "Boiler  IN: %03.2f`C, OUT: %03.2f`C, Flow: %fliter/s\t\t\t"
             "Fan: %3.2f%%, Worker: %s;\t\t\tCycles: %llu; Uptime: %llius(%llus) or %s",
             boiler_ctx_copy.workers[0].termal.temp[DIR_IN].temp,
             boiler_ctx_copy.workers[0].termal.temp[DIR_OUT].temp,
             boiler_ctx_copy.workers[0].termal.flow.flow, boiler_ctx_copy.colant.temp[DIR_IN].temp,
             boiler_ctx_copy.colant.temp[DIR_OUT].temp, boiler_ctx_copy.colant.flow.flow,
             boiler_ctx_copy.boiler.temp[DIR_IN].temp, boiler_ctx_copy.boiler.temp[DIR_OUT].temp,
             boiler_ctx_copy.boiler.flow.flow, boiler_ctx_copy.cooler_motor_pwr * 100,
             boiler_ctx_copy.workers[0].enabled ? "Enabled" : "Disabled", boiler_ctx_copy.cycles,
             uptime, uptime / 1000000llu, esp_uptime_str(uptime));
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err) {
    /* For any other URI send 404 and close socket */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404");
    return ESP_FAIL;
}

static httpd_uri_t hello = {.uri = "/",
                            .method = HTTP_GET,
                            .handler = main_page_handler,

                            .user_ctx = NULL};

static httpd_handle_t start_webserver(boiler_t *boiler_ctx) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    hello.user_ctx = boiler_ctx;

    // Start the httpd server
    ESP_LOGI(__func__, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(__func__, "Registering URI handlers");
        httpd_register_uri_handler(server, &hello);
        httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);

        return server;
    }

    ESP_LOGI(__func__, "Error starting server!");
    return NULL;
}

static esp_err_t stop_webserver(httpd_handle_t server) {
    // Stop the httpd server
    return httpd_stop(server);
}

static void disconnect_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                               void *event_data) {
    boiler_t *server = (boiler_t *)arg;
    if (server->server) {
        ESP_LOGI(__func__, "Stopping webserver");
        if (stop_webserver(server->server) == ESP_OK) {
            server->server = NULL;
        } else {
            ESP_LOGE(__func__, "Failed to stop http server");
        }
    }
}

static void connect_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                            void *event_data) {
    boiler_t *server = (boiler_t *)arg;
    if (server->server == NULL) {
        ESP_LOGI(__func__, "Starting webserver");
        server->server = start_webserver(server);
    }
}

/********** main logic **********
 * @todo split plain code into oop abstractions
 */

/**
 *  @brief read and display sensors data
 */
void read_and_draw_sensors(hw_ow_t *hw_ow, temp_sensor_t *sensors_arr, uint8_t sensors_arr_count,
                           ssd1306_handle_t *display, uint8_t draw_y_offset) {
    if (!display) {
        ESP_LOGE(__func__, "no display");
        return;
    }

    /** @todo remove along with scan/read splitting */
    static rom_t sensors_rom_arr[POINT_MAX];
    int sensors = ow_temp_search_all_temp_sensors(hw_ow, sensors_rom_arr, POINT_MAX);
    if (!sensors) {
        ESP_LOGW(__func__, "Zero sensors founded, continue");
        ssd1306_draw_string(display, 0, draw_y_offset, (const uint8_t *)"   =( no sensors", 16, 1);

        /** @todo remove later */
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return;
    }
    ESP_LOGD(__func__, "Sensors count: %i", sensors);

    for (uint8_t i = 0; i < sensors_arr_count; i++) {
        char data_str[24] = "---";
        if (rom_is_null(&sensors_rom_arr[i]) || i >= sensors) {
            rom_zeroing(&sensors_arr[i].rom);
            sensors_arr->temp = -127;
            ssd1306_draw_string(display, 0, draw_y_offset + 16 * i, (const uint8_t *)data_str, 16,
                                1);
            continue;
        }
        sensors_arr[i].rom.raw = sensors_rom_arr[i].raw;
        char *rom_name = (char *)rom_to_string(&sensors_rom_arr[i]);
        rom_name[6] = '\0';  // cut rom
        for (uint8_t n = 0; n < 3; n++) {
            if (ow_temp_read_sensor(hw_ow, &sensors_arr[i].rom, &sensors_arr[i].temp)) {
                snprintf(data_str, sizeof(data_str), "%s: %03.2f`C", rom_name, sensors_arr[i].temp);
                ESP_LOGI(__func__, "%s", data_str);
                break;
            } else if (n == 3 - 1) {
                snprintf(data_str, sizeof(data_str), "%s: error", rom_name);
                ESP_LOGW(__func__, "%s", data_str);
            }
        }
        ssd1306_draw_string(display, 0, draw_y_offset + 16 * i, (const uint8_t *)data_str, 16, 1);
    }
}

struct counter_handler_t {
    uint32_t last_calc;
    QueueHandle_t evt_queue;
};

static struct counter_handler_t queues[] = {
    {.last_calc = 0, .evt_queue = NULL},
    {.last_calc = 0, .evt_queue = NULL},
    {.last_calc = 0, .evt_queue = NULL},
};

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t time = (uint32_t)esp_timer_get_time();
    const uint8_t queues_items = sizeof(queues) / sizeof(struct counter_handler_t);
    uint32_t item = (uint32_t)arg;
    if (item >= queues_items) return;
    uint32_t last_time = queues[item].last_calc;
    uint32_t diff_time = last_time < time ? time - last_time : time + (UINT32_MAX - last_time);
    queues[item].last_calc = time;
    uint32_t flow = !diff_time ? 0 : (1000000000000) / (diff_time * CNT_TIC_L);
    xQueueSendFromISR(queues[item].evt_queue, &flow, NULL);
}

void flow_calc_task(void *ctx) {
    /* init */
    if (!ctx) {
        ESP_LOGW(__func__, "ctx error");
        return;
    }

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    boiler_t *boiler = (boiler_t *)ctx;

    for (uint8_t i = 0; i < sizeof(queues) / sizeof(struct counter_handler_t); i++)
        queues[i].evt_queue = xQueueCreate(1, sizeof(uint32_t));

    uint64_t counters_pins =
        ((1ull << GPIO_CNT_CLNT) | (1ull << GPIO_CNT_BOIL) | (1ull << GPIO_CNT_WRK0));

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    io_conf.pin_bit_mask = counters_pins;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    /// @todo put pointer on boiler structs
    gpio_isr_handler_add(GPIO_CNT_CLNT, gpio_isr_handler, (void *)0);
    gpio_isr_handler_add(GPIO_CNT_BOIL, gpio_isr_handler, (void *)1);
    gpio_isr_handler_add(GPIO_CNT_WRK0, gpio_isr_handler, (void *)2);

    for (;;) {
        uint32_t last;
        if (xQueueReceive(queues[0].evt_queue, &last, 10)) {
            ESP_LOGD(__func__, "Counter(%d) flow: %fl/s\n", GPIO_CNT_CLNT, ((float)last) / 1000000);
            boiler->colant.flow.flow = ((float)last) / 1000000;
        } else
            boiler->colant.flow.flow = 0;
        if (xQueueReceive(queues[1].evt_queue, &last, 10)) {
            ESP_LOGD(__func__, "Counter(%d) flow: %fl/s\n", GPIO_CNT_BOIL, ((float)last) / 1000000);
            boiler->boiler.flow.flow = ((float)last) / 1000000;
        } else
            boiler->boiler.flow.flow = 0;
        if (xQueueReceive(queues[2].evt_queue, &last, 10)) {
            ESP_LOGD(__func__, "Counter(%d) flow: %fl/s\n", GPIO_CNT_WRK0, ((float)last) / 1000000);
            boiler->workers[0].termal.flow.flow = ((float)last) / 1000000;
        } else
            boiler->workers[0].termal.flow.flow = 0;
        esp_task_wdt_reset();
    }
}

/**
 *  @brief main task for health logic
 *
 */
void temp_mon_task(void *ctx) {
    /* init */
    if (!ctx) {
        ESP_LOGW(__func__, "ctx error");
        return;
    }

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    for (;;) {
        /* alloc new hardware 1-wire */
        hw_ow_t *hw_ow = hw_ow_new(UART_HW_OW_NUM, GPIO_HW_OW_TX, GPIO_HW_OW_RX, GPIO_HW_OW_EN);
        if (!hw_ow) {
            ESP_LOGW(__func__, "hw_ow allocation error");
            continue;
        }

        hw_ow_change_baud(hw_ow, PARMSET_115200);

        /* screen init */
        ssd1306_handle_t ssd1306_dev = NULL;

        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = (gpio_num_t)GPIO_MASTER_SDA;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_io_num = (gpio_num_t)GPIO_MASTER_SCL;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
        conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

        i2c_param_config(I2C_MASTER_NUM, &conf);
        i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

        ssd1306_dev = ssd1306_create(I2C_MASTER_NUM, SSD1306_I2C_ADDRESS);
        ssd1306_refresh_gram(ssd1306_dev);
        ssd1306_clear_screen(ssd1306_dev, 0x00);

        /* i/o gpio init */
        uint64_t out_pins_sel =
            ((1ull << GPIO_TEMP_WRK0_IN) | (1ull << GPIO_TEMP_WRK0_OUT) |
             (1ull << GPIO_TEMP_CLNT_IN) | (1ull << GPIO_TEMP_CLNT_OUT) |
             (1ull << GPIO_TEMP_BOIL_BOTH) | (1ull << GPIO_TEMP_COLER) | (1ull << GPIO_TEMP_AUX0) |
             (1ull << GPIO_CTRL_WRK0) | (1ull << GPIO_CTRL_PUMP_PRI) |
             (1ull << GPIO_CTRL_PUMP_SEC) | (1ull << GPIO_CTRL_FAN0) | (1ull << GPIO_CTRL_FAN1));

#ifdef CONFIG_ESP_DEBUG_STUBS_ENABLE
        /* free jtag pins */
        out_pins_sel &= ~((1ull << 12) | (1ull << 13) | (1ull << 14) | (1ull << 14));
#endif
        // zero-initialize the config structure.
        gpio_config_t io_conf = {};
        // disable interrupt
        io_conf.intr_type = GPIO_INTR_DISABLE;
        // set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        // bit mask of the pins that you want to set,e.g.GPIO18/19
        io_conf.pin_bit_mask = out_pins_sel;
        // disable pull-down mode
        io_conf.pull_down_en = 0;
        // disable pull-up mode
        io_conf.pull_up_en = 0;
        // configure GPIO with the given settings
        gpio_config(&io_conf);

        /**
         * @brief main loop cycle counter
         * @todo create common struct
         */
        boiler_t *boiler = (boiler_t *)ctx;

        /* fan startup test */
        bdc_motor_enable(boiler->cooler_motor);
        bdc_motor_set_speed(boiler->cooler_motor, 10);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        bdc_motor_set_speed(boiler->cooler_motor, 0);
        bdc_motor_set_speed(boiler->cooler_motor, 200);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        bdc_motor_set_speed(boiler->cooler_motor, 0);

        /* assump liquid level is okey */
        gpio_set_level(GPIO_CTRL_PUMP_PRI, 1);
        gpio_set_level(GPIO_CTRL_PUMP_SEC, 1);

        /* loop */
        for (;;) {
            /* display buf */
            char data_str[64] = {0};

            /**
             * @todo Split temp sensors and main logic tasks
             *       Temp sensors task split INTO TWO procedures:
             *         * sensors reading
             *         * sensors scanning
             *       Split display and temp sens into separate tasks
             */

            /* draw worker in temp */
            ssd1306_clear_screen(ssd1306_dev, 0x00);
            ssd1306_draw_string(ssd1306_dev, 0, 0,
                                (const uint8_t *)esp_uptime_str(esp_timer_get_time()), 16, 1);
            sprintf(data_str, "WRK0 I(1/7) %4llu", boiler->cycles % 10000);
            ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)data_str, 16, 1);
            /* read and draw sensors*/
            gpio_set_level(GPIO_TEMP_WRK0_IN, 1);
            read_and_draw_sensors(hw_ow, &boiler->workers[0].termal.temp[DIR_IN], 1, ssd1306_dev,
                                  32);
            gpio_set_level(GPIO_TEMP_WRK0_IN, 0);
            /* draw out last frame */
            int ret = ssd1306_refresh_gram(ssd1306_dev);
            if (ret) {
                ESP_LOGI(__func__, "Count: %llu, gram ret: %i", boiler->cycles, ret);
                break;
            }

            /* draw worker out temp */
            ssd1306_clear_screen(ssd1306_dev, 0x00);
            ssd1306_draw_string(ssd1306_dev, 0, 0,
                                (const uint8_t *)esp_uptime_str(esp_timer_get_time()), 16, 1);
            sprintf(data_str, "WRK0 O(2/7) %4llu", boiler->cycles % 10000);
            ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)data_str, 16, 1);
            /* read and draw sensors*/
            gpio_set_level(GPIO_TEMP_WRK0_OUT, 1);
            read_and_draw_sensors(hw_ow, &boiler->workers[0].termal.temp[DIR_OUT], 1, ssd1306_dev,
                                  32);
            gpio_set_level(GPIO_TEMP_WRK0_OUT, 0);
            /* draw out last frame */
            ret = ssd1306_refresh_gram(ssd1306_dev);
            if (ret) {
                ESP_LOGI(__func__, "Count: %llu, gram ret: %i", boiler->cycles, ret);
                break;
            }

            /* draw coolant in temp */
            ssd1306_clear_screen(ssd1306_dev, 0x00);
            ssd1306_draw_string(ssd1306_dev, 0, 0,
                                (const uint8_t *)esp_uptime_str(esp_timer_get_time()), 16, 1);
            sprintf(data_str, "CLNT I(3/7) %4llu", boiler->cycles % 10000);
            ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)data_str, 16, 1);
            /* read and draw sensors*/
            gpio_set_level(GPIO_TEMP_CLNT_IN, 1);
            read_and_draw_sensors(hw_ow, &boiler->colant.temp[DIR_IN], 1, ssd1306_dev, 32);
            gpio_set_level(GPIO_TEMP_CLNT_IN, 0);
            /* draw out last frame */
            ret = ssd1306_refresh_gram(ssd1306_dev);
            if (ret) {
                ESP_LOGI(__func__, "Count: %llu, gram ret: %i", boiler->cycles, ret);
                break;
            }

            /* draw coolant out temp */
            ssd1306_clear_screen(ssd1306_dev, 0x00);
            ssd1306_draw_string(ssd1306_dev, 0, 0,
                                (const uint8_t *)esp_uptime_str(esp_timer_get_time()), 16, 1);
            sprintf(data_str, "CLNT O(4/7) %4llu", boiler->cycles % 10000);
            ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)data_str, 16, 1);
            /* read and draw sensors*/
            gpio_set_level(GPIO_TEMP_CLNT_OUT, 1);
            read_and_draw_sensors(hw_ow, &boiler->colant.temp[DIR_OUT], 1, ssd1306_dev, 32);
            gpio_set_level(GPIO_TEMP_CLNT_OUT, 0);
            /* draw out last frame */
            ret = ssd1306_refresh_gram(ssd1306_dev);
            if (ret) {
                ESP_LOGI(__func__, "Count: %llu, gram ret: %i", boiler->cycles, ret);
                break;
            }

            /* draw boiler both sensors temp */
            ssd1306_clear_screen(ssd1306_dev, 0x00);
            ssd1306_draw_string(ssd1306_dev, 0, 0,
                                (const uint8_t *)esp_uptime_str(esp_timer_get_time()), 16, 1);
            sprintf(data_str, "BOILER(5/7) %4llu", boiler->cycles % 10000);
            ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)data_str, 16, 1);
            /* read and draw sensors*/
            gpio_set_level(GPIO_TEMP_BOIL_BOTH, 1);
            read_and_draw_sensors(hw_ow, boiler->boiler.temp,
                                  sizeof(boiler->boiler.temp) / sizeof(temp_sensor_t), ssd1306_dev,
                                  32);
            gpio_set_level(GPIO_TEMP_BOIL_BOTH, 0);
            /* draw out last frame */
            ret = ssd1306_refresh_gram(ssd1306_dev);
            if (ret) {
                ESP_LOGI(__func__, "Count: %llu, gram ret: %i", boiler->cycles, ret);
                break;
            }
            /* draw coler temp */
            ssd1306_clear_screen(ssd1306_dev, 0x00);
            ssd1306_draw_string(ssd1306_dev, 0, 0,
                                (const uint8_t *)esp_uptime_str(esp_timer_get_time()), 16, 1);
            sprintf(data_str, "COOLER(6/7) %4llu", boiler->cycles % 10000);
            ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)data_str, 16, 1);
            /* read and draw sensors*/
            gpio_set_level(GPIO_TEMP_COLER, 1);
            read_and_draw_sensors(hw_ow, boiler->cooler_temps,
                                  sizeof(boiler->cooler_temps) / sizeof(temp_sensor_t), ssd1306_dev,
                                  32);
            gpio_set_level(GPIO_TEMP_COLER, 0);
            /* draw out last frame */
            ret = ssd1306_refresh_gram(ssd1306_dev);
            if (ret) {
                ESP_LOGI(__func__, "Count: %llu, gram ret: %i", boiler->cycles, ret);
                break;
            }
            /* draw aux temp */
            ssd1306_clear_screen(ssd1306_dev, 0x00);
            ssd1306_draw_string(ssd1306_dev, 0, 0,
                                (const uint8_t *)esp_uptime_str(esp_timer_get_time()), 16, 1);
            sprintf(data_str, "AUX0  (7/7) %4llu", boiler->cycles % 10000);
            ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)data_str, 16, 1);
            /* read and draw sensors*/
            gpio_set_level(GPIO_TEMP_AUX0, 1);
            read_and_draw_sensors(hw_ow, boiler->aux_temps,
                                  sizeof(boiler->aux_temps) / sizeof(temp_sensor_t), ssd1306_dev,
                                  32);
            gpio_set_level(GPIO_TEMP_AUX0, 0);
            /* draw out last frame */
            ret = ssd1306_refresh_gram(ssd1306_dev);
            if (ret) {
                ESP_LOGI(__func__, "Count: %llu, gram ret: %i", boiler->cycles, ret);
                break;
            }

            /* calc flow */

            /* calc cooler */
            boiler->cooler_motor_pwr =
                boiler->colant.temp[DIR_IN].temp > 30              /* start temp */
                    ? (boiler->colant.temp[DIR_IN].temp - 30) / 20 /* divide by temp range */
                    : 0;
            bdc_motor_set_speed(
                boiler->cooler_motor,
                boiler->cooler_motor_pwr * BDC_MCPWM_DUTY_TICK_MAX); /* multiply by max duty */

            /* calc worker safe zone */
            bool worker_en0 = /*boiler->workers[0].termal.flow.flow > 0.03 &&*/
                              boiler->workers[0].termal.temp[DIR_IN].temp < 50 &&
                              boiler->workers[0].termal.temp[DIR_OUT].temp < 60;
            if (worker_en0 != boiler->workers[0].enabled) {
                boiler->workers[0].enabled = worker_en0;
                ESP_LOGI(__func__, "Worker 0: %s",
                         boiler->workers[0].enabled ? "Enabled" : "Disabled");
            }
            gpio_set_level(GPIO_CTRL_WRK0, boiler->workers[0].enabled);

            /* calc fans */
            gpio_set_level(
                GPIO_CTRL_FAN0,
                (boiler->cooler_temps[DIR_IN].temp + boiler->cooler_temps[DIR_OUT].temp) / 2 > 35);
            gpio_set_level(
                GPIO_CTRL_FAN1,
                (boiler->aux_temps[DIR_IN].temp + boiler->aux_temps[DIR_OUT].temp) / 2 > 35);

            /* cycle */
            ++boiler->cycles;
            esp_task_wdt_reset();
        }

        ESP_LOGE(__func__, "Main loop exit, restarting(%llu times)...", ++boiler->loop_err);
        gpio_set_level(GPIO_CTRL_PUMP_PRI, 0);
        gpio_set_level(GPIO_CTRL_PUMP_SEC, 0);
        bdc_motor_disable(boiler->cooler_motor);
        ssd1306_delete(ssd1306_dev);
        hw_ow_delete(hw_ow);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    static boiler_t boiler = {0};

    ESP_ERROR_CHECK(nvs_flash_init());

    /* run health monitor */
    TaskHandle_t temp_mon_handler = NULL;
    xTaskCreate(temp_mon_task, "temp_reader", 8192, &boiler, MAIN_PRIO, &temp_mon_handler);
    xTaskCreate(flow_calc_task, "flow_calc_task", 8192, &boiler, MAIN_PRIO, NULL);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    /* Register event handlers to stop the server when Wi-Fi is disconnected,
     * and re-start it upon connection.
     */
    ESP_ERROR_CHECK(
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &boiler));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
                                               &disconnect_handler, &boiler));

    /* bdc motor handle */
    ESP_LOGI(__func__, "Create DC motor");
    bdc_motor_config_t motor_config = {.pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
                                       .pwma_gpio_num = GPIO_CTRL_COOLER,
                                       .pwmb_gpio_num = GPIO_CTRL_COOLER};
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &boiler.cooler_motor));

    /* Start the server for the first time */
    boiler.server = start_webserver(&boiler);
}
