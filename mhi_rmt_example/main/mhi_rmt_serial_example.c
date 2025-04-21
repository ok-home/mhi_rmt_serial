/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include <string.h>

#include "freertos/queue.h"
#include "esp_clk_tree.h"
#include <rom/ets_sys.h>

#include "mhi_rmt_serial.h"

#include "soc/gpio_sig_map.h"
#include "soc/gpio_periph.h"
#include "soc/io_mux_reg.h"
#define gpio_matrix_in(a, b, c) esp_rom_gpio_connect_in_signal(a, b, c)
#define gpio_matrix_out(a, b, c, d) esp_rom_gpio_connect_out_signal(a, b, c, d)

#define DBG 1

#if CONFIG_IDF_TARGET_ESP32C3
#define RMT_TX_GPIO (3)//esp32 ->(23)
#define RMT_RX_GPIO (4)//esp32 ->(22)
#define TX_TEST_GPIO (5)//for c3 !! esp32->(25)
#endif
#if CONFIG_IDF_TARGET_ESP32
#define RMT_TX_GPIO (23)//esp32 ->(23)
#define RMT_RX_GPIO (22)//esp32 ->(22)
#define TX_TEST_GPIO (25)//for c3 !! esp32->(25)
#endif

static const char *TAG = "RMT TEST";

// dbg logic analyzer
#include "logic_analyzer_ws_server.h"
void app_main(void)
{
    mhi_packet_t tx_packet =
            {
                {.raw_data = 0x23},
                {.raw_data = 0x00},
                {.raw_data = 0xff},
                {.raw_data = 0xaa},
                {.raw_data = 0x55},
                {.raw_data = 0x1f},
                {.raw_data = 0xf1},
                {.raw_data = 0x0f},
                {.raw_data = 0xf0},
                {.raw_data = 0xbb},
                {.raw_data = 0xcc},
                {.raw_data = 0x01},
                {.raw_data = 0x80},
                {.raw_data = 0x03},
                {.raw_data = 0xc0},
                {.raw_data = 0x00},
            };

#if DBG

    logic_analyzer_ws_server();
    //
    //  dbg GPIO PIN
    gpio_reset_pin(TX_TEST_GPIO);
    gpio_set_direction(TX_TEST_GPIO, GPIO_MODE_OUTPUT);
    //
#endif
    mhi_init(RMT_RX_GPIO, RMT_TX_GPIO);
#if DBG
    // connect rx & tx pins without wires -> test only
    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[RMT_TX_GPIO]);
#if CONFIG_IDF_TARGET_ESP32C3
    gpio_matrix_in(RMT_TX_GPIO, RMT_SIG_IN0_IDX, false);//esp32->RMT_SIG_IN1_IDX
#endif
#if CONFIG_IDF_TARGET_ESP32
    gpio_matrix_in(RMT_TX_GPIO, RMT_SIG_IN2_IDX, false);//esp32->RMT_SIG_IN2_IDX
#endif

#endif
   ESP_LOGI(TAG, "Size1 tx=%d", sizeof(tx_packet)/sizeof(uint8_t));

    mhi_packet_t rx_packet = {0};
    int cnt = 0;
    int icnt = 0;
    int err_cnt = 0;

    while (1)
    {
        vTaskDelay(200);
#if DBG
        gpio_set_level(TX_TEST_GPIO, 1);
#endif
        mhi_tx_packet(&tx_packet);
        mhi_rx_packet(&rx_packet,portMAX_DELAY);
#if DBG
        gpio_set_level(TX_TEST_GPIO, 0);
#endif

#if 0
        for (int i = 0; i < sizeof(tx_packet); i++)
        {
            if (tx_packet[i].raw_data != rx_packet[i].raw_data)
            {
                ESP_LOGE(TAG, "ERROR Packet idx= %d tx %x rx %x err_cnt=%d", i, tx_packet.packet_data[i].data, rx_packet.packet_data[i].data, err_cnt++);
            }
        }
        memset((void *)&rx_packet, 0, sizeof(rx_packet));
        cnt++;
        if (cnt >= 10)
        {
            ESP_LOGI(TAG, "Send/Receive %d packet icnt=%d", cnt, icnt++);
            cnt = 0;
        }
#endif
    }
}