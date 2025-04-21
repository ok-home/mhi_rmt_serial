/* mhi rmt serial

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
#include "freertos/event_groups.h"

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

#include "mhi_rmt_serial.h"

static const char *TAG = "MHI_RMT";

/*
 * Divider set
 */

#define RX_CHANNEL RMT_CHANNEL_1
#define RMT_RX_DIV (80) // 8
#define RMT_RX_IDLE_THRES (20*1000) // 
#define RMT_RX_CLK_OUT (80 * 1000 * 1000 / RMT_RX_DIV) // 1 mks
#define RMT_RX_DELTA (20)
#define RX_INVERT_LVL 0

#if CONFIG_IDF_TARGET_ESP32C3
#define RX_BLOCK_SYMBOL (48*2) 
#define TX_BLOCK_SYMBOL (48*2) 
#endif

#if CONFIG_IDF_TARGET_ESP32
#define RX_BLOCK_SYMBOL (64*2)
#define TX_BLOCK_SYMBOL (64*2)
#endif


#define TX_CHANNEL RMT_CHANNEL_0

#define RMT_TX_DIV (80) // 8 // 1 mks
#define RMT_TX_CLK_OUT (80 * 1000 * 1000 / RMT_TX_DIV) // 1 mks

static QueueHandle_t mhi_rx_packet_queue;
static QueueHandle_t mhi_tx_packet_queue;
static TaskHandle_t mhi_rx_packet_task_handle;
static TaskHandle_t mhi_tx_packet_task_handle;

static rmt_channel_handle_t tx_chan_handle = NULL;
static rmt_encoder_handle_t tx_encoder = NULL;
static rmt_channel_handle_t rx_chan = NULL;
static QueueHandle_t receive_queue;

#define MHI_TX_DONE_BIT BIT0
static EventGroupHandle_t mhi_tx_event_group;

// single rmt item
typedef struct
{
    union
    {
        struct
        {
            uint16_t duration : 15; /*!< Duration of level */
            uint16_t level : 1;     /*!< Level  */
        };
        uint16_t val; /*!< Equivalent unsigned value for the RMT item */
    };
} rmt_item16_t;

// single mhi item
typedef struct
{
    union
    {
        struct
        {
            rmt_item16_t t_l;
            rmt_item16_t t_s;
            rmt_item16_t t_h;
            rmt_item16_t t_end;
        };
        uint64_t val; /*!< Equivalent unsigned value for the RMT item */
    };
} rmt_item64_t;

#define MHL_T_L (52)
#define MHL_T_H (52)
#define MHL_T_S (104)
#define MHL_T_D (1248)

static const rmt_item64_t symbols[8] = {
    {.t_l.level = 0, .t_l.duration = MHL_T_L, .t_s.level = 1, (MHL_T_S*1)-MHL_T_L, .t_h.level = 0, .t_h.duration = MHL_T_H, .t_end.level = 1, .t_end.duration = MHL_T_D-MHL_T_L-MHL_T_H-(MHL_T_S*1) },
    {.t_l.level = 0, .t_l.duration = MHL_T_L, .t_s.level = 1, (MHL_T_S*2)-MHL_T_L, .t_h.level = 0, .t_h.duration = MHL_T_H, .t_end.level = 1, .t_end.duration = MHL_T_D-MHL_T_L-MHL_T_H-(MHL_T_S*2) },
    {.t_l.level = 0, .t_l.duration = MHL_T_L, .t_s.level = 1, (MHL_T_S*3)-MHL_T_L, .t_h.level = 0, .t_h.duration = MHL_T_H, .t_end.level = 1, .t_end.duration = MHL_T_D-MHL_T_L-MHL_T_H-(MHL_T_S*3) },
    {.t_l.level = 0, .t_l.duration = MHL_T_L, .t_s.level = 1, (MHL_T_S*4)-MHL_T_L, .t_h.level = 0, .t_h.duration = MHL_T_H, .t_end.level = 1, .t_end.duration = MHL_T_D-MHL_T_L-MHL_T_H-(MHL_T_S*4) },
    {.t_l.level = 0, .t_l.duration = MHL_T_L, .t_s.level = 1, (MHL_T_S*5)-MHL_T_L, .t_h.level = 0, .t_h.duration = MHL_T_H, .t_end.level = 1, .t_end.duration = MHL_T_D-MHL_T_L-MHL_T_H-(MHL_T_S*5) },
    {.t_l.level = 0, .t_l.duration = MHL_T_L, .t_s.level = 1, (MHL_T_S*6)-MHL_T_L, .t_h.level = 0, .t_h.duration = MHL_T_H, .t_end.level = 1, .t_end.duration = MHL_T_D-MHL_T_L-MHL_T_H-(MHL_T_S*6) },
    {.t_l.level = 0, .t_l.duration = MHL_T_L, .t_s.level = 1, (MHL_T_S*7)-MHL_T_L, .t_h.level = 0, .t_h.duration = MHL_T_H, .t_end.level = 1, .t_end.duration = MHL_T_D-MHL_T_L-MHL_T_H-(MHL_T_S*7) },
    {.t_l.level = 0, .t_l.duration = MHL_T_L, .t_s.level = 1, (MHL_T_S*8)-MHL_T_L, .t_h.level = 0, .t_h.duration = MHL_T_H, .t_end.level = 1, .t_end.duration = MHL_T_D-MHL_T_L-MHL_T_H-(MHL_T_S*8) }
};

static rmt_item64_t rx_items[16*3] = {0};
static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    // return whether any task is woken up
    return high_task_wakeup == pdTRUE;
}

static void mhi_rx_packet_task(void *p)
{
    rmt_rx_done_event_data_t rx_data;
    size_t length = 0;
    mhi_packet_t packet = {};

    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 25*1000,             // the shortest duration
        .signal_range_max_ns = RMT_RX_IDLE_THRES * 1000 // the longest duration
    };
    while (1)
    {
        ESP_ERROR_CHECK(rmt_receive(rx_chan, rx_items, sizeof(rx_items), &receive_config));
        if (xQueueReceive(receive_queue, &rx_data, portMAX_DELAY) == pdTRUE)
        {
            printf("cnt items = %d sizeof = %d \n",rx_data.num_symbols, sizeof(rx_items)/sizeof(rmt_item64_t));
            for(int i=0;i<sizeof(rx_items)/sizeof(rmt_item64_t);i++)
            {
                printf("lvl_l=%d dur_l=%d, lvl_s=%d dur_s=%d lvl_h=%d dur_h=%d lvl_end=%d dur_end=%d dur_byte=%d, dur_all=%d\n",
                    rx_items[i].t_l.level,rx_items[i].t_l.duration,
                    rx_items[i].t_s.level,rx_items[i].t_s.duration,
                    rx_items[i].t_h.level,rx_items[i].t_h.duration,
                    rx_items[i].t_end.level,rx_items[i].t_end.duration,
                    rx_items[i].t_l.duration+t_s.duration,rx_items[i].t_l.duration+t_s.duration+rx_items[i].t_h.duration+t_end.duration 
                )
            }
        // ESP_LOGI(TAG, "all item converted %d byte ",cnt_byte);
        xQueueSend(mhi_rx_packet_queue, &packet, portMAX_DELAY);
        }
    }
}

static void mhi_item_to_rmt_item_cvt(rmt_item16_t *rmt_data, mhi_item16_t data)
{
}

static rmt_symbol_word_t tx_items[(16*3*2)+2]; // 16*4 -> 64 bit ( with 00 end transfer )
static void mhi_tx_packet_task(void *p)
{
    mhi_packet_t packet = {0};
    rmt_transmit_config_t rmt_tx_config = {
        .loop_count = 0,
    };
    while (1)
    {
        xQueueReceive(mhi_tx_packet_queue, &packet, portMAX_DELAY);
        mhi_item_to_rmt_item_cvt(tx_items, &packet);
        ESP_ERROR_CHECK(rmt_transmit(tx_chan_handle, tx_encoder, tx_item, 64, &rmt_tx_config));
        rmt_tx_wait_all_done(tx_chan_handle, portMAX_DELAY);
        xEventGroupSetBits(mhi_tx_event_group, mhi_TX_DONE_BIT);
    }
}

esp_err_t mhi_init(gpio_num_t rx_pin, gpio_num_t tx_pin)
{

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = tx_pin,
        .mem_block_symbols = TX_BLOCK_SYMBOL, //for esp32c3 !!!!!!! esp32s -> 64 !!!!!!! // 16*2 bit = 32 bit
        .flags.invert_out = true,
        .resolution_hz = RMT_TX_CLK_OUT,
        .trans_queue_depth = 5, // set the maximum number of transactions that can pend in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_chan_handle));

    rmt_copy_encoder_config_t tx_encoder_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&tx_encoder_config, &tx_encoder));
    ESP_ERROR_CHECK(rmt_enable(tx_chan_handle));
    //
    mhi_tx_event_group = xEventGroupCreate();
    mhi_tx_packet_queue = xQueueCreate(1, sizeof(mhi_packet_t));
    xTaskCreate(mhi_tx_packet_task, "rmt tx", 4096, NULL, 5, &mhi_tx_packet_task_handle);

    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,  // select source clock
        .resolution_hz = RMT_RX_CLK_OUT, // tick resolution,
        .mem_block_symbols = RX_BLOCK_SYMBOL,// for esp32c3 !!!!!!! esp32s -> 338 !!!!!!!        // memory block size, 338*4 = 676 bytes -> 26 bit in msg * 26 msg in packet =
        .gpio_num = rx_pin,              // GPIO number
        .flags.invert_in = false,        // do not invert input signal
        .flags.with_dma = false,         // do not need DMA backend
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_chan));
    receive_queue = xQueueCreate(10, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, receive_queue));
    ESP_ERROR_CHECK(rmt_enable(rx_chan));
    //
    mhi_rx_packet_queue = xQueueCreate(4, sizeof(mhi_packet_t));
    xTaskCreate(mhi_rx_packet_task, "rmt rx", 4096, NULL, 5, &mhi_rx_packet_task_handle);

    return ESP_OK;
}
esp_err_t mhi_deinit(void)
{
    // deinit RX
    vTaskDelete(mhi_rx_packet_task_handle);

    rmt_disable(rx_chan);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = NULL,
    };
    rmt_rx_register_event_callbacks(rx_chan, &cbs, receive_queue);
    rmt_del_channel(rx_chan);
    vQueueDelete(receive_queue);

    // deinit TX
    vQueueDelete(mhi_rx_packet_queue);

    vTaskDelete(mhi_tx_packet_task_handle);

    rmt_disable(tx_chan_handle);
    rmt_del_encoder(tx_encoder);
    rmt_del_channel(tx_chan_handle);

    vQueueDelete(mhi_tx_packet_queue);
    vEventGroupDelete(mhi_tx_event_group);

    return ESP_OK;
}
void mhi_tx_packet(mhi_packet_t *packet)
{
    xQueueSend(mhi_tx_packet_queue, packet, portMAX_DELAY);                                   // data send to tx queue, start transmit
    xEventGroupWaitBits(mhi_tx_event_group, mhi_TX_DONE_BIT, pdTRUE, pdFALSE, portMAX_DELAY); // all data transmitted
}
esp_err_t mhi_rx_packet(mhi_packet_t *packet, TickType_t wait_time)
{
    int ret = xQueueReceive(mhi_rx_packet_queue, packet, wait_time);
    if (ret != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}
void mhi_clear_rx_queue(void)
{
    xQueueReset(mhi_rx_packet_queue);
}