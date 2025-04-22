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

#define RMT_RX_DIV (80) // 8
#define RMT_RX_IDLE_THRES (20*1000) // 
#define RMT_RX_CLK_OUT (80 * 1000 * 1000 / RMT_RX_DIV) // 1 mks
#define RMT_RX_DELTA (20)
#define HBS_GLITCH 5


#if CONFIG_IDF_TARGET_ESP32C3
#define RX_BLOCK_SYMBOL (48*2) 
#define TX_BLOCK_SYMBOL (48*2) 
#endif

#if CONFIG_IDF_TARGET_ESP32
#define RX_BLOCK_SYMBOL (64*2)
#define TX_BLOCK_SYMBOL (64*2)
#endif


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

#define MHI_T_L (52)
#define MHI_T_H (52)
#define MHI_T_S (104)
#define MHI_T_D (1248)
static const rmt_item64_t symbols[8] = {
    {.t_l.level = 1, .t_l.duration = MHI_T_L, .t_s.level = 0, .t_s.duration = (MHI_T_S*1)-MHI_T_L, .t_h.level = 1, .t_h.duration = MHI_T_H, .t_end.level = 0, .t_end.duration = MHI_T_D-MHI_T_H-(MHI_T_S*1) },
    {.t_l.level = 1, .t_l.duration = MHI_T_L, .t_s.level = 0, .t_s.duration = (MHI_T_S*2)-MHI_T_L, .t_h.level = 1, .t_h.duration = MHI_T_H, .t_end.level = 0, .t_end.duration = MHI_T_D-MHI_T_H-(MHI_T_S*2) },
    {.t_l.level = 1, .t_l.duration = MHI_T_L, .t_s.level = 0, .t_s.duration = (MHI_T_S*3)-MHI_T_L, .t_h.level = 1, .t_h.duration = MHI_T_H, .t_end.level = 0, .t_end.duration = MHI_T_D-MHI_T_H-(MHI_T_S*3) },
    {.t_l.level = 1, .t_l.duration = MHI_T_L, .t_s.level = 0, .t_s.duration = (MHI_T_S*4)-MHI_T_L, .t_h.level = 1, .t_h.duration = MHI_T_H, .t_end.level = 0, .t_end.duration = MHI_T_D-MHI_T_H-(MHI_T_S*4) },
    {.t_l.level = 1, .t_l.duration = MHI_T_L, .t_s.level = 0, .t_s.duration = (MHI_T_S*5)-MHI_T_L, .t_h.level = 1, .t_h.duration = MHI_T_H, .t_end.level = 0, .t_end.duration = MHI_T_D-MHI_T_H-(MHI_T_S*5) },
    {.t_l.level = 1, .t_l.duration = MHI_T_L, .t_s.level = 0, .t_s.duration = (MHI_T_S*6)-MHI_T_L, .t_h.level = 1, .t_h.duration = MHI_T_H, .t_end.level = 0, .t_end.duration = MHI_T_D-MHI_T_H-(MHI_T_S*6) },
    {.t_l.level = 1, .t_l.duration = MHI_T_L, .t_s.level = 0, .t_s.duration = (MHI_T_S*7)-MHI_T_L, .t_h.level = 1, .t_h.duration = MHI_T_H, .t_end.level = 0, .t_end.duration = MHI_T_D-MHI_T_H-(MHI_T_S*7) },
    {.t_l.level = 1, .t_l.duration = MHI_T_L, .t_s.level = 0, .t_s.duration = (MHI_T_S*8)-MHI_T_L, .t_h.level = 1, .t_h.duration = MHI_T_H, .t_end.level = 0, .t_end.duration = MHI_T_D-MHI_T_H-(MHI_T_S*8) }
};

static void rmt_item_to_mhi_packet_cvt(mhi_packet_t *rx_packet, rmt_item64_t *rx_rmt_items )
{
    int packet_idx = 0;
    uint16_t data = 0;
    for(int i = 0; i < 16; i++ )
    {
        data = 0;
        for(int j = 0;j<3;j++)
        {
            int byte_3_duration = rx_rmt_items[i*3+j].t_l.duration + rx_rmt_items[i*3+j].t_s.duration;
            int byte_all_duration = byte_3_duration + rx_rmt_items[i*3+j].t_h.duration + rx_rmt_items[i*3+j].t_end.duration;

            if((rx_rmt_items[i*3+j].t_l.duration < HBS_GLITCH || rx_rmt_items[i*3+j].t_s.duration < HBS_GLITCH || rx_rmt_items[i*3+j].t_h.duration < HBS_GLITCH || rx_rmt_items[i*3+j].t_end.duration < HBS_GLITCH) && rx_rmt_items[i*3+j].t_end.duration !=0)
            {
                ESP_LOGE(TAG,"error Glitch detected t_l=%d t_s=%d t_h=%d t_end=%d",rx_rmt_items[i*3+j].t_l.duration,rx_rmt_items[i*3+j].t_s.duration,rx_rmt_items[i*3+j].t_h.duration,rx_rmt_items[i*3+j].t_end.duration);
                return;
            }
            if(byte_3_duration > (MHI_T_S*8 + RMT_RX_DELTA) || byte_3_duration < (MHI_T_S-RMT_RX_DELTA) )
            {
                ESP_LOGE(TAG,"Byte_3 duration out of range %d",byte_3_duration);
                return;
            }
            if((byte_all_duration > (MHI_T_D + RMT_RX_DELTA) || byte_all_duration < (MHI_T_S-RMT_RX_DELTA)) && byte_all_duration !=0)
            {
                ESP_LOGE(TAG,"Byte_all duration out of range %d",byte_all_duration);
                return;
            }
//            ESP_LOGI(TAG,"byte_3_duration = %d",byte_3_duration);
            data |= ((((byte_3_duration+RMT_RX_DELTA)/MHI_T_S)-1)&0x7)<<9;
            data >>=3;
        }
        rx_packet->raw_data[packet_idx] = (uint8_t) data & 0xff;
//        ESP_LOGI(TAG,"idx =%d data = %x",packet_idx,data);
        data = 0;
        packet_idx++; 
    }
    return;

}
static rmt_item64_t rx_items[16*4] = {0};
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
    mhi_packet_t packet = {};

    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 3*1000,             // the shortest duration
        .signal_range_max_ns = RMT_RX_IDLE_THRES * 1000 // the longest duration
    };
    while (1)
    {
        ESP_ERROR_CHECK(rmt_receive(rx_chan, rx_items, sizeof(rx_items), &receive_config));
        if (xQueueReceive(receive_queue, &rx_data, portMAX_DELAY) == pdTRUE)
        {
#if 0
            //            printf("cnt items = %d sizeof = %d \n",rx_data.num_symbols, sizeof(rx_items)/sizeof(rmt_item64_t));
            for(int i=0;i<sizeof(rx_items)/sizeof(rmt_item64_t);i++)
            {
                
                printf("cnt = %d lvl_l=%d dur_l=%d, lvl_s=%d dur_s=%d lvl_h=%d dur_h=%d lvl_end=%d dur_end=%d dur_byte=%d, dur_all=%d\n",
                    i,
                    rx_items[i].t_l.level,rx_items[i].t_l.duration,
                    rx_items[i].t_s.level,rx_items[i].t_s.duration,
                    rx_items[i].t_h.level,rx_items[i].t_h.duration,
                    rx_items[i].t_end.level,rx_items[i].t_end.duration,
                    rx_items[i].t_l.duration+rx_items[i].t_s.duration,rx_items[i].t_l.duration+rx_items[i].t_s.duration+rx_items[i].t_h.duration+rx_items[i].t_end.duration 
                );

                }
#endif
        if(rx_data.num_symbols != 96)
            {ESP_LOGE(TAG,"ERROR: rx_data.num_symbols != 96 %d",rx_data.num_symbols);}
        rmt_item_to_mhi_packet_cvt(&packet, rx_items);
        // ESP_LOGI(TAG, "all item converted %d byte ",cnt_byte);
        xQueueSend(mhi_rx_packet_queue, &packet, portMAX_DELAY);
        }
    }
}

static void mhi_item_to_rmt_item_cvt(rmt_item64_t *rmt_data, mhi_packet_t *data)
{
    int i=0;
    uint8_t byte_3;
    for(i=0;i<16;i++)
    {
        byte_3 = data->raw_data[i];
        for(int j=0;j<3;j++)
        {
            rmt_data[i*3+j].val = symbols[(byte_3 & 0x7)].val;
            byte_3 >>= 3;
        }
    }
    rmt_data[16*3].val = 0;
}

static rmt_item64_t tx_items[(16*3)+1]; // 16*4 -> 64 bit ( with 00 end transfer )
static void mhi_tx_packet_task(void *p)
{
    mhi_packet_t packet;
    rmt_transmit_config_t rmt_tx_config = {
        .loop_count = 0,
    };
    while (1)
    {
        xQueueReceive(mhi_tx_packet_queue, &packet, portMAX_DELAY);
        mhi_item_to_rmt_item_cvt(tx_items, &packet);
        ESP_ERROR_CHECK(rmt_transmit(tx_chan_handle, tx_encoder, (void*)tx_items, sizeof(tx_items), &rmt_tx_config));
        rmt_tx_wait_all_done(tx_chan_handle, portMAX_DELAY);
        xEventGroupSetBits(mhi_tx_event_group, MHI_TX_DONE_BIT);
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
    xEventGroupWaitBits(mhi_tx_event_group, MHI_TX_DONE_BIT, pdTRUE, pdFALSE, portMAX_DELAY); // all data transmitted
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