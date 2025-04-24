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

//#define GLITCH_DEBUG 
/*
 * Divider set
 */

#define RMT_RX_DIV (80) // 8
#define RMT_RX_IDLE_THRES (5*1000) // 
#define RMT_RX_CLK_OUT (80 * 1000 * 1000 / RMT_RX_DIV) // 1 mks
#define RMT_RX_DELTA (20)
#define HBS_GLITCH 10


#if CONFIG_IDF_TARGET_ESP32C3
#define RX_BLOCK_SYMBOL (48*2) 
#define TX_BLOCK_SYMBOL (48*2) 
#endif

#if CONFIG_IDF_TARGET_ESP32
#define RX_BLOCK_SYMBOL (64*6)
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


static esp_err_t rmt_item_to_mhi_packet_cvt(mhi_packet_t *rx_packet, rmt_rx_done_event_data_t *rx_edata )
{
    int packet_idx = 0;
    size_t rmt_size = (rx_edata->num_symbols) * 2;
    rmt_item16_t *rmt_data = (rmt_item16_t *)rx_edata->received_symbols;
    uint16_t byte1_durations[4] = {0}; //idx->  0-t_l, 1-t_s, 2-t_h, 3-tend

    for(int rmt_idx = 0; rmt_idx < rmt_size;)
    {
        uint16_t data = 0;
        for(int b3_idx = 0;b3_idx<3;b3_idx++)
        {
            for(int b1_idx=0;b1_idx<4;b1_idx++) // read b1 durations -> t_l t_s t_h t_end
            {
                uint16_t filtered_duration = rmt_data[rmt_idx].duration;
                rmt_idx++;
                if (rmt_data[rmt_idx].duration < HBS_GLITCH ) // next symbol glitch -> attension filter only on glitch in pulse !!
                {
                    filtered_duration = filtered_duration + rmt_data[rmt_idx].duration + rmt_data[rmt_idx+1].duration; // add glitch duration + next duration
                    rmt_idx += 2;
                }
                byte1_durations[b1_idx] = filtered_duration;
            }
            uint16_t byte_3_duration = byte1_durations[0]+byte1_durations[1];
            uint16_t byte_all_duration = byte_3_duration+byte1_durations[2]+byte1_durations[3];

            if(byte_3_duration > (MHI_T_S*8 + RMT_RX_DELTA) || byte_3_duration < (MHI_T_S-RMT_RX_DELTA) )
            {
                ESP_LOGE(TAG,"ERROR: Symbol encoding duration out of range %d",byte_3_duration);
                return ESP_FAIL;
            }
            if((byte_all_duration > (MHI_T_D + RMT_RX_DELTA) || byte_all_duration < (MHI_T_S-RMT_RX_DELTA)) && byte_all_duration !=0)
            {
                ESP_LOGE(TAG,"ERROR: Symbol distance duration out of range %d",byte_all_duration);
                return ESP_FAIL;
            }

            data |= ((((byte_3_duration+RMT_RX_DELTA)/MHI_T_S)-1)&0x7)<<9;
            data >>=3;
        }
        rx_packet->raw_data[packet_idx] = (uint8_t) data & 0xff;
        packet_idx++;
        if(packet_idx > sizeof(mhi_packet_t))
        {
            ESP_LOGE(TAG,"ERROR: Packet overflow");
            return ESP_FAIL;
        }
    }
    return ESP_OK;

}
static rmt_item16_t rx_items[RX_BLOCK_SYMBOL*2] = {0}; // max rx buffer for glith detection
static bool IRAM_ATTR rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
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
    rmt_rx_done_event_data_t rx_edata;
    mhi_packet_t packet = {};

    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 3*1000,             // the shortest duration
        .signal_range_max_ns = RMT_RX_IDLE_THRES * 1000 // the longest duration
    };
    while (1)
    {
        ESP_ERROR_CHECK(rmt_receive(rx_chan, rx_items, sizeof(rx_items), &receive_config));
        if (xQueueReceive(receive_queue, &rx_edata, portMAX_DELAY) == pdTRUE)
        {
        if(rx_edata.num_symbols >= RX_BLOCK_SYMBOL-4)
            {ESP_LOGE(TAG,"ERROR: reseived symbols = %d greater then buff %d skip data!!",rx_edata.num_symbols,RX_BLOCK_SYMBOL-4 ); continue;}
        if(rx_edata.num_symbols != 96)
            {ESP_LOGW(TAG,"Warning. Reseived symbols: expected=%d actual=%d, continue with glitch filter",sizeof(mhi_packet_t)*3*2,rx_edata.num_symbols);}
        if (rmt_item_to_mhi_packet_cvt(&packet, &rx_edata) == ESP_OK)
            {
                if(xQueueSend(mhi_rx_packet_queue, &packet, 200/portTICK_PERIOD_MS) != pdPASS) {
                    ESP_LOGE(TAG,"ERROR: mhi_rx_packet_queue full" );
                }
            }
        }
    }
}

#ifdef GLITCH_DEBUG
static rmt_item64_t tx_items[(sizeof(mhi_packet_t)*4)+1]; // 16*4 -> 64 bit ( with 00 end transfer )
static void mhi_item_to_rmt_item_cvt_glitch(rmt_item64_t *rmt_data, mhi_packet_t *data)
{
    int i=0;
    uint8_t byte_3;
    for(i=0;i<sizeof(mhi_packet_t);i++)
    {
        byte_3 = data->raw_data[i];
        for(int j=0;j<4;j++)
        {
            if(j<3){
            rmt_data[i*4+j].val = symbols[(byte_3 & 0x7)].val;
            byte_3 >>= 3;
            }else{
                rmt_data[i*4+j].t_l.level = 1;
                rmt_data[i*4+j].t_l.duration = 2;
                rmt_data[i*4+j].t_s.level = 1;
                rmt_data[i*4+j].t_s.duration = 2;
                rmt_data[i*4+j].t_h.level = 0;
                rmt_data[i*4+j].t_h.duration = 2;
                rmt_data[i*4+j].t_end.level = 0;
                rmt_data[i*4+j].t_end.duration = 2;
            }
        }
    }
    rmt_data[sizeof(mhi_packet_t)*4].val = 0;
}
#else
static rmt_item64_t tx_items[(sizeof(mhi_packet_t)*3)+1]; // 16*4 -> 64 bit ( with 00 end transfer )
static void mhi_item_to_rmt_item_cvt(rmt_item64_t *rmt_data, mhi_packet_t *data)
{
    int i=0;
    uint8_t byte_3;
    for(i=0;i<sizeof(mhi_packet_t);i++)
    {
        byte_3 = data->raw_data[i];
        for(int j=0;j<3;j++)
        {
            rmt_data[i*3+j].val = symbols[(byte_3 & 0x7)].val;
            byte_3 >>= 3;
        }
    }
    rmt_data[sizeof(mhi_packet_t)*3].val = 0;
}
#endif

static void mhi_tx_packet_task(void *p)
{
    mhi_packet_t packet;
    rmt_transmit_config_t rmt_tx_config = {
        .loop_count = 0,
    };
    while (1)
    {
        xQueueReceive(mhi_tx_packet_queue, &packet, portMAX_DELAY);
#ifdef GLITCH_DEBUG
        mhi_item_to_rmt_item_cvt_glitch(tx_items, &packet);
#else
        mhi_item_to_rmt_item_cvt(tx_items, &packet);
#endif
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