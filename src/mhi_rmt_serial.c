/* MHI serial excange driver

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
 * all divider set to 1 mHz (1 mks)
 */

#define RMT_RX_DIV (80) 
#define RMT_RX_IDLE_THRES (5*1000) // 5 millisek  time between two frame
#define RMT_RX_CLK_OUT (80 * 1000 * 1000 / RMT_RX_DIV) // 1 mks
#define RMT_RX_DELTA (20)   // mhi time fluctuation
#define HBS_GLITCH 10       // software glith remove time ( 10 mks ) for bad line
#define HBS_RMT_GLITCH  (3*1000)    // rmt glitch revove time ( 3 mks max )

#ifdef CONFIG_IDF_TARGET_ESP32
#define RX_BLOCK_SYMBOL (SOC_RMT_MEM_WORDS_PER_CHANNEL*7)      // 5 rmt channel memory
#define TX_BLOCK_SYMBOL (SOC_RMT_MEM_WORDS_PER_CHANNEL*1)      // 1 rmt channel memory always ping/pong
#define RMT_RX_BUFF_SIZE (RX_BLOCK_SYMBOL)                     //rmt_item32_t rx buff size
#else // ESP32-S3 ESP32-C3 -> PINGPONG RX
#define RX_BLOCK_SYMBOL (SOC_RMT_MEM_WORDS_PER_CHANNEL*1)      //  rmt channel memory
#define TX_BLOCK_SYMBOL (SOC_RMT_MEM_WORDS_PER_CHANNEL*1)      // 1 rmt channel memory always ping/pong
#define RMT_RX_BUFF_SIZE (sizeof(((mhi_packet_t*)0)->raw_data)*3*2*2) //rmt_item32_t rx buff size *2 with glitch detect
#endif

#define RMT_TX_DIV (80) // 8 // 1 mks
#define RMT_TX_CLK_OUT (80 * 1000 * 1000 / RMT_TX_DIV) // 1 mks

#define MHI_T_L (52)    // low pulse duration
#define MHI_T_H (52)    // hight pulse duration
#define MHI_T_S (104)   // symbol encoding duration tick
#define MHI_T_D (1248)  // symbol distanse duration

static QueueHandle_t mhi_rx_packet_queue;
static QueueHandle_t mhi_tx_packet_queue;
static TaskHandle_t mhi_rx_packet_task_handle;
static TaskHandle_t mhi_tx_packet_task_handle;

static rmt_channel_handle_t tx_chan_handle = NULL;
static rmt_encoder_handle_t tx_encoder_handle = NULL;
static rmt_channel_handle_t rx_chan_handle = NULL;
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

// const rmt definition for 0-7 mhi item - for tx encode
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

// encode rmt buffer to mhi packet with glitch remover
// attension, remove only one glitch in pulse 
static esp_err_t rmt_item_to_mhi_packet_cvt(mhi_packet_t *rx_packet, const rmt_rx_done_event_data_t *rx_edata )
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
                if ((rmt_data[rmt_idx].duration < HBS_GLITCH) && (rmt_data[rmt_idx].duration != 0) ) // next symbol glitch or eof -> attension filter only one glitch in pulse !!
                {
                    //ESP_LOGI(TAG,"Glitch found in rx data %d %d",rmt_idx,rmt_data[rmt_idx].duration); //dbg only
                    filtered_duration = filtered_duration + rmt_data[rmt_idx].duration + rmt_data[rmt_idx+1].duration; // add glitch duration + next duration
                    rmt_idx += 2;
                }
                byte1_durations[b1_idx] = filtered_duration;
            }
            uint16_t byte_3_duration = byte1_durations[0]+byte1_durations[1]; //symbol duration 0 = 104, 7 = 104*8
//            uint16_t byte_all_duration = byte_3_duration+byte1_durations[2]+byte1_durations[3]; // symbol distanse duration, default 1248 mks

            if(byte_3_duration > (MHI_T_S*8 + RMT_RX_DELTA) || byte_3_duration < (MHI_T_S-RMT_RX_DELTA) )
            {
                ESP_LOGE(TAG,"ERROR: Symbol encoding duration out of range %d idx %d",byte_3_duration,rmt_idx);
                return ESP_FAIL;
            }
/*            
            if((byte_all_duration > (MHI_T_D + RMT_RX_DELTA) || byte_all_duration < (MHI_T_D-RMT_RX_DELTA)) && byte_all_duration !=0)
            {
                ESP_LOGE(TAG,"ERROR: Symbol distance duration out of range %d",byte_all_duration);
                return ESP_FAIL;
            }
*/
            data |= ((((byte_3_duration+RMT_RX_DELTA)/MHI_T_S)-1)&0x7)<<9;
            data >>=3;
        }
        rx_packet->raw_data[packet_idx] = (uint8_t) data & 0xff; // mhi_packet data 
        packet_idx++;
        if(packet_idx > sizeof(rx_packet->raw_data))
        {
            ESP_LOGE(TAG,"ERROR: Packet overflow");
            return ESP_FAIL;
        }
        //ESP_LOGI(TAG,"rmtidx = %d packetidx = %d byte1_durations = %d %d %d %d",rmt_idx,packet_idx,byte1_durations[0],byte1_durations[1],byte1_durations[2],byte1_durations[3]); //dbg only
    }
    rx_packet->packet_size = packet_idx;
    return ESP_OK;

}
static rmt_item16_t rx_items[RMT_RX_BUFF_SIZE*2] = {0}; // max rx buffer for glith detection
// rmt callback
static bool IRAM_ATTR rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    // return whether any task is woken up
    return high_task_wakeup == pdTRUE;
}
// receiver task
static void mhi_rx_packet_task(void *p)
{
    rmt_rx_done_event_data_t rx_edata;
    mhi_packet_t packet = {};

    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = HBS_RMT_GLITCH,             // the shortest duration
        .signal_range_max_ns = RMT_RX_IDLE_THRES * 1000, // the longest duration
#if SOC_RMT_SUPPORT_RX_PINGPONG        
        .flags.en_partial_rx = true,
#endif
    };
    while (1)
    {
        memset(rx_items,0,sizeof(rx_items)); // clear rmt rx buffer
        memset(&packet,0,sizeof(mhi_packet_t)); // clear packet rx buffer
        ESP_ERROR_CHECK(rmt_receive(rx_chan_handle, rx_items, sizeof(rx_items), &receive_config));
        if (xQueueReceive(receive_queue, &rx_edata, portMAX_DELAY) == pdTRUE)
        {
        if(rx_edata.num_symbols >= RMT_RX_BUFF_SIZE-1)
            {ESP_LOGE(TAG,"ERROR: reseived symbols = %d greater then buff %d skip data!!",rx_edata.num_symbols,RX_BLOCK_SYMBOL-4 ); continue;}

            // This message is not relevant when the packet size is variable.
/*
            if(rx_edata.num_symbols != sizeof(packet.packet_size)*3*2)
                {ESP_LOGW(TAG,"Warning. Reseived symbols: expected=%d actual=%d, continue with glitch filter",sizeof(packet.packet_size)*3*2,rx_edata.num_symbols);}
*/
            if (rmt_item_to_mhi_packet_cvt(&packet, &rx_edata) == ESP_OK)
            {
                if(xQueueSend(mhi_rx_packet_queue, &packet,0 /*5000/portTICK_PERIOD_MS*/) != pdPASS) { // wait time = 0 for debug
                    ESP_LOGE(TAG,"ERROR: mhi_rx_packet_queue full" );
                }
            }
        }
    }
}

static rmt_item64_t tx_items[(sizeof(((mhi_packet_t*)0)->raw_data)*3)+1]; // rmt transmit buffer
// encode mhi items to rmt items
static void mhi_item_to_rmt_item_cvt(rmt_item64_t *rmt_data, const mhi_packet_t *data)
{
    int i=0;
    uint8_t byte_3;
    for(i=0;i< data->packet_size ;i++)
    {
        byte_3 = data->raw_data[i];
        for(int j=0;j<3;j++)
        {
            rmt_data[i*3+j].val = symbols[(byte_3 & 0x7)].val;
            byte_3 >>= 3;
        }
    }
    rmt_data[data->packet_size*3].val = 0;
}
// transmitter task
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
        ESP_ERROR_CHECK(rmt_transmit(tx_chan_handle, tx_encoder_handle, (void*)tx_items, sizeof(tx_items), &rmt_tx_config)); // change to packet_size ??
        rmt_tx_wait_all_done(tx_chan_handle, portMAX_DELAY);
        xEventGroupSetBits(mhi_tx_event_group, MHI_TX_DONE_BIT);
    }
}
// init rmt channels
esp_err_t mhi_init(gpio_num_t rx_pin, gpio_num_t tx_pin)
{

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = tx_pin,
        .mem_block_symbols = TX_BLOCK_SYMBOL, 
        .flags.invert_out = true,
        .resolution_hz = RMT_TX_CLK_OUT,
        .trans_queue_depth = 5, // set the maximum number of transactions that can pend in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_chan_handle));

    rmt_copy_encoder_config_t tx_encoder_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&tx_encoder_config, &tx_encoder_handle));
    ESP_ERROR_CHECK(rmt_enable(tx_chan_handle));
    //
    mhi_tx_event_group = xEventGroupCreate();
    mhi_tx_packet_queue = xQueueCreate(1, sizeof(mhi_packet_t));
    xTaskCreate(mhi_tx_packet_task, "rmt tx", 4096, NULL, 5, &mhi_tx_packet_task_handle);

    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,  // select source clock
        .resolution_hz = RMT_RX_CLK_OUT, // tick resolution,
        .mem_block_symbols = RX_BLOCK_SYMBOL,
        .gpio_num = rx_pin,              // GPIO number
        .flags.invert_in = false,        // do not invert input signal
        .flags.with_dma = false,         // do not need DMA backend
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_chan_handle));
    receive_queue = xQueueCreate(10, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan_handle, &cbs, receive_queue));
    ESP_ERROR_CHECK(rmt_enable(rx_chan_handle));
    
    mhi_rx_packet_queue = xQueueCreate(4, sizeof(mhi_packet_t));
    xTaskCreate(mhi_rx_packet_task, "rmt rx", 4096, NULL, 5, &mhi_rx_packet_task_handle);

    return ESP_OK;
}
// free rmt channels
esp_err_t mhi_deinit(void)
{
    // deinit RX
    vTaskDelete(mhi_rx_packet_task_handle);

    rmt_disable(rx_chan_handle);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = NULL,
    };
    rmt_rx_register_event_callbacks(rx_chan_handle, &cbs, receive_queue);
    rmt_del_channel(rx_chan_handle);
    vQueueDelete(receive_queue);
    vQueueDelete(mhi_rx_packet_queue);

    // deinit TX
    vTaskDelete(mhi_tx_packet_task_handle);

    rmt_disable(tx_chan_handle);
    rmt_del_encoder(tx_encoder_handle);
    rmt_del_channel(tx_chan_handle);

    vQueueDelete(mhi_tx_packet_queue);
    vEventGroupDelete(mhi_tx_event_group);

    return ESP_OK;
}
// transmit mhi packet
void mhi_tx_packet(const mhi_packet_t *packet)
{
    xQueueSend(mhi_tx_packet_queue, packet, portMAX_DELAY);                                   // data send to tx queue, start transmit
    xEventGroupWaitBits(mhi_tx_event_group, MHI_TX_DONE_BIT, pdTRUE, pdFALSE, portMAX_DELAY); // all data transmitted
}
// receive mhi packet
esp_err_t mhi_rx_packet(mhi_packet_t *packet, TickType_t wait_time)
{
    int ret = xQueueReceive(mhi_rx_packet_queue, packet, wait_time);
    if (ret != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}
// flush rx buffer
void mhi_clear_rx_queue(void)
{
    xQueueReset(mhi_rx_packet_queue);
}