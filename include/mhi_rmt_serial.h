#pragma once
// mhi packet
typedef struct
{
    uint32_t packet_size; // sizeof raw_data 16/29/??? tx -> out, rx->in
    union
    {
        struct
        {
            uint16_t receiver_address;  // receiver_address

            uint8_t on_off :1;          // 0-off, 1-on
            uint8_t mode :6;            // 010101-Cooling only, 010111-Fan Only, 111001-Heating Only, 010001-Auto, 010011-Dehumidify
            uint8_t unknovn :1;         // 1-default, 0-not seen

            uint8_t fan_speed;          // 0x98 - level1, 0x99 - level2, 0x9a - level3, 0xaa - off
            uint8_t target_temperature;  // (input-0x80)/2 
            uint8_t unknovn_temperature; // ??

            uint8_t db7;
            uint8_t db8;
            uint8_t db9;
            uint8_t db10;
            uint8_t db11;
            uint8_t db12;
            uint8_t db13;
            uint8_t db14;
            uint8_t db15;

            uint8_t checksum;
        };
        uint8_t raw_data[31]; // Equivalent unsigned value for the mhi item 3 channels max 32*3*2 = 192
    };
} mhi_packet_t;

#ifdef __cplusplus
extern "C"
{
#endif

esp_err_t mhi_init(gpio_num_t rx_pin, gpio_num_t tx_pin);   // init mhi/rmt task and queue
esp_err_t mhi_deinit(void);                                 // deinit mhi/rmt task and queue
void      mhi_tx_packet(const mhi_packet_t *packet);              // send mhi packet ( return after all data transmitted )
esp_err_t mhi_rx_packet(mhi_packet_t *packet, TickType_t wait_time); // receive mhi packet
void      mhi_clear_rx_queue(void);                         // clear rx buffer

#ifdef __cplusplus
}
#endif