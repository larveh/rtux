#include <stdio.h>
#include <stdlib.h>
#include "driver/spi_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "hal/spi_types.h"
#include "sdkconfig.h"
#include "driver/spi_slave.h"
#include "string.h"

#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

#define SPI_MALLOC_CAP (MALLOC_CAP_DMA | MALLOC_CAP_32BIT)
#define SPI_DEFAULT_MAX_BUFFER_SIZE 128
#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK 14
#define PIN_NUM_CS 15
#define SPI_MODE 0
#define SPI_DMA 0
#define MAX_BUFFER_SIZE 32
#define SPI_QUEUE_SIZE 10 
// TODO Set correct queue and buffer size

static const spi_host_device_t spi_host = SPI2_HOST;
static const size_t max_buffer_size = MAX_BUFFER_SIZE;  // should set to the minimum transaction length
static uint8_t *tx_buffer;
static uint8_t *rx_buffer;
static const char *tag = "rtux";
static int transmitted = 0;
static int transaction_queued = 0;


esp_err_t spi_send_bytes(uint8_t *data, size_t len){
    memcpy(tx_buffer, data, len);
    spi_slave_transaction_t transaction = {
        .length    = len,  //< Total data length, in bits (x8) -- maximum receivable
        .trans_len = len,                     //< Transaction data length, in bits -- actual received
        .tx_buffer = tx_buffer,             //< tx buffer, or NULL for no MOSI phase
        .rx_buffer = NULL,                  //< rx buffer, or NULL for no MISO phase
        .user      = NULL,                  //< User-defined variable. Can be used to store e.g. transaction ID.
    }; 
    return spi_slave_queue_trans(spi_host, &transaction, portMAX_DELAY);
}

void call_matcher_after_queueing(){
    transaction_queued++;
}
void call_matcher_after_transmission(){
    transmitted++;
}
void init_spi_slave(){
    // Initialize the Slave-SPI module:
    ESP_LOGI(tag, "Initializing SPI...");
    tx_buffer = (uint8_t*) heap_caps_malloc(max_buffer_size, SPI_MALLOC_CAP);
    if (tx_buffer == NULL){
        ESP_LOGE(tag, "Failed to allocate tx_buffer!");
    }
    rx_buffer = (uint8_t*) heap_caps_malloc(max_buffer_size, SPI_MALLOC_CAP);
    if (rx_buffer == NULL){
        ESP_LOGE(tag, "Failed to allocate rx_buffer!");
    }
    memset(tx_buffer, 0, max_buffer_size);
    memset(rx_buffer, 0, max_buffer_size);

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI, 
        .miso_io_num = PIN_NUM_MISO, 
        .sclk_io_num = PIN_NUM_CLK,
    };
    gpio_set_pull_mode(PIN_NUM_MOSI,   GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_MISO,   GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CLK,    GPIO_PULLUP_ONLY);

    spi_slave_interface_config_t slvcfg = {
        .spics_io_num = PIN_NUM_CS,              //< CS GPIO pin for this device
        .flags        = 0,               //< Bitwise OR of SPI_SLAVE_* flags
        .queue_size   = SPI_QUEUE_SIZE,  //< Transaction queue size.
                                         //  This sets how many transactions can be 'in the air' (
                                         //    queued using spi_slave_queue_trans -- non-blocking --
                                         //    but not yet finished using spi_slave_get_trans_result -- blocking)
                                         //    at the same time
        .mode          = SPI_MODE,       //< SPI mode (0-3)
        .post_setup_cb = call_matcher_after_queueing,     //< Called after the SPI registers are loaded with new data
        .post_trans_cb = call_matcher_after_transmission  //< Called after a transaction is done
    };

    esp_err_t err = spi_slave_initialize(spi_host, &buscfg, &slvcfg, SPI_DMA);  // Setup the SPI module
    if (err != ESP_OK) {
        ESP_LOGE(tag, "Error %d",err);
    }

}

void app_main(void)
{
    init_spi_slave();
    while (1) {
        ESP_LOGI(tag, "Sending 1...");
        uint8_t num = 1;
        spi_send_bytes(&num, 1);
        ESP_LOGI(tag, "Queued %d, Transmitted %d", transaction_queued, transmitted);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(tag, "Sending 0...");
        num = 0;
        spi_send_bytes(&num, 1);
        ESP_LOGI(tag, "Queued %d, Transmitted %d", transaction_queued, transmitted);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
