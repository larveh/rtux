#include <stdio.h>
#include <stdlib.h>
#include "driver/spi_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "hal/spi_types.h"
#include "sdkconfig.h"
#include "driver/spi_slave.h"
#include "driver/spi_master.h"
#include "string.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "cmd_system.h"
#include "ssd1306.h"
#include "iot_button.h"
#include "math.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "esp_log.h"

#define SPI_CLOCK 1000000
#define SPI_CHANNEL SPI2_HOST
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

#define I2C_MASTER_SCL_IO 4        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 0        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1    /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */

#define PIN_BUT_1 5
#define PIN_BUT_2 6
#define PIN_BUT_3 7
#define PIN_BUT_4 8
#define PIN_BUT_5 9

static const size_t max_buffer_size = MAX_BUFFER_SIZE;  // should set to the minimum transaction length
static uint16_t *tx_buffer;
static uint16_t *rx_buffer;
static const char *tag = "rtux";
static int transmitted = 0;
static int transaction_queued = 0;
static spi_device_handle_t spi_handle = {0};
static ssd1306_handle_t ssd1306_dev = NULL;

void draw_screen1(void);

void call_matcher_after_queueing(){
    transaction_queued++;
}
void call_matcher_after_transmission(){
    transmitted++;
}

// Initialize the SPI2 device in master mode
void init_spi() {
	// Configuration for the SPI bus
        ESP_LOGI(tag, "Initializing SPI...");
        tx_buffer = (uint16_t*) heap_caps_malloc(max_buffer_size, SPI_MALLOC_CAP);
        if (tx_buffer == NULL){
            ESP_LOGE(tag, "Failed to allocate tx_buffer!");
        }
        rx_buffer = (uint16_t*) heap_caps_malloc(max_buffer_size, SPI_MALLOC_CAP);
        if (rx_buffer == NULL){
            ESP_LOGE(tag, "Failed to allocate rx_buffer!");
        }
        memset(tx_buffer, 0, max_buffer_size);
        memset(rx_buffer, 0, max_buffer_size);

	spi_bus_config_t buscfg = {
		.mosi_io_num = PIN_NUM_MOSI,
		.miso_io_num = PIN_NUM_MISO,
		.sclk_io_num = PIN_NUM_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = MAX_BUFFER_SIZE,
	};

	// Configuration for the SPI master interface
	spi_device_interface_config_t devcfg = {
		.command_bits = 0,
		.address_bits = 0,
		.dummy_bits = 0,
		.duty_cycle_pos = 128,
		.cs_ena_pretrans = 0,
		.cs_ena_posttrans = 0, // Keep the CS low 3 cycles after transaction, to stop the master from missing the last bit when CS has less propagation delay than CLK
		.clock_speed_hz = SPI_CLOCK,
		.mode = SPI_MODE,
		.spics_io_num = PIN_NUM_CS,
		.queue_size = SPI_QUEUE_SIZE,
		.flags = 0,
		.pre_cb = call_matcher_after_queueing,
		.post_cb = call_matcher_after_transmission,
	};
	
    	ESP_LOGI(tag, "Setting SPI CLOCK : %d Hz", SPI_CLOCK);
	// Initialize and enable SPI
	esp_err_t err = spi_bus_initialize(SPI_CHANNEL, &buscfg, 1);
    	if (err != ESP_OK) {
        	ESP_LOGE(tag, "Error %d",err);
    	}
	err = spi_bus_add_device(SPI_CHANNEL, &devcfg, &spi_handle);
    	if (err != ESP_OK) {
        	ESP_LOGE(tag, "Error %d",err);
    	}
	int freq_khz;
	spi_device_get_actual_freq(spi_handle, &freq_khz);
    	ESP_LOGI(tag, "Actual SPI CLOCK : %d kHz", freq_khz);

}

// Full buffer DMA transfer
int32_t spi_trans(uint16_t *tx_data, uint16_t *rx_data, uint16_t tx_len, uint16_t rx_len) {
	// Prepare transaction parameters
	memcpy(tx_buffer, tx_data, tx_len * sizeof(uint16_t));	
	spi_transaction_t trans = {
		.rx_buffer = (uint8_t*)rx_buffer,
		.tx_buffer = (uint8_t*)tx_buffer,
		.rxlength = 8 * sizeof(uint16_t) * rx_len,
		.length = 8 * sizeof(uint16_t) * tx_len,
		.flags = 0,
		.cmd = 0,
		.addr = 0,
		.user = NULL,
	};

	// Perform transaction
	esp_err_t res = spi_device_transmit(spi_handle, &trans);
	memcpy(rx_data, rx_buffer, rx_len * sizeof(uint16_t));
	return res;
}

void init_console(){
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = ">";

    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    /* Register commands */
    esp_console_register_help_command();
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

void init_lcd(){
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    ssd1306_dev = ssd1306_create(I2C_MASTER_NUM, SSD1306_I2C_ADDRESS);
    ssd1306_refresh_gram(ssd1306_dev);
    ssd1306_clear_screen(ssd1306_dev, 0x00);

    // This section will go in task
}


#define NUM_OF_BUTTONS 5
button_handle_t gpio_btn[NUM_OF_BUTTONS];
int32_t gpio_pin_nums [NUM_OF_BUTTONS] = {
	PIN_BUT_1,
	PIN_BUT_2,
	PIN_BUT_3,
	PIN_BUT_4,
	PIN_BUT_5
};

void init_buttons(void){
	for(int i = 0; i < NUM_OF_BUTTONS; i++){
		button_config_t gpio_btn_cfg = {
		    .type = BUTTON_TYPE_GPIO,
		    .gpio_button_config = {
			.gpio_num = gpio_pin_nums[i],
			.active_level = 0,
		    },
		};

		gpio_btn[i] = iot_button_create(&gpio_btn_cfg);
		
		if(NULL == gpio_btn[i]) {
		    ESP_LOGE(tag, "Button %d create failed", i);
		}
	}
}

typedef enum button_numbers {
	BTN_UP,
	BTN_DOWN,
	BTN_LEFT,
	BTN_RIGHT,
	BTN_MIDDLE,
} button_number_t;

typedef struct {
	button_event_t btn_evt;
	button_number_t button_number;
	bool is_valid;
} button_m_event_t;

button_m_event_t gather_button_events(void){
	button_m_event_t ret = {0};
	int number_of_events = 0;
	for(int i = 0; i < NUM_OF_BUTTONS; i++){
		button_event_t temp_button_evt = iot_button_get_event(gpio_btn[i]);
		//ESP_LOGI(tag, "Button %d with evt type %d", ret.button_number, ret.btn_evt);
		if(temp_button_evt < BUTTON_EVENT_MAX){
			ret.btn_evt = temp_button_evt;
			ret.button_number = i;
			number_of_events++;
		}
	}
	if(number_of_events == 1){
		// ESP_LOGI(tag, "Got valid button event %d on btn %d!", ret.btn_evt, ret.button_number);
		ret.is_valid = true;
		return ret; 
	}
	// ESP_LOGI(tag, "Got %d button events", number_of_events);
	ret.btn_evt = false;
	return ret;
}

typedef enum screens {
	SEND_INT,
} screens_t;

typedef struct _screen1 {
	uint16_t int_val;
	int8_t current_digit;
	uint16_t recieved_num;
	bool send_number;
} screen1_t;

typedef struct _program_state {
	screen1_t* scrn1;
	uint16_t message_len;
	screens_t current_screen;
} program_state_t;

screen1_t scrn1 = {0};
program_state_t state = {0};

void init_program_state(void){
	state.scrn1 = &scrn1;
	state.current_screen = SEND_INT;
}

bool is_done = false;
void handle_button_press(button_m_event_t evt){
	//ESP_LOGI(tag, "button event: %d button number: %d is valid %d", evt.btn_evt, evt.button_number, evt.is_valid);
	if(evt.is_valid == false){
		//ESP_LOGI(tag, "Button press not valid..");
		is_done = false;
		return;
	}
	// Onl accept single click events
	if(evt.btn_evt != BUTTON_SINGLE_CLICK){
		is_done = false;
		return;
	}
	// Ignore multiple key press events that happen during one actual key press
	if(is_done){
		return;	
	}
	is_done = true;

	if(state.current_screen != SEND_INT){
		return;
	}

	switch(evt.button_number){
		case BTN_UP: {
			// TODO: Prevent overflow
			ESP_LOGI(tag, "Up button pressed!");
			state.scrn1->int_val+= pow(10, state.scrn1->current_digit);
			break;
		}
		case BTN_DOWN: {
			ESP_LOGI(tag, "Down button pressed!");
			uint16_t to_pow = pow(10, state.scrn1->current_digit);
			if(to_pow < state.scrn1->int_val){
				state.scrn1->int_val-= to_pow;
			}
			break;
		}
		case BTN_LEFT: {
			// TODO: Set limits
			ESP_LOGI(tag, "Left button pressed!");
			state.scrn1->current_digit++;
			break;
		}
		case BTN_RIGHT: {
			ESP_LOGI(tag, "Right button pressed!");
			state.scrn1->current_digit--;
			break;
		}
		case BTN_MIDDLE: {		 
			ESP_LOGI(tag, "Middle button pressed!");
			// TODO: add state to enable sending
			break;
		}
		default: 
		{	
			ESP_LOGE(tag, "Invalid Button number %d!", evt.button_number);
			break;
		}
	}
	
	draw_screen1();

}

void draw_screen1(void){
    char data_str[4][17] = {0};
    sprintf(&data_str[0][0], "Send integer[ ]");
    sprintf(&data_str[1][0], "%05d  %d", state.scrn1->int_val, state.scrn1->current_digit);
    sprintf(&data_str[2][0], "Recieved:");
    sprintf(&data_str[3][0], "%d ml:%d", state.scrn1->recieved_num, state.message_len);
    ssd1306_draw_string(ssd1306_dev, 0, 0, (const uint8_t *)&data_str[0][0], 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)&data_str[1][0], 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 32, (const uint8_t *)&data_str[2][0], 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 48, (const uint8_t *)&data_str[3][0], 16, 1);
    
    // Draw cursor
    uint8_t xpos1 = 32 - (8 * state.scrn1->current_digit);
    uint8_t ypos1 = 33;
    uint8_t xpos2 = xpos1 + 8;
    uint8_t ypos2 = ypos1;
    uint8_t dot = 1;
    ssd1306_fill_rectangle(ssd1306_dev, xpos1, ypos1, xpos2, ypos2, dot);
    
    ssd1306_refresh_gram(ssd1306_dev);
}

void app_main(void)
{
    init_program_state();
    init_console();
    init_spi();
    init_lcd();
    init_buttons();

    uint32_t counter = 0;
    uint16_t broken_message_len;
    uint16_t dummy_rx;
    spi_trans(&dummy_rx, &broken_message_len, 1, 1);
    ESP_LOGI(tag, "Broken message len 0x%02X", broken_message_len);
    state.message_len = broken_message_len >> 8;
    ESP_LOGI(tag, "Recieved message length: %d 0x%02X", state.message_len, state.message_len);

    uint16_t rx_num[3] = {0};
    uint16_t tx_num[3] = {0}; 
    
    while (1) {
	counter++;
	button_m_event_t evt = gather_button_events();
	handle_button_press(evt);
	

        tx_num[2] = state.scrn1->int_val; 
	//ESP_LOGI(tag, "Sending: 0x%02X 0x%02X 0x%02X", 
	//		tx_num[0], tx_num[1], tx_num[2]);
        spi_trans(tx_num, rx_num, state.message_len, state.message_len);
	//ESP_LOGI(tag, "Recieved: 0x%02X 0x%02X 0x%02X",
	//		rx_num[0], rx_num[1], rx_num[2]);
	//ESP_LOGI(tag, "Queued successfully %d, Transmitted successfully %d", transaction_queued, transmitted);
	// TODO: Fix extra byte in recieved numbers	
	state.scrn1->recieved_num = rx_num[2];

	if((counter % 3000000) == 0){
		// Yield for FREERTOS housekeeping
		ESP_LOGI(tag, "Sleeping..");
		ESP_LOGI(tag, "Int value: %d", state.scrn1->int_val);
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
    }
}
