#include <stdint.h>
#include "expander.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#define EXPANDER_HOST           SPI2_HOST
#define EXPANDER_PIN_NUM_MISO   13
#define EXPANDER_PIN_NUM_MOSI   18
#define EXPANDER_PIN_NUM_CLK    14
#define EXPANDER_CLK_FREQ       100*1000 // 100kHz
#define IN_EXPANDER_PIN_NUM_CS  16
#define OUT_EXPANDER_PIN_NUM_CS 4

#define EXPANDER_REG_IODIR      0x00
#define EXPANDER_REG_GPIO       0x09

/* In and Out expander instances */
static spi_device_handle_t inexp_handle;
static spi_device_handle_t outexp_handle;

static uint8_t expander_read(spi_device_handle_t handle, uint8_t reg_addr);
static void expander_write(spi_device_handle_t handle, uint8_t reg_addr, uint8_t reg_data);


static const char *TAG = "Expander";


void expander_init() {
    // Initialize the SPI bus
    spi_bus_config_t bus_cfg = {
        .miso_io_num = EXPANDER_PIN_NUM_MISO,
        .mosi_io_num = EXPANDER_PIN_NUM_MOSI,
        .sclk_io_num = EXPANDER_PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8,
    };
    ESP_LOGI(TAG, "Initializing bus SPI%d...", EXPANDER_HOST+1);
    ESP_ERROR_CHECK(spi_bus_initialize(EXPANDER_HOST, &bus_cfg, 0));

    // Initialize In Expander (8-bit expander MCP23S08)
    spi_device_interface_config_t inexp_cfg = {
        .command_bits = 8,
        .address_bits = 8,
        .clock_speed_hz = EXPANDER_CLK_FREQ,
        .mode = 0,
        .spics_io_num = IN_EXPANDER_PIN_NUM_CS,
        .queue_size = 1,
    };
    ESP_LOGI(TAG, "Initializing In Expander...");
    ESP_ERROR_CHECK(spi_bus_add_device(EXPANDER_HOST, &inexp_cfg, &inexp_handle));

    // Initialize Out Expander (8-bit expander MCP23S08)
    spi_device_interface_config_t outexp_cfg = {
        .command_bits = 8,
        .address_bits = 8,
        .clock_speed_hz = EXPANDER_CLK_FREQ,
        .mode = 0,
        .spics_io_num = OUT_EXPANDER_PIN_NUM_CS,
        .queue_size = 1,
    };
    ESP_LOGI(TAG, "Initializing Out Expander...");
    ESP_ERROR_CHECK(spi_bus_add_device(EXPANDER_HOST, &outexp_cfg, &outexp_handle));

    // set I/O direction register of the out expander to
    // configure all pins as output
    expander_write(outexp_handle, EXPANDER_REG_IODIR, 0x00);
}

void expander_out_set_gpio_pin(uint8_t pin) {
    /* pins goes from 0 to 7 */
    if (pin <= 7) {
        /* in the interested context, we can avoid a read (or saving the gpio value) */
        /* since we are interested just at 1 bit */
        // uint8_t gpio_reg_value = expander_read(outexp_handle, EXPANDER_REG_GPIO);
        // expander_write(outexp_handle, EXPANDER_REG_GPIO, gpio_reg_value | (1 << pin));
        expander_write(outexp_handle, EXPANDER_REG_GPIO, 1 << pin);
    }
}

void expander_out_unset_gpio_pin(uint8_t pin) {
    /* pins goes from 0 to 7 */
    if (pin <= 7) {
        /* in the interested context, we can avoid a read (or caching the gpio value) */
        /* since we are interested just at 1 bit */
        // uint8_t gpio_reg_value = expander_read(outexp_handle, EXPANDER_REG_GPIO);
        // expander_write(outexp_handle, EXPANDER_REG_GPIO, gpio_reg_value & ~(1 << pin));
        expander_write(outexp_handle, EXPANDER_REG_GPIO, 0x00);
    }
}

_Bool expander_in_get_gpio_pin(uint8_t pin) {
    /* pins goes from 0 to 7 */
    if (pin <= 7) {
        uint8_t gpio_reg_value = expander_read(inexp_handle, EXPANDER_REG_GPIO);
        /* negating because 0 is set, 1 is not set */
        return !(gpio_reg_value & (1 << pin));
    }

    return 0;
}

void expander_test() {
    ESP_LOGI(TAG, "Test - write Out Exp GPIO");
    expander_write(outexp_handle, EXPANDER_REG_GPIO, 0x01);

    ESP_LOGI(TAG, "Test - read In Exp GPIO (return: %hhx)",
             expander_read(inexp_handle, EXPANDER_REG_GPIO));
}

static uint8_t expander_read(spi_device_handle_t handle, uint8_t reg_addr) {
    spi_transaction_t t = {0};
    t.cmd = 0x41;
    t.addr = reg_addr;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.length = 8;

    ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &t));
    return t.rx_data[0];
}

static void expander_write(spi_device_handle_t handle, uint8_t reg_addr, uint8_t reg_data) {
    spi_transaction_t t = {0};
    t.cmd = 0x40;
    t.addr = reg_addr;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = reg_data;
    t.length = 8;

    ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &t));
}
