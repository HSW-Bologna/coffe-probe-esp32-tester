#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "slave.h"
#include "expander.h"
#include "driver/uart.h"
#include "../components/liblightmodbus-esp/src/esp.config.h"
#include "lightmodbus/base.h"
#include "lightmodbus/lightmodbus.h"
#include "lightmodbus/slave.h"
#include "lightmodbus/slave_func.h"
#include "esp_log.h"

/* enable logs only if debugging
 * since UART port used by the logger is used by modbus */
#define MODBUS_ENABLE_LOGS    0

#define MODBUS_FRAME_SIZE     256
#define MODBUS_TIMEOUT        10

#define MODBUS_REG_CMD_ADDR   0x0000
#define MODBUS_REG_STATE_ADDR 0x0001

typedef enum test_state {
    TEST_INIT,
    TEST_IN_PROGRESS,
    TEST_FAILED,
    TEST_SUCCESS
} test_state_t;

static ModbusError register_callback(
    const ModbusSlave *slave,
    const ModbusRegisterCallbackArgs *args,
    ModbusRegisterCallbackResult *result
);
ModbusError exception_callback(
    const ModbusSlave *slave,
    uint8_t function,
    ModbusExceptionCode code
);
static void test_task(void *args);

/* slave instance */
static ModbusSlave slave;
/* test handles */
static TaskHandle_t test_task_handle;
static SemaphoreHandle_t test_sem;
static test_state_t test_state;


static const char *TAG = "Slave";


void slave_init() {
    ESP_LOGI(TAG, "Initializing modbus slave...");
    ModbusErrorInfo err = modbusSlaveInit(
        &slave,
        register_callback,              // Callback for register operations
        exception_callback,             // Callback for handling slave exceptions (optional)
        modbusDefaultAllocator,         // Memory allocator for allocating responses
        modbusSlaveDefaultFunctions,    // Set of supported functions
        modbusSlaveDefaultFunctionCount // Number of supported functions
    );

    if (!modbusIsOk(err)) {
        ESP_LOGE(TAG, "modbusSlaveInit() failed");
    }

    /* install UART driver */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, MODBUS_FRAME_SIZE * 2, 0, 0, NULL, 0));

    /* init test state */
    test_state = TEST_INIT;

    /* semaphore to synchronize the test task */
    test_sem = xSemaphoreCreateBinary();
    xSemaphoreTake(test_sem, (TickType_t) 0);

    /* test task */
    static StackType_t test_task_stack[4096];
    static StaticTask_t test_stack_tcb;
    test_task_handle = xTaskCreateStatic(
        test_task, "slave-test", 4096, NULL, tskIDLE_PRIORITY, test_task_stack, &test_stack_tcb
    );

#if !MODBUS_ENABLE_LOGS
    /* it does not really disable the logs, but log errors should not occur */
    esp_log_level_set(TAG, ESP_LOG_ERROR);
    uart_flush(UART_NUM_0);
#endif
    /* this does not print if the above line is compiled */
    ESP_LOGI(TAG, "Logs enabled");
}

void slave_manage() {
    static uint8_t frame[MODBUS_FRAME_SIZE];

    size_t frame_len = uart_read_bytes(
        UART_NUM_0, frame, MODBUS_FRAME_SIZE, pdMS_TO_TICKS(MODBUS_TIMEOUT)
    );

    if (frame_len > 0) {
        ModbusErrorInfo err = modbusParseRequestRTU(&slave, 1, frame, frame_len);

        if (modbusIsOk(err)) {
            size_t response_len = modbusSlaveGetResponseLength(&slave);

            if (response_len > 0) {
                uart_write_bytes(UART_NUM_0, modbusSlaveGetResponse(&slave), response_len);
                ESP_LOGI(TAG, "Sent response, length: %zu", response_len);
            } else {
                ESP_LOGI(TAG, "Empty response, nothing to send");
            }
        } else if (err.error != MODBUS_ERROR_ADDRESS && err.error != MODBUS_ERROR_CRC) {
            ESP_LOGW(TAG, "Invalid request with source %i and error %i", err.source, err.error);
            ESP_LOG_BUFFER_HEX(TAG, frame, frame_len);
        }
    }
}

static ModbusError register_callback(
    const ModbusSlave *slave,
    const ModbusRegisterCallbackArgs *args,
    ModbusRegisterCallbackResult *result
) {
    ESP_LOGI(TAG, "%i %i %i %i %i",
             args->query, args->type, args->index, args->value, args->function);

    /* start with EXCEP_NONE, update it if necessary */
    result->exceptionCode = MODBUS_EXCEP_NONE;

    switch (args->query)
    {
        /* read request */
        case MODBUS_REGQ_R_CHECK:
            ESP_LOGI(TAG, "Received read request");

            if (args->type != MODBUS_HOLDING_REGISTER) {
                result->exceptionCode = MODBUS_EXCEP_ILLEGAL_FUNCTION;
                break;
            }

            switch (args->index) {
                case MODBUS_REG_STATE_ADDR:
                    break;
                default:
                    result->exceptionCode = MODBUS_EXCEP_ILLEGAL_ADDRESS;
                    break;
            }
            
            break;

        /* write request */
        case MODBUS_REGQ_W_CHECK:
            ESP_LOGI(TAG, "Received write request");

            if (args->type != MODBUS_COIL) {
                result->exceptionCode = MODBUS_EXCEP_ILLEGAL_FUNCTION;
                break;
            }

            switch (args->index) {
                case MODBUS_REG_CMD_ADDR:
                    if (test_state == TEST_IN_PROGRESS) {
                        result->exceptionCode = MODBUS_EXCEP_SLAVE_FAILURE;
                    }
                    break;
                default:
                    result->exceptionCode = MODBUS_EXCEP_ILLEGAL_ADDRESS;
                    break;
            }

            break;

        /* read request handler */
        case MODBUS_REGQ_R:
            ESP_LOGI(TAG, "Handling read request");

            switch (args->index) {
                case MODBUS_REG_STATE_ADDR:
                    result->value = test_state;
                    break;
                default:
                    break;
            }
            break;

        /* write request handler */
        case MODBUS_REGQ_W:
            ESP_LOGI(TAG, "Handling write request");

            switch (args->index) {
                case MODBUS_REG_CMD_ADDR:
                    test_state = TEST_IN_PROGRESS;
                    xSemaphoreGive(test_sem);
                    break;
                default:
                    break;
            }

            break;

        default:
            break;
    }

    return MODBUS_OK;
}

ModbusError exception_callback(
    const ModbusSlave *slave,
    uint8_t function,
    ModbusExceptionCode code
) {
    ESP_LOGW(TAG, "Slave exception %d (function %hhd)", code, function);
    return MODBUS_OK;
}

void test_task(void *args) {
    /* note: the state of the gpio pin of the output expander is saved to avoid 1 read
     * and this can be done just because expander functions are not used elsewhere */
    _Bool out_pin = 0;

    for (;;) {
        /* handle test */
        if (test_state == TEST_IN_PROGRESS) {
            if (out_pin == 0) {
                if (expander_in_get_gpio_pin(0)) {
                    test_state = TEST_FAILED;
                } else {
                    expander_out_set_gpio_pin(0);
                    out_pin = 1;
                    /* wait 100ms to have a clean read of the input */
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            } else { /* out_pin == 1 */
                if (expander_in_get_gpio_pin(0)) {
                    test_state = TEST_SUCCESS;
                } else {
                    test_state = TEST_FAILED;
                }
            }
        }

        /* init or terminate test */
        if (
            test_state == TEST_INIT ||
            test_state == TEST_SUCCESS ||
            test_state == TEST_FAILED
        ) {
            expander_out_unset_gpio_pin(0);
            out_pin = 0;
            xSemaphoreTake(test_sem, (TickType_t) 0);
        }
    }
}
