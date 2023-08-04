#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"

#include "model/model.h"
#include "controller/controller.h"


static const char *TAG = "Main";


void app_main(void) {
    model_t model;

    model_init(&model);
    // view_init(&model);
    controller_init(&model);

    ESP_LOGI(TAG, "Begin main loop");
    for (;;) {
        controller_manage();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
