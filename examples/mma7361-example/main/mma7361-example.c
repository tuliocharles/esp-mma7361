#include <stdio.h>

#include "freertos/FreeRTOS.h"

//#include "../../../include/esp-mma7361.h"
#include "esp-mma7361.h"

void app_main(void)
{
    mma7361_handle_t mma7361_handle;
    mma7361_config_t mma7361_config = {
        .x_channel = ADC_CHANNEL_0,
        .y_channel = ADC_CHANNEL_3,
        .z_channel = ADC_CHANNEL_6,
        .g_select = GPIO_NUM_4,
        .self_test = GPIO_NUM_16,
        .zero_g = GPIO_NUM_17,
        .sleep = GPIO_NUM_15,
        .x_offset = 1650,
        .y_offset = 1850,
        .z_offset = 1350,
    };

    mma7361_new(&mma7361_config, &mma7361_handle);

    int axis_x, axis_y, axis_z;

    while (1)
    {
        mma7361_read_3axes(&axis_x, &axis_y, &axis_z, mma7361_handle);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}