#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "esp_log.h"
#include <optional>
#include <vector>
#include <iostream>
#include "hdlc.h"
#include "pedal.h"
#include "tonex.h"

namespace pedal
{
    static const char *TAG = "TONEX_CONTROLLER_PEDAL";

    void pedal_receiver(void *arg)
    {
        auto tonex = static_cast<Tonex *>(arg);

        while (1)
        {
            tonex->setSlot(Slot::B);
            vTaskDelay(pdMS_TO_TICKS(10*1000));
            tonex->setSlot(Slot::A);
            vTaskDelay(pdMS_TO_TICKS(10*1000));
        }
    }

    void init(Tonex *tonex)
    {
        xTaskCreatePinnedToCore(pedal_receiver, "pedal_receiver", 4096, tonex, 10, NULL, 0);
    }
}