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

#define APP_PIN GPIO_NUM_0

namespace pedal
{
    static const char *TAG = "TONEX_CONTROLLER_PEDAL";

    static QueueHandle_t app_event_queue;

    typedef enum
    {
        APP_SLOT = 0
    } app_event_group_t;

    typedef struct
    {
        app_event_group_t event_group;
        union
        {
            Slot slot;
        };
    } app_event_queue_t;

    static void gpio_cb(void *arg)
    {
        const app_event_queue_t evt_queue = {
            .event_group = APP_SLOT,
            .slot = Slot::A};

        BaseType_t xTaskWoken = pdFALSE;

        if (app_event_queue)
        {
            xQueueSendFromISR(app_event_queue, &evt_queue, &xTaskWoken);
        }

        if (xTaskWoken == pdTRUE)
        {
            portYIELD_FROM_ISR();
        }
    }

    void pedal_receiver(void *arg)
    {
        auto tonex = static_cast<Tonex *>(arg);

        const gpio_config_t input_pin = {
            .pin_bit_mask = BIT64(APP_PIN),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .intr_type = GPIO_INTR_NEGEDGE,
        };
        ESP_ERROR_CHECK(gpio_config(&input_pin));
        ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
        ESP_ERROR_CHECK(gpio_isr_handler_add(APP_PIN, gpio_cb, NULL));

        app_event_queue = xQueueCreate(10, sizeof(app_event_queue_t));
        app_event_queue_t evt_queue;

        while (1)
        {
            if (xQueueReceive(app_event_queue, &evt_queue, portMAX_DELAY))
            {
                if (APP_SLOT == evt_queue.event_group)
                {
                    tonex->setSlot(evt_queue.slot);
                }
            }
        }
    }

    void init(Tonex *tonex)
    {
        xTaskCreatePinnedToCore(pedal_receiver, "pedal_receiver", 4096, tonex, 10, NULL, 0);
    }
}