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
        APP_SLOT = 0,
        APP_PRESET,
        APP_PRESET_INCR
    } app_event_group_t;

    typedef struct
    {
        app_event_group_t event_group;
        union
        {
            Slot slot;
            uint8_t preset;
            int8_t incr;
        };
    } app_event_queue_t;

    static void gpio_cb(void *arg)
    {
        const app_event_queue_t evt_queue = {
            .event_group = APP_PRESET_INCR,
            .incr = 1};

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
                const Slot currentSlot = tonex->getCurrentSlot();
                const uint8_t preset = tonex->getPreset(currentSlot);

                switch (evt_queue.event_group)
                {
                case APP_SLOT:
                    tonex->setSlot(evt_queue.slot);
                    break;
                case APP_PRESET:
                    tonex->changePreset(currentSlot, evt_queue.preset);
                    break;
                case APP_PRESET_INCR:
                {
                    int8_t newPreset = preset + evt_queue.incr;

                    if (newPreset < 0) newPreset = 19;
                    if (newPreset > 19) newPreset = 0;
                    tonex->changePreset(currentSlot, (uint8_t) newPreset);
                }
                break;
                default:
                    break;
                }
            }
        }
    }

    void init(Tonex *tonex)
    {
        xTaskCreatePinnedToCore(pedal_receiver, "pedal_receiver", 4096, tonex, 10, NULL, 0);
    }
}