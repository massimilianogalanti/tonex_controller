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
#include "led_strip.h"
#include "button.h"

#define STOMP_L_PIN 4
#define STOMP_C_PIN 5
#define STOMP_R_PIN 6

#define BLINK_GPIO 48

/// LED strip common configuration
led_strip_config_t strip_config = {
    .strip_gpio_num = BLINK_GPIO,                                // The GPIO that connected to the LED strip's data line
    .max_leds = 1,                                               // The number of LEDs in the strip,
    .led_model = LED_MODEL_WS2812,                               // LED strip model, it determines the bit timing
    .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color component format is G-R-B
    .flags = {
        .invert_out = false, // don't invert the output signal
    }};

/// RMT backend specific configuration
led_strip_rmt_config_t rmt_config = {
    .clk_src = RMT_CLK_SRC_DEFAULT,    // different clock source can lead to different power consumption
    .resolution_hz = 10 * 1000 * 1000, // RMT counter clock frequency: 10MHz
    .mem_block_symbols = 64,           // the memory size of each RMT channel, in words (4 bytes)
    .flags = {
        .with_dma = false, // DMA feature is available on chips like ESP32-S3/P4
    }};

/// Create the LED strip object
led_strip_handle_t led_strip;

namespace pedal
{
    static const char *TAG = "TONEX_CONTROLLER_PEDAL";

    void pedal_receiver(void *arg)
    {
        auto tonex = static_cast<Tonex *>(arg);
        QueueHandle_t button_events = button_init(PIN_BIT(STOMP_L_PIN) | PIN_BIT(STOMP_C_PIN) | PIN_BIT(STOMP_R_PIN));

        button_event_t ev;

        while (1)
        {
            if (xQueueReceive(button_events, &ev, portMAX_DELAY) && tonex->getConnectionState() == StateInitialized)
            {
                const Slot currentSlot = tonex->getCurrentSlot();
                const uint8_t preset = tonex->getPreset(currentSlot);

                switch (ev.pin)
                {
                // case APP_SLOT:
                //     tonex->setSlot(evt_queue.slot);
                //     break;
                // case APP_PRESET:
                //     tonex->changePreset(currentSlot, evt_queue.preset);
                //     break;
                case STOMP_C_PIN:
                    if (ev.event == BUTTON_DOWN)
                    {
                        tonex->setSlot(currentSlot == Slot::A ? Slot::B : Slot::A);
                    }
                    break;
                case STOMP_L_PIN:
                case STOMP_R_PIN:
                {
                    if (ev.event == BUTTON_DOWN)
                    {
                        int8_t newPreset = preset + (ev.pin == STOMP_L_PIN ? -1 : 1);

                        if (newPreset < 0)
                            newPreset = 19;
                        if (newPreset > 19)
                            newPreset = 0;
                        tonex->changePreset(currentSlot, (uint8_t)newPreset);
                    }
                    else if (ev.event == BUTTON_HELD)
                    {
                    }
                }
                break;
                default:
                    break;
                }
            }
        }
    }

    void ux_receiver(void *arg)
    {
        auto tonex = static_cast<Tonex *>(arg);

        ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
        ESP_ERROR_CHECK(led_strip_clear(led_strip));
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 4, 0));
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));

        while (1)
        {
            switch (tonex->getConnectionState())
            {
            case StateInitialized:
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 4, 0));
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                break;
            default:
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 4, 0, 0));
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));

                vTaskDelay(500 / portTICK_PERIOD_MS);
                ESP_ERROR_CHECK(led_strip_clear(led_strip));
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);
            // if (xQueueReceive(button_events, &ev, portMAX_DELAY))
            // }
        }
    }

    void init(Tonex *tonex)
    {
        xTaskCreatePinnedToCore(pedal_receiver, "pedal_receiver", 4096, tonex, 10, NULL, 0);
        xTaskCreatePinnedToCore(ux_receiver, "ux_receiver", 4096, tonex, 10, NULL, 0);
    }
}