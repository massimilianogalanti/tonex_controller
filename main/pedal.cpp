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
#include "max7219.h"

#define CASCADE_SIZE 2
#define MOSI_PIN 11
#define CS_PIN 10
#define CLK_PIN 12
#define HOST SPI3_HOST

static const uint64_t symbols[] = {
    0x00003c666e76663c,
    0x00003c18181c1818,
    0x00007e0c3060663c,
    0x00003c627860663c,
    0x0000307e32343830,
    0x00003c66603e067e,
    0x00003c663e06663c,
    0x000018183030667e,
    0x00003c663c66663c,
    0x00003c607c66663c,
    0xff003c666e76663c,
    0xff003c18181c1818,
    0xff007e0c3060663c,
    0xff003c627860663c,
    0xff00307e32343830,
    0xff003c66603e067e,
    0xff003c663e06663c,
    0xff0018183030667e,
    0xff003c663c66663c,
    0xff003c607c66663c,
    0xffffffffffffffff,
    0xffffc3c3c3c3ffff,
    0x187e66c3c3667e18,
    0xff7e7e3c3c3c1818,
    0xc3e77e3c3c7ee7c3,
    0x3c1818181818183c,
    0x1e3666666666361e,
    0x3e7e607e7e067e7c,
    0x3e66667e1e36361e,
    0x0406ff2664ff6020,
    0x1818181818181818,
    0x6666666666666666,
    0xdbdbdbdbdbdbdbdb,
    0x0000000000000000};
static const size_t symbols_size = sizeof(symbols) - sizeof(uint64_t) * CASCADE_SIZE;

enum
{
    SYMBOL_SQUAREFULL = 20,
    SYMBOL_SQUARE,
    SYMBOL_CIRCLE,
    SYMBOL_TRIANGLE,
    SYMBOL_CROSS,
    SYMBOL_I,
    SYMBOL_D,
    SYMBOL_S,
    SYMBOL_B,
    SYMBOL_ARROW_FWD,
    SYMBOL_ROMAN_I,
    SYMBOL_ROMAN_II,
    SYMBOL_ROMAN_III,
    SYMBOL_BLANK
};

#define SYMBOL_SCENE0 SYMBOL_ROMAN_I
#define SYMBOL_SCENE1 SYMBOL_ROMAN_II
#define SYMBOL_SCENE2 SYMBOL_ROMAN_III

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

typedef enum
{
    MODE_MENU = 0,
    MODE_PRESETS_IMMEDIATE,
    MODE_PRESETS_DELAYED,
    MODE_SCENES,
    MODE_BRIGHTNESS
} mode_e;

unsigned char modeSymbol(mode_e mode)
{
    switch (mode)
    {
    case MODE_PRESETS_IMMEDIATE:
        return SYMBOL_I;
    case MODE_PRESETS_DELAYED:
        return SYMBOL_D;
    case MODE_SCENES:
        return SYMBOL_S;
    case MODE_BRIGHTNESS:
        return SYMBOL_B;
    default:
        return SYMBOL_CROSS;
    }
}

unsigned char sceneSymbol(uint8_t scene)
{
    switch (scene)
    {
    case 0:
        return SYMBOL_SCENE0;
    case 1:
        return SYMBOL_SCENE1;
    case 2:
        return SYMBOL_SCENE2;
    default:
        return SYMBOL_CROSS;
    }
}

static mode_e mode = MODE_PRESETS_IMMEDIATE;
static mode_e newMode = MODE_PRESETS_IMMEDIATE;
static int8_t newPreset = -1;
static int8_t prevPreset = -1;
static bool heldDebounce = false;

static uint8_t scene = 0;
static uint8_t brightness = 0;

typedef struct
{
    uint8_t presetA;
    uint8_t presetB;
} scene_t;

static scene_t scenePresets[] = {{1, 15}, {1, 15}, {1, 15}};

#define SCENE_CHANGE_IS_PENDING() (scenePresets[scene].presetA != tonex->getPreset(Slot::A) || scenePresets[scene].presetB != tonex->getPreset(Slot::B))

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
                case STOMP_C_PIN:
                    if (ev.event == BUTTON_UP)
                    {
                        if (mode == MODE_PRESETS_IMMEDIATE && !heldDebounce)
                        {
                            tonex->setSlot(currentSlot == Slot::A ? Slot::B : Slot::A);
                        }
                        else if (mode == MODE_PRESETS_DELAYED && !heldDebounce)
                        {
                            if (newPreset != preset)
                            {
                                tonex->changePreset(currentSlot, (uint8_t)newPreset);
                                prevPreset = preset;
                            }
                            else
                            {
                                tonex->changePreset(currentSlot, (uint8_t)prevPreset);
                                newPreset = prevPreset;
                                prevPreset = preset;
                            }
                        }
                        else if (mode == MODE_SCENES && !heldDebounce)
                        {
                            if (SCENE_CHANGE_IS_PENDING())
                            {
                                if (1 == scene)
                                {
                                    scenePresets[scene].presetA = tonex->getPreset(Slot::A);
                                    scenePresets[scene].presetB = tonex->getPreset(Slot::B);
                                }
                            }
                            else
                            {
                                scene = 1;
                                tonex->changePreset(Slot::A, scenePresets[scene].presetA);
                                tonex->changePreset(Slot::B, scenePresets[scene].presetB);
                            }
                        }
                        heldDebounce = false;
                    }
                    else if (ev.event == BUTTON_HELD && !heldDebounce)
                    {
                        if (mode != MODE_MENU)
                        {
                            newMode = mode;
                            mode = MODE_MENU;
                        }
                        else
                        {
                            mode = newMode;
                        }

                        heldDebounce = true;
                        newPreset = preset;
                    }
                    break;
                case STOMP_L_PIN:
                case STOMP_R_PIN:
                {
                    if (ev.event == BUTTON_UP && mode == MODE_SCENES)
                    {
                        if (SCENE_CHANGE_IS_PENDING())
                        {
                            if ((ev.pin == STOMP_L_PIN && scene == 0) || (ev.pin == STOMP_R_PIN && scene == 2))
                            {
                                scenePresets[scene].presetA = tonex->getPreset(Slot::A);
                                scenePresets[scene].presetB = tonex->getPreset(Slot::B);
                            }
                        }
                        else
                        {
                            scene = ev.pin == STOMP_L_PIN ? 0 : 2;
                            tonex->changePreset(Slot::A, scenePresets[scene].presetA);
                            tonex->changePreset(Slot::B, scenePresets[scene].presetB);
                        }
                    }
                    else if (ev.event == BUTTON_DOWN)
                    {
                        if (mode == MODE_MENU)
                        {
                            if (ev.pin == STOMP_L_PIN)
                            {
                                if (newMode > 1)
                                    newMode = (mode_e)((int)newMode - 1);
                            }
                            else
                            {
                                if (newMode < MODE_BRIGHTNESS)
                                    newMode = (mode_e)((int)newMode + 1);
                            }
                        }
                        else if (mode == MODE_PRESETS_IMMEDIATE || mode == MODE_PRESETS_DELAYED)
                        {
                            if (mode == MODE_PRESETS_IMMEDIATE || newPreset < 0)
                            {
                                newPreset = preset + (ev.pin == STOMP_L_PIN ? -1 : 1);
                            }
                            else
                            {
                                newPreset += (ev.pin == STOMP_L_PIN ? -1 : 1);
                            }

                            if (newPreset < 0)
                                newPreset = 19;
                            if (newPreset > 19)
                                newPreset = 0;

                            if (mode == MODE_PRESETS_IMMEDIATE)
                                tonex->changePreset(currentSlot, (uint8_t)newPreset);
                        }
                        else if (mode == MODE_BRIGHTNESS)
                        {
                            if (ev.pin == STOMP_L_PIN)
                            {
                                if (brightness > 0)
                                    brightness--;
                            }
                            else
                            {
                                if (brightness < MAX7219_MAX_BRIGHTNESS)
                                    brightness++;
                            }
                        }
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

        spi_bus_config_t cfg = {
            .mosi_io_num = MOSI_PIN,
            .miso_io_num = -1,
            .sclk_io_num = CLK_PIN,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 0,
            .flags = 0};
        ESP_ERROR_CHECK(spi_bus_initialize(HOST, &cfg, SPI_DMA_CH_AUTO));

        max7219_t dev = {
            .digits = 0,
            .cascade_size = CASCADE_SIZE,
            .mirrored = true};
        ESP_ERROR_CHECK(max7219_init_desc(&dev, HOST, MAX7219_MAX_CLOCK_SPEED_HZ, GPIO_NUM_10));
        ESP_ERROR_CHECK(max7219_init(&dev));

        max7219_set_brightness(&dev, brightness);

        uint8_t i = 0;
        while (1)
        {
            switch (tonex->getConnectionState())
            {
            case StateInitialized:
                switch (mode)
                {
                case MODE_MENU:
                    max7219_draw_image_8x8(&dev, 8, (uint8_t *)symbols + 8 * modeSymbol(newMode));
                    max7219_draw_image_8x8(&dev, 0, (uint8_t *)symbols + 8 * (SYMBOL_ARROW_FWD));
                    if (0 == (i % 10))
                        max7219_draw_image_8x8(&dev, 8, (uint8_t *)symbols + 8 * (SYMBOL_BLANK));
                    break;
                case MODE_PRESETS_IMMEDIATE:
                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 4, 0));
                    ESP_ERROR_CHECK(led_strip_refresh(led_strip));

                    max7219_draw_image_8x8(&dev, 0, (uint8_t *)symbols + 8 * modeSymbol(mode));
                    max7219_draw_image_8x8(&dev, 8, (uint8_t *)symbols + 8 * (tonex->getPreset(tonex->getCurrentSlot()) % 20));
                    break;
                case MODE_PRESETS_DELAYED:
                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 4, 4, 0));
                    ESP_ERROR_CHECK(led_strip_refresh(led_strip));

                    max7219_draw_image_8x8(&dev, 0, (uint8_t *)symbols + 8 * modeSymbol(mode));
                    max7219_draw_image_8x8(&dev, 8, (uint8_t *)symbols + 8 * (newPreset % 20));
                    if ((newPreset == tonex->getPreset(tonex->getCurrentSlot()) && 0 == (i % 10)) ||
                        (newPreset != tonex->getPreset(tonex->getCurrentSlot()) && 0 == (i % 5)))
                        max7219_draw_image_8x8(&dev, 8, (uint8_t *)symbols + 8 * (SYMBOL_BLANK));
                    break;
                case MODE_SCENES:
                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 4, 0, 4));
                    ESP_ERROR_CHECK(led_strip_refresh(led_strip));

                    max7219_draw_image_8x8(&dev, 0, (uint8_t *)symbols + 8 * sceneSymbol(scene));
                    max7219_draw_image_8x8(&dev, 8, (uint8_t *)symbols + 8 * (tonex->getPreset(tonex->getCurrentSlot()) % 20));

                    if (SCENE_CHANGE_IS_PENDING() && 0 == (i % 5))
                        max7219_draw_image_8x8(&dev, 8, (uint8_t *)symbols + 8 * (SYMBOL_BLANK));
                    break;
                case MODE_BRIGHTNESS:
                    max7219_draw_image_8x8(&dev, 0, (uint8_t *)symbols + 8 * modeSymbol(mode));
                    max7219_draw_image_8x8(&dev, 8, (uint8_t *)symbols + 8 * (brightness));
                    max7219_set_brightness(&dev, brightness);
                    break;
                default:
                    break;
                }
                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;
            default:
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 4, 0, 0));
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                max7219_draw_image_8x8(&dev, 0, (uint8_t *)symbols + 8 * (SYMBOL_CROSS));
                max7219_draw_image_8x8(&dev, 8, (uint8_t *)symbols + 8 * (SYMBOL_CROSS));

                vTaskDelay(500 / portTICK_PERIOD_MS);
                ESP_ERROR_CHECK(led_strip_clear(led_strip));
                max7219_clear(&dev);
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            i++;
        }
    }

    void init(Tonex *tonex)
    {
        xTaskCreatePinnedToCore(pedal_receiver, "pedal_receiver", 4096, tonex, 10, NULL, 0);
        xTaskCreatePinnedToCore(ux_receiver, "ux_receiver", 4096, tonex, 10, NULL, 0);
    }
}