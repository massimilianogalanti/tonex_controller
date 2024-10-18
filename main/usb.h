/* 
 * MIT License
 *
 * Copyright (c) 2024 vit3k
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once 

#include <memory>
#include <functional>
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

class USB {
private:
    cdc_acm_dev_hdl_t cdc_dev = nullptr;
    bool connected = false;
    std::function<void(const std::vector<uint8_t>&)> onMessageCallback;
    std::function<void(void)> onConnectionCallback;
    uint16_t vid;
    uint16_t pid;
    USB() = default;
public:
    static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *arg);
    static bool handle_rx(const uint8_t *data, size_t data_len, void *arg);
    static void usb_host_task(void* arg);
    static std::unique_ptr<USB> init(uint16_t vid, uint16_t pid, std::function<void(const std::vector<uint8_t>&)> onMessageCallback);
    void send(const std::vector<uint8_t>& data);
    void setConnectionCallback(std::function<void(void)> callback);
};
