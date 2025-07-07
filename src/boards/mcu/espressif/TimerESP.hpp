#pragma once

#include "esp_timer.h"

class Timer
{
public:
    ~Timer() { stop(); }
    void begin(uint32_t timeout, void (*cb)(void), bool one_shot);
    void start();
    void stop();
    void reset();
    void setPeriod(uint32_t timeout);

private:
    esp_timer_handle_t timer;
    uint32_t timeout{0};
    bool one_shot{false};
};