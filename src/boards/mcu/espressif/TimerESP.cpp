#include "TimerESP.h"

void Timer::begin(uint32_t timeout, void (*cb)(void), bool one_shot)
{
    this->one_shot = one_shot;
    this->timeout = timeout;

    const esp_timer_create_args_t args = {
        .callback = &cb,
        .arg = this,
        .name = "timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&args, &timer));

    start();
}

void Timer::start()
{
    if (esp_timer_is_active(timer))
        esp_timer_stop(timer);

    if (one_shot)
        esp_timer_start_once(timer, timeout);
    else
        esp_timer_start_periodic(timer, timeout);
}

void Timer::stop()
{
    if (esp_timer_is_active(timer))
        esp_timer_stop(timer);
}

void Timer::reset()
{
    start();
}

void Timer::setPeriod(uint32_t timeout)
{
    this->timeout = timeout;
    start();
}