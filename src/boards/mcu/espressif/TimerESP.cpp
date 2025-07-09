#include "TimerESP.hpp"

void TimerESP::begin(uint32_t timeout, void (*cb)(void), bool one_shot)
{
    this->one_shot = one_shot;
    this->timeout = timeout * 1000;

    const esp_timer_create_args_t args = {
        .callback = reinterpret_cast<void(*)(void*)>(cb),
        .arg = this,
        .name = "timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&args, &timer));
}

void TimerESP::start()
{
    if (esp_timer_is_active(timer))
        esp_timer_stop(timer);

    if (one_shot)
        esp_timer_start_once(timer, timeout);
    else
        esp_timer_start_periodic(timer, timeout);
}

void TimerESP::stop()
{
    if (esp_timer_is_active(timer))
        esp_timer_stop(timer);
}

void TimerESP::reset()
{
    start();
}

void TimerESP::setPeriod(uint32_t timeout)
{
    this->timeout = timeout * 1000;
    start();
}