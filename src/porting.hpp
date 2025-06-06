#pragma once

#include <vector>
#include <cinttypes>

#define HIGH    1
#define LOW     0

//Interrupt Modes
#define DISABLED  0x00
#define RISING    0x01
#define FALLING   0x02

//GPIO FUNCTIONS
#define INPUT 0x01
// Changed OUTPUT from 0x02 to behave the same as Arduino pinMode(pin,OUTPUT)
// where you can read the state of pin even when it is set as OUTPUT
#define OUTPUT            0x03
#define PULLUP            0x04
#define INPUT_PULLUP      0x05
#define PULLDOWN          0x08
#define INPUT_PULLDOWN    0x09
#define OPEN_DRAIN        0x10
#define OUTPUT_OPEN_DRAIN 0x13
#define ANALOG            0xC0

typedef void (*spi_init_ptr)();
typedef uint8_t (*spi_transfer_ptr)(uint8_t data);

typedef void (*gpio_mode_ptr)(int pin, int mode);
typedef void (*attach_irq_ptr)(int pin, void (*)(), int mode);
typedef void (*detach_irq_ptr)(int pin);
typedef void (*gpio_set_level_ptr)(int pin, int level);
typedef uint8_t (*gpio_read_ptr)(int pin);

typedef void (*delay_ms_ptr)(uint32_t ms);
// typedef void (*print_ptr)(__VA_ARGS__);

struct ctx_t
{
    spi_init_ptr   spi_init;
    spi_transfer_ptr  spi_transfer;

    gpio_mode_ptr   gpio_mode;
    attach_irq_ptr attach_irq;
    detach_irq_ptr detach_irq;
    gpio_set_level_ptr   gpio_set_level;
    gpio_read_ptr gpio_read;

    delay_ms_ptr   delay_ms;

    // print_ptr print;
};

extern ctx_t ctx;
