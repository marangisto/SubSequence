#define NO_TIMER_VECTORS 1
#include <UART.h>
#include <Main.h>
#include <Delay.h>
#include <Timer.h>
#include <TWI.h>
#include <ADC.h>
#include <Pins.h>
#include <Buttons.h>
#include <WatchDog.h>
#include <stdlib.h>

template <class T> const T& max(const T& a, const T& b) { return (a<b) ? b : a; }
template <class T> const T& min(const T& a, const T& b) { return (a<b) ? a : b; }

static const uint8_t twi_addr = 0x61;   // FIXME: use EEPROM and reset config

//typedef twi_master_t<1> twi;
typedef twi_slave_t<1> twi;

typedef output_t<PB, 2> leds;

typedef output_t<PD, 5> sink_2;
typedef output_t<PD, 6> sink_6;
typedef output_t<PB, 0> sink_7;
typedef output_t<PD, 7> sink_3;
typedef output_t<PD, 4> sink_0;
typedef output_t<PD, 3> sink_4;
typedef output_t<PB, 7> sink_1;
typedef output_t<PB, 6> sink_5;

typedef outputs_t<sink_0, sink_1, sink_2, sink_3, sink_4, sink_5, sink_6, sink_7> sinks;

typedef input_t<PB, 1> sense_a;
typedef input_t<PD, 2> sense_b;

static volatile uint8_t led_state = 0;
static volatile uint8_t swa_state = 0;
static volatile uint8_t swb_state = 0;

static const uint8_t ch0 = 5;
static const uint8_t ch1 = 3;
static const uint8_t ch2 = 0;
static const uint8_t ch3 = 6;
static const uint8_t ch4 = 4;
static const uint8_t ch5 = 2;
static const uint8_t ch6 = 1;
static const uint8_t ch7 = 7;

static volatile uint16_t levels[8];

typedef timer_t<2> aux;

ISR(TIMER2_OVF_vect)
{
    static const uint8_t led_duty = 8; // of 255
    static uint8_t i = 0;

    switch (i)
    {
    case 0:
        sinks::write(~led_state);
        leds::set();
        break;
    case led_duty:
        leds::clear();
        break;
    default:
        if (i > led_duty && i - led_duty - 1 < 8)
        {
            uint8_t bit = 1 << (i - led_duty - 1);
            sinks::write(~bit);
            if (sense_a::read())
                swa_state &= ~bit;
            else
                swa_state |= bit;
        }
        else if (i > led_duty + 8 && i - led_duty - 9 < 8)
        {
            uint8_t bit = 1 << (i - led_duty - 9);
            sinks::write(~bit);
            if (sense_b::read())
                swb_state &= ~bit;
            else
                swb_state |= bit;
        }
    }

    i++;
}

ISR(TWI1_vect)
{
    twi::isr();
}

static inline uint16_t sw_bit(uint8_t i, uint8_t sw)
{
    return (sw & (1 << i)) >> i;
}

static void slave_callback(bool read, volatile uint8_t *buf, uint8_t len)
{
    uint8_t cmd = buf[0];

    switch (cmd)
    {
    case 0:                         // write leds
        if (!read)
            led_state = buf[1];
        break;
    case 1:                         // read data
        if (read)
        {
            uint8_t step = buf[1];
            volatile uint16_t *value = reinterpret_cast<volatile uint16_t*>(buf);
            *value = levels[step];
            wdt_reset();            // give the dog a bone
        }
        break;
    default:
        ;                           // ignore illegal command
    }
}

void setup()
{
    enable_watchdog<512>(); // every 4 seconds

    leds::setup();
    sinks::setup();
    sinks::write(~1);   // first sink active
    adc::setup<128>();

    aux::setup<normal_mode>();
    aux::clock_select<1>();
    aux::enable();

    twi::setup(twi_addr, slave_callback);

    sei();

    for (uint8_t i = 0; i < 8; ++i)
    {
        led_state = i & 0x1 ? 0xff : 0;
        delay_ms(25);
    }

    led_state = 0;
    twi::start();
}

void loop()
{
    for (uint8_t i = 0; i < 8; ++i)
    {
        uint16_t x = 0;

        switch (i)
        {
            case 0: x = adc::read<ch0>(); break;
            case 1: x = adc::read<ch1>(); break;
            case 2: x = adc::read<ch2>(); break;
            case 3: x = adc::read<ch3>(); break;
            case 4: x = adc::read<ch4>(); break;
            case 5: x = adc::read<ch5>(); break;
            case 6: x = adc::read<ch6>(); break;
            case 7: x = adc::read<ch7>(); break;
        }

        uint16_t xab = x | sw_bit(i, swa_state) << 13 | sw_bit(i, swb_state) << 14;

        cli();
        levels[i] = xab;
        sei();
    }
}

