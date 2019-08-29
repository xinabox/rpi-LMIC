/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/

#ifdef RASPBERRY_PI
#include "raspi/raspi.h"
#else
#include <Arduino.h>
#include <SPI.h>
#endif
#include "../lmic.h"
#include "hal.h"
#include <stdio.h>
#inlcude "xXI02.h"

xXI02 spi;


// -----------------------------------------------------------------------------
// I/O
// default we do not use IRQ line and DIO output
static bool check_dio = 0;

static void hal_io_init () {
    // NSS is required
    ASSERT(lmic_pins.nss != LMIC_UNUSED_PIN);

    // No more needed, if dio pins are declared as unused, then LIMC will check
    // interrputs directly into Lora module register, avoiding needed GPIO line to IRQ
    //ASSERT(lmic_pins.dio[0] != LMIC_UNUSED_PIN);
    //ASSERT(lmic_pins.dio[1] != LMIC_UNUSED_PIN || lmic_pins.dio[2] != LMIC_UNUSED_PIN);

    //pinMode(lmic_pins.nss, OUTPUT);
    if (lmic_pins.rxtx != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.rxtx, OUTPUT);
    if (lmic_pins.rst != LMIC_UNUSED_PIN)
        spi.pinMode(lmic_pins.rst, OUTPUT);

    // if using DIO lines, DIO0 is always required, DIO1 is required for LoRa, DIO2 for FSK
    for (uint8_t i = 0; i < NUM_DIO; ++i) {
        if (lmic_pins.dio[i] != LMIC_UNUSED_PIN) {
            check_dio = 1; // we need to use DIO line check
            pinMode(lmic_pins.dio[i], INPUT);
            
// #ifdef RASPBERRY_PI
//             // Enable pull down an rising edge detection on this one
//             bcm2835_gpio_set_pud(lmic_pins.dio[i], BCM2835_GPIO_PUD_DOWN);
//             bcm2835_gpio_ren(lmic_pins.dio[i]);
// #endif

        }
    }
}

// val == 1  => tx 1
void hal_pin_rxtx (u1_t val) {
    if (lmic_pins.rxtx != LMIC_UNUSED_PIN)
        digitalWrite(lmic_pins.rxtx, val);
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    if (lmic_pins.rst == LMIC_UNUSED_PIN)
        return;

    if(val == 0 || val == 1) { // drive pin
        spi.pinMode(lmic_pins.rst, OUTPUT);
        spi.digitalWrite(lmic_pins.rst, val);
    } else { // keep pin floating
        spi.pinMode(lmic_pins.rst, INPUT);
    }
}

static bool dio_states[NUM_DIO] = {0};

static void hal_io_check() {
    // We have DIO line connected?
    if (check_dio == 1) {
        uint8_t i;
        for (i = 0; i < NUM_DIO; ++i) {
          
// #ifdef RASPBERRY_PI
//             // Rising edge fired ?
//             if (bcm2835_gpio_eds(lmic_pins.dio[i])) {
//                 // Now clear the eds flag by setting it to 1
//                 bcm2835_gpio_set_eds(lmic_pins.dio[i]);
//                 // Handle pseudo interrupt
//                 radio_irq_handler(i);
//             }
// #else
            if (dio_states[i] != spi.digitalRead(lmic_pins.dio[i])) {
                dio_states[i] = !dio_states[i];
                if (dio_states[i]) {
                    radio_irq_handler(i);
                }
            }
#endif
        } // For
        
    } else {
        // Check IRQ flags in radio module
        if ( radio_has_irq() ) 
            radio_irq_handler(0);
    }
}
// -----------------------------------------------------------------------------
// SPI

static void hal_spi_init () {
    spi.begin(3);
    spi.pinMode(lmic_pins.nss, OUTPUT);
    spi.pinMode(lmic_pins.rst, OUTPUT);
    spi.digitalWrite(lmic_pins.rst, HIGH);
}

// #ifdef RASPBERRY_PI
//     // Clock divider / 32 = 8MHz
//     static const SPISettings settings(BCM2835_SPI_CLOCK_DIVIDER_32 , BCM2835_SPI_BIT_ORDER_MSBFIRST, BCM2835_SPI_MODE0);
// #else
//     static const SPISettings settings(10E6, MSBFIRST, SPI_MODE0);
// #endif

void hal_pin_nss (u1_t val) {
    spi.digitalWrite(lmic_pins.nss,val);
//     if (!val)
//         SPI.beginTransaction(settings);
//     else
//         SPI.endTransaction();

//     //Serial.println(val?">>":"<<");
//     digitalWrite(lmic_pins.nss, val);
}


// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
    u1_t res = spi.transfer(out);
/*
    Serial.print(">");
    Serial.print(out, HEX);
    Serial.print("<");
    Serial.println(res, HEX);
    */
    return res;
}

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init () {
    // Nothing to do
}

u4_t hal_ticks () {
    // Because micros() is scaled down in this function, micros() will
    // overflow before the tick timer should, causing the tick timer to
    // miss a significant part of its values if not corrected. To fix
    // this, the "overflow" serves as an overflow area for the micros()
    // counter. It consists of three parts:
    //  - The US_PER_OSTICK upper bits are effectively an extension for
    //    the micros() counter and are added to the result of this
    //    function.
    //  - The next bit overlaps with the most significant bit of
    //    micros(). This is used to detect micros() overflows.
    //  - The remaining bits are always zero.
    //
    // By comparing the overlapping bit with the corresponding bit in
    // the micros() return value, overflows can be detected and the
    // upper bits are incremented. This is done using some clever
    // bitwise operations, to remove the need for comparisons and a
    // jumps, which should result in efficient code. By avoiding shifts
    // other than by multiples of 8 as much as possible, this is also
    // efficient on AVR (which only has 1-bit shifts).
    static uint8_t overflow = 0;

    // Scaled down timestamp. The top US_PER_OSTICK_EXPONENT bits are 0,
    // the others will be the lower bits of our return value.
    uint32_t scaled = micros() >> US_PER_OSTICK_EXPONENT;
    // Most significant byte of scaled
    uint8_t msb = scaled >> 24;
    // Mask pointing to the overlapping bit in msb and overflow.
    const uint8_t mask = (1 << (7 - US_PER_OSTICK_EXPONENT));
    // Update overflow. If the overlapping bit is different
    // between overflow and msb, it is added to the stored value,
    // so the overlapping bit becomes equal again and, if it changed
    // from 1 to 0, the upper bits are incremented.
    overflow += (msb ^ overflow) & mask;

    // Return the scaled value with the upper bits of stored added. The
    // overlapping bit will be equal and the lower bits will be 0, so
    // bitwise or is a no-op for them.
    return scaled | ((uint32_t)overflow << 24);

    // 0 leads to correct, but overly complex code (it could just return
    // micros() unmodified), 8 leaves no room for the overlapping bit.
    static_assert(US_PER_OSTICK_EXPONENT > 0 && US_PER_OSTICK_EXPONENT < 8, "Invalid US_PER_OSTICK_EXPONENT value");
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}

void hal_waitUntil (u4_t time) {
    s4_t delta = delta_time(time);
    // From delayMicroseconds docs: Currently, the largest value that
    // will produce an accurate delay is 16383.
    while (delta > (16000 / US_PER_OSTICK)) {
        delay(16);
        delta -= (16000 / US_PER_OSTICK);
    }
    if (delta > 0)
        delayMicroseconds(delta * US_PER_OSTICK);
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    // No need to schedule wakeup, since we're not sleeping
    return delta_time(time) <= 0;
}

static uint8_t irqlevel = 0;

void hal_disableIRQs () {
    noInterrupts();
    irqlevel++;
}

void hal_enableIRQs () {
    if(--irqlevel == 0) {
        interrupts();

        // Instead of using proper interrupts (which are a bit tricky
        // and/or not available on all pins on AVR), just poll the pin
        // values. Since os_runloop disables and re-enables interrupts,
        // putting this here makes sure we check at least once every
        // loop.
        //
        // As an additional bonus, this prevents the can of worms that
        // we would otherwise get for running SPI transfers inside ISRs
        hal_io_check();
    }
}

void hal_sleep () {
    // Not implemented
}

// -----------------------------------------------------------------------------

#if defined(LMIC_PRINTF_TO)
static int uart_putchar (char c, FILE *)
{
    LMIC_PRINTF_TO.write(c) ;
    return 0 ;
}

void hal_printf_init() {
    // create a FILE structure to reference our UART output function
    static FILE uartout;
    memset(&uartout, 0, sizeof(uartout));

    // fill in the UART file descriptor with pointer to writer.
    fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);

    // The uart is the standard output device STDOUT.
    stdout = &uartout ;
}
#endif // defined(LMIC_PRINTF_TO)

void hal_init () {
    // configure radio SPI
    hal_spi_init();
    // configure radio I/O and interrupt handler
    hal_io_init();

    // configure timer and interrupt handler
    hal_time_init();
#if defined(LMIC_PRINTF_TO)
    // printf support
    hal_printf_init();
#endif
}

void hal_failed (const char *file, u2_t line) {
#if defined(LMIC_FAILURE_TO)
    LMIC_FAILURE_TO.println("FAILURE ");
    LMIC_FAILURE_TO.print(file);
    LMIC_FAILURE_TO.print(':');
    LMIC_FAILURE_TO.println(line);
    LMIC_FAILURE_TO.flush();
#endif
    hal_disableIRQs();
    while(1);
}
