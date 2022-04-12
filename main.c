/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"

#include "bsp/board.h"
#include "tusb.h"

#define N64_DIO_PIN	2

static uint16_t m_vid = 0;
static uint16_t m_pid = 0;

static volatile uint8_t buttons[2] = { 0, 0 };
static volatile uint8_t sticks[2] = { 0, 0 };

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+
void led_blinking_task(void);

extern void xpad_task(void);
extern void hid_app_task(void);
extern void my_wait_us_asm(int n);

/*------------- MAIN -------------*/
void usb_host_process(void)
{
    tusb_init();

    while (1) {
	tuh_task();

	led_blinking_task();

#if CFG_TUH_XPAD
	xpad_task();
#endif

#if CFG_TUH_HID
	hid_app_task();
#endif
    }
}

void tuh_mount_cb(uint8_t dev_addr)
{
    tuh_vid_pid_get(dev_addr, &m_vid, &m_pid);

    printf("A device %04X:%04X with address %d is mounted\r\n", m_vid, m_pid, dev_addr);
}

static int8_t analog_value(uint8_t *val8)
{
    int16_t val = *((int16_t *) val8) / 0x190;

    if (val < 10 && val > -10) return 0;

    if (val > 0x50) val = 0x50;

    if (val < -0x50) val = -0x50;

    return val;
}

void tuh_xpad_read_cb(uint8_t dev_addr, uint8_t *report)
{
    uint8_t b = 0;
    uint8_t b1 = 0;

    if (report[0] == 0x20) {
	if (report[5] & 0x01) b |= 0x08; // D-U    D-UP
	if (report[5] & 0x02) b |= 0x04; // D-D    D-D
	if (report[5] & 0x04) b |= 0x02; // D-L    D-L
	if (report[5] & 0x08) b |= 0x01; // D-R    D-R

	if (analog_value(&report[16]) > 40)  b1|= 0x08; // RS-U  C-U
	if (analog_value(&report[16]) < -40) b1|= 0x04; // RS-D  C-D
	if (analog_value(&report[14]) < -40) b1|= 0x02; // RS-L  C-L
	if (analog_value(&report[14]) > 40)  b1|= 0x01; // RS-R  C-R

	if (report[4] & 0x10) b |= 0x80; // A      A
	if (report[4] & 0x20) b |= 0x40; // B      B
	if (*((int16_t *)&report[8]) > 0x180) b |= 0x20; // LT    Z
	if (*((int16_t *)&report[6]) > 0x180) b |= 0x20; // LT    Z
	if (report[4] & 0x04) b |= 0x10; // START  START

	if (report[5] & 0x10) b1|= 0x20; // LB      L
	if (report[5] & 0x20) b1|= 0x10; // RB      R

	buttons[0] = b;
	buttons[1] = b1;
	sticks[0] = analog_value(&report[10]);
	sticks[1] = analog_value(&report[12]);

	//printf("RS %d %d\r\n", analog_value(&report[14]), analog_value(&report[16]));
    }
}

void xpad_task(void)
{

}

void led_blinking_task(void)
{
    const uint32_t interval_ms = 1000;
    static uint32_t start_ms = 0;

    static bool led_state = false;

    // Blink every interval ms
    if ( board_millis() - start_ms < interval_ms) return; // not enough time
    start_ms += interval_ms;

    board_led_write(led_state);
    led_state = 1 - led_state; // toggle
}

static uint8_t reverse(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

static void write_1()
{
    gpio_put(N64_DIO_PIN, 0);
    my_wait_us_asm(1);
    gpio_put(N64_DIO_PIN, 1);
    my_wait_us_asm(1);
    my_wait_us_asm(1);
    my_wait_us_asm(1);
}

static void write_0()
{
    gpio_put(N64_DIO_PIN, 0);
    my_wait_us_asm(1);
    my_wait_us_asm(1);
    my_wait_us_asm(1);
    gpio_put(N64_DIO_PIN, 1);
    my_wait_us_asm(1);
}

static void send_stop()
{
    gpio_put(N64_DIO_PIN, 0);
    my_wait_us_asm(1);
    gpio_put(N64_DIO_PIN, 1);
}

// send a byte from LSB to MSB (proper serialization)
static void send_byte(uint8_t b)
{
    for(int i = 0;i < 8;i++) { // send all 8 bits, one at a time
        if((b >> i) & 1) {
            write_1();
        } else {
            write_0();
        }
    }
}

static uint8_t get_middle_of_pulse()
{
    uint32_t ct = 0;
    // wait for line to go high
    while(!gpio_get(N64_DIO_PIN)) {
        ct++;
        if(ct == 200000) { // failsafe limit TBD
	    return 0x50; // error code
	}
    }

    ct = 0;
    // wait for line to go low
    while(gpio_get(N64_DIO_PIN)) {
        ct++;
	if(ct == 200000) {// failsafe limit TBD
    	    return 0x51; // error code
	}
    }

    // now we have the falling edge

    // wait 2 microseconds to be in the middle of the pulse, and read. high --> 1.  low --> 0.
    my_wait_us_asm(1);
    my_wait_us_asm(1);

    return gpio_get(N64_DIO_PIN) ? 1 : 0;
}

static uint32_t read_command()
{
    int bits_read = 0;
    uint32_t command = 0;

    while (1) {
	my_wait_us_asm(1);
	my_wait_us_asm(1);
	command <<= 1;
	command |= (gpio_get(N64_DIO_PIN) ? 1 : 0);

	bits_read++;
	if (bits_read == 9) {
	    break;
	}

	while(!gpio_get(N64_DIO_PIN)) {
	}

	while(gpio_get(N64_DIO_PIN)) {
	}
    }

    return command;
}

int main(void)
{
    board_init();

    printf("USB to N64 adapter\n");

    if (watchdog_caused_reboot()) {
        TU_LOG2("Rebooted by Watchdog!\n");
    } else {
        TU_LOG2("Clean boot\n");
    }

    TU_LOG2("clock sys = %d\n", clock_get_hz(clk_sys));

    gpio_init(N64_DIO_PIN);
    gpio_pull_up(N64_DIO_PIN);
    gpio_set_dir(N64_DIO_PIN, GPIO_IN);

    gpio_init(N64_DIO_PIN+1);
    gpio_pull_up(N64_DIO_PIN+1);
    gpio_set_dir(N64_DIO_PIN+1, GPIO_OUT);

    TU_LOG2("Controller enabled.\n");

    int wd_enabled = 0;

    while (1) {
	while(!gpio_get(N64_DIO_PIN)) {
	}

	gpio_put(N64_DIO_PIN+1, 1);
	while(gpio_get(N64_DIO_PIN)) {
	}

	uint32_t irq_status = save_and_disable_interrupts();

	gpio_put(N64_DIO_PIN+1, 0);

	uint32_t cmd = read_command();

	cmd >>= 1;

	if (cmd == 0x00) {
	    gpio_set_dir(N64_DIO_PIN, GPIO_OUT);
	    send_byte(reverse(0x05));
	    send_byte(reverse(0x00));
	    send_byte(reverse(0x02));
	    send_stop();
	    gpio_set_dir(N64_DIO_PIN, GPIO_IN);
	} else if (cmd == 0x01) {
	    gpio_set_dir(N64_DIO_PIN, GPIO_OUT);
	    send_byte(reverse(buttons[0]));
	    send_byte(reverse(buttons[1]));
	    send_byte(reverse(sticks[0]));
	    send_byte(reverse(sticks[1]));
	    send_stop();
	    gpio_set_dir(N64_DIO_PIN, GPIO_IN);
	}

	restore_interrupts(irq_status);

	if (cmd != 0x00 && cmd != 0x01) {
	    printf("Get command %02X\n", cmd);
	}

	if (!wd_enabled) {
	    multicore_reset_core1();
	    multicore_launch_core1(usb_host_process);
	    wd_enabled = 1;
	}

#if 0
        uint32_t value = pio_sm_get_blocking(pio, sm);
        value = value >> 1;

        if (value == 0x0 || value == 0xff) {
            uint32_t msg =
                reverse(0x05) |
                ((reverse(0x00)) << 8) |
                ((reverse(0x02)) << 16);

            pio_sm_put_blocking(pio, sm, 23);
            pio_sm_put_blocking(pio, sm, msg);
        } else if (value == 0x1) {

            uint32_t msg = 0;

            msg =
                reverse(buttons[0]) |
              ((reverse(buttons[1])) << 8) |
              ((reverse(sticks[0])) << 16) |
              ((reverse(sticks[1])) << 24);

            pio_sm_put_blocking(pio, sm, 31);
            pio_sm_put_blocking(pio, sm, msg);
        }

	if (value != 0x00 && value != 0x01) {
	    printf("GOTDATA %u\n", value);
	}

	if (!wd_enabled) {
	    //multicore_reset_core1();
	    //multicore_launch_core1(usb_host_process);
	    wd_enabled = 1;
	}
#endif
    }

    return 0;
}
