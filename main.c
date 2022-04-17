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
#include "hardware/structs/iobank0.h"
#include "hardware/irq.h"
#include "hardware/structs/systick.h"

#include "bsp/board.h"
#include "tusb.h"

#define N64_DIO_PIN	2

static uint8_t _dev_addr;

static uint16_t m_vid = 0;
static uint16_t m_pid = 0;

static volatile uint8_t enable_vibro = 0;
static volatile uint8_t disable_vibro = 0;

static volatile uint8_t buttons[2] = { 0, 0 };
static volatile uint8_t sticks[2] = { 0, 0 };

static volatile uint8_t use_rumble_pack = 0;

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

extern void xpad_task(void);
extern void hid_app_task(void);

void led_blinking_task(void);
void debug_dump_16(uint8_t *ptr);

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
    _dev_addr = dev_addr;

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
    } else if (report[0] == 0x07 && report[1] == 0x20) {
	if (report[4] & 0x01) {
	    use_rumble_pack = !use_rumble_pack;
	}
    }
    //debug_dump_16(report);
}

static uint8_t start_vibro[] = {
    0x09, 0x08, 0x00,
    0x09, 0x00, 0x0f,
    0x20, 0x20, 0x20, 0x20,
    0x20, 0x00
};

void xpad_task(void)
{
    if (enable_vibro == 1) {
//	printf("Start vibro\n");
	start_vibro[5] = 0x0f;
	tuh_xpad_write(_dev_addr, start_vibro, 12);
	enable_vibro = 0;
    }

    if (disable_vibro == 1) {
//	printf("Stop vibro\n");
	//start_vibro[5] = 0x00;
	//tuh_xpad_write(_dev_addr, start_vibro, 12);
	disable_vibro = 0;
    }
}

void led_blinking_task(void)
{
    const uint32_t interval_ms = use_rumble_pack ? 500 : 1000;
    static uint32_t start_ms = 0;

    static bool led_state = false;

    // Blink every interval ms
    if ( board_millis() - start_ms < interval_ms) return; // not enough time
    start_ms += interval_ms;

    board_led_write(led_state);
    led_state = 1 - led_state; // toggle
}

void debug_dump_16(uint8_t *ptr)
{
    int i;
    char tmp[128];
    const char dig[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

    memset(tmp, ' ', sizeof(tmp));
    tmp[127] = 0;

    for (i = 0; i < 16; i++) {
	tmp[i * 3 + 0] = dig[ptr[i] >> 4];
	tmp[i * 3 + 1] = dig[ptr[i] & 0xf];
	tmp[i * 3 + 2] = ' ';
	tmp[49 + i] = (ptr[i] >= 32) ? ptr[i] : '.';
    }
    tmp[48] = ' ';
    tmp[65] = 0;
    printf("DUMP16: %s\n", tmp);
}

static uint16_t __not_in_flash_func(calc_address_crc)(uint16_t address)
{
    /* CRC table */
    uint16_t xor_table[16] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x15, 0x1F, 0x0B, 0x16, 0x19, 0x07, 0x0E, 0x1C, 0x0D, 0x1A, 0x01 };
    uint16_t crc = 0;

    /* Make sure we have a valid address */
    address &= ~0x1F;

    /* Go through each bit in the address, and if set, xor the right value into the output */
    for(int i = 15; i >= 5; i--) {
        /* Is this bit set? */
        if(((address >> i) & 0x1)) {
            crc ^= xor_table[i];
        }
    }

    /* Just in case */
    crc &= 0x1F;

    /* Create a new address with the CRC appended */
    return address | crc;
}

static uint8_t __not_in_flash_func(calc_data_crc)( uint8_t *data )
{
    uint8_t ret = 0;

    for(int i = 0; i <= 32; i++) {
        for(int j = 7; j >= 0; j--) {
            int tmp = 0;

            if(ret & 0x80) {
                tmp = 0x85;
            }

            ret <<= 1;

            if(i < 32) {
                if(data[i] & (0x01 << j)) {
                    ret |= 0x1;
                }
            }
            ret ^= tmp;
        }
    }

    return ret;
}

static inline uint8_t reverse(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

#define TICKS_1US	124

static inline void wait_ticks(uint32_t count)
{
    systick_hw->rvr = 0xFFFFFF;
    systick_hw->csr = 0x5;
    uint32_t old = systick_hw->cvr;
    while (old - systick_hw->cvr < count) {
    }
}

static inline void write_1()
{
    gpio_set_dir(N64_DIO_PIN, GPIO_OUT);
    wait_ticks(TICKS_1US);
    gpio_set_dir(N64_DIO_PIN, GPIO_IN);
    wait_ticks(TICKS_1US * 3);
}

static inline void write_0()
{
    gpio_set_dir(N64_DIO_PIN, GPIO_OUT);
    wait_ticks(TICKS_1US * 3);
    gpio_set_dir(N64_DIO_PIN, GPIO_IN);
    wait_ticks(TICKS_1US);
}

static inline void send_stop()
{
    gpio_set_dir(N64_DIO_PIN, GPIO_OUT);
    wait_ticks(TICKS_1US * 2);
    gpio_set_dir(N64_DIO_PIN, GPIO_IN);
    wait_ticks(TICKS_1US);
}

// send a byte from LSB to MSB (proper serialization)
static void __not_in_flash_func(send_byte)(uint8_t b)
{
    for(int i = 0;i < 8;i++) { // send all 8 bits, one at a time
        if((b >> i) & 1) {
            write_1();
        } else {
            write_0();
        }
    }
}

static uint8_t data_block[32];
static uint8_t memory_pak[32768];

static int __not_in_flash_func(read_data_block)(uint8_t *data_block)
{
    uint8_t byte = 0;
    int bits_read = 0;
    int bytes_read = 0;

    while (1) {
	wait_ticks(TICKS_1US * 2);
	byte <<= 1;
	byte |= (gpio_get(N64_DIO_PIN) ? 1 : 0);

	bits_read++;

	if (bits_read == 8) {
	    data_block[bytes_read++] = byte;
	    byte = 0;
	    bits_read = 0;
	    if (bytes_read == 32) {
		wait_ticks(TICKS_1US * 2); // console stop bit
		wait_ticks(TICKS_1US);     //
		uint8_t crc = calc_data_crc(data_block);
		wait_ticks(TICKS_1US * 3);
		send_byte(reverse(crc));
		send_stop();
		return bytes_read;
	    }
	}

	int timeout = 300;
	while(!gpio_get(N64_DIO_PIN) && timeout--) {
	}

	if (timeout != 0) {
	    timeout = 300;
	    while(gpio_get(N64_DIO_PIN) && timeout--) {
	    }
	}

	if (timeout == 0) {
	    return -3;
	}
    }
}

static int __not_in_flash_func(write_data_block)(uint8_t *data_block)
{
    uint8_t crc = calc_data_crc(data_block);
    for (int i = 0; i < 32; i++) {
	send_byte(reverse(data_block[i]));
    }
    send_byte(reverse(crc));
    send_stop();
}

static uint32_t __not_in_flash_func(read_command)()
{
    int bits_read = 0;
    uint32_t command = 0;
    int timeout;

    while (1) {
	wait_ticks(TICKS_1US * 2);
	command <<= 1;
	command |= (gpio_get(N64_DIO_PIN) ? 1 : 0);

	bits_read++;

	if (bits_read == 9) {
	    // if not command 0x02 and 0x03
	    if ((command >> 1) != 0x02 && (command >> 1) != 0x03) {
		return command >> 1;
	    }
	}

	// command 0x02 + address 2 bytes + stop bit
//	if (bits_read == 25 && (command >> 16) == 0x02) {
//	    return command >> 1;
//	}

	timeout = 300;
	while(!gpio_get(N64_DIO_PIN) && timeout--) {
	}

	if (timeout != 0) {
	    timeout = 300;
	    while(gpio_get(N64_DIO_PIN) && timeout--) {
	    }
	}

	if (timeout == 0) {
	    return -1;
	}

	// command 0x03 + address 2 bytes
	if (bits_read == 24) {
	    if ((command >> 16) == 0x03) {
		if (read_data_block(data_block) != 32) {
		    return -2;
		}

		uint32_t addr = command & 0xFFE0;

		if (addr < 0x8000) {
		    memmove(&memory_pak[addr], data_block, 32);
		} else if (use_rumble_pack && (command & 0xFFE0) == 0xC000) {
		    if (data_block[0] == 0x00) {
			// stop rumble pack
			disable_vibro = 1;
		    } else {
			// start rumble pack
			enable_vibro = 1;
		    }
		}
	    } else {
		wait_ticks(TICKS_1US * 3); // skip console stop bit

		uint32_t addr = command & 0xFFE0;

		if (addr < 0x8000) {
		    memmove(data_block, &memory_pak[addr], 32);
		} else if (use_rumble_pack) {
		    memset(data_block, 0x80, 32);
		} else {
		    memset(data_block, 0x00, 32);
		}
		write_data_block(data_block);
	    }
	    return command;
	}
    }
}

static void __not_in_flash_func(gpio_irq_handler)(void)
{
    io_irq_ctrl_hw_t *irq_ctrl_base = get_core_num() ?
                                           &iobank0_hw->proc1_irq_ctrl : &iobank0_hw->proc0_irq_ctrl;

    uint gpio = N64_DIO_PIN;
    io_ro_32 *status_reg = &irq_ctrl_base->ints[gpio / 8];
    uint events = (*status_reg >> 4 * (gpio % 8)) & 0xf;

    if (events & GPIO_IRQ_EDGE_FALL) {
//        gpio_acknowledge_irq(gpio, events);

	uint32_t cmd = read_command();

//	cmd >>= 1;

	if (cmd == 0x00) {
	    send_byte(reverse(0x05));
	    send_byte(reverse(0x00));
	    send_byte(reverse(0x01));
	    send_stop();
	} else if (cmd == 0x01) {
	    send_byte(reverse(buttons[0]));
	    send_byte(reverse(buttons[1]));
	    send_byte(reverse(sticks[0]));
	    send_byte(reverse(sticks[1]));
	    send_stop();
	} else if ((cmd >> 16) == 0x03) {
//	    debug_dump_16(&data_block[0]);
//	    debug_dump_16(&data_block[16]);
	    //printf("Addr crc %04X, data crc %02X\n", calc_address_crc(cmd & 0xffff), calc_data_crc(data_block));
//	    printf("Addr crc %04X\n", calc_addr_crc(cmd & 0xffff));
//	    printf("[%02X]\n", data_block[0]);
	} else if ((cmd >> 16) == 0x02) {
	
	}

//	if (cmd != 0x00 && cmd != 0x01) {
//	    printf("Get command %X\n", cmd);
//	}
        gpio_acknowledge_irq(gpio, events);
    } else {
	printf("unk irq\n");
    }
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
    gpio_put(N64_DIO_PIN, 0);
    gpio_pull_up(N64_DIO_PIN);
    gpio_set_dir(N64_DIO_PIN, GPIO_IN);

    multicore_reset_core1();
    multicore_launch_core1(usb_host_process);

    TU_LOG2("Controller enabled.\n");

    gpio_acknowledge_irq(N64_DIO_PIN, GPIO_IRQ_EDGE_FALL);
    gpio_set_irq_enabled(N64_DIO_PIN, GPIO_IRQ_EDGE_FALL, true);

    irq_set_exclusive_handler(IO_IRQ_BANK0, gpio_irq_handler);
    irq_set_enabled(IO_IRQ_BANK0, true);

    while (1) {
    }

    return 0;
}
