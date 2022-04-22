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
#include "hardware/pio.h"
#include "hardware/flash.h"
#include "hardware/dma.h"

#include "bsp/board.h"
#include "tusb.h"

#include "n64send.pio.h"

#define USE_GPIO_IRQ

#define USE_PIO_DMA

#define N64_DIO_PIN	2

#define FLASH_TARGET_SIZE	(32 * 1024)
#define FLASH_TARGET_OFFSET	(2 * 1024 * 1024 - 32 * 1024)

#define N64SEND_DATA(d0, d1, b) ((((b) - 1) << 16) | ((d0) << 8) | (d1))

const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

static uint8_t _dev_addr;

static uint16_t m_vid = 0;
static uint16_t m_pid = 0;

static volatile uint8_t enable_vibro = 0;
static volatile uint8_t disable_vibro = 0;

static volatile uint8_t buttons[2] = { 0, 0 };
static volatile uint8_t sticks[2] = { 0, 0 };

static volatile uint8_t use_rumble_pack = 0;
static volatile uint8_t memory_pak_changed = 0;
static volatile uint16_t console_is_off = 0;

static uint8_t data_block[32];
static uint8_t memory_pak[32768];

static volatile PIO pio;
static volatile uint sm;
static volatile uint pio_offset;

#ifdef USE_PIO_DMA
static volatile uint32_t pio_dma_chan;

// 16 words (data) + 1 word (crc) + 1 word (stop) + 1 word (reset)
static volatile uint32_t dma_buffer[19] __attribute__((aligned (16)));
#endif

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

#define TICKS_1US	197UL
//#define TICKS_1US	175

static inline void wait_ticks(uint32_t count)
{
    systick_hw->csr = 0x0;
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
//		wait_ticks(TICKS_1US * 2); // console stop bit
//		wait_ticks(TICKS_1US);     //
		uint8_t crc = calc_data_crc(data_block);
//		wait_ticks(TICKS_1US * 3);

		uint32_t data = N64SEND_DATA(crc, 0x00, 8);

		while((pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + sm))) != 0) {}
		pio->txf[sm] = data;

		while((pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + sm))) != 0) {}
		pio->txf[sm] = 0;

		pio_sm_get_blocking(pio, sm);

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
    uint32_t data;

    for (int i = 0; i < 32; i += 2) {
	data = N64SEND_DATA(data_block[i + 0], data_block[i + 1], 16);
	while((pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + sm))) != 0) {}
	pio->txf[sm] = data;
    }
    
    data = N64SEND_DATA(crc, 0x00, 8);

    while((pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + sm))) != 0) {}
    pio->txf[sm] = data;

    while((pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + sm))) != 0) {}
    pio->txf[sm] = 0;

    pio_sm_get_blocking(pio, sm);
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
		command >>= 1;

		wait_ticks(TICKS_1US * 4);

		if (command == 0x00 || command == 0xFF) {
#ifdef USE_PIO_DMA
		    dma_buffer[0] = N64SEND_DATA(0x05, 0x00, 16);
		    dma_buffer[1] = N64SEND_DATA(0x01, 0x00, 8);
		    dma_buffer[2] = 0;

		    pio_sm_exec(pio, sm, pio_encode_jmp(pio_offset + n64send_dma_offset_loop));

		    dma_channel_transfer_from_buffer_now(pio_dma_chan, dma_buffer, 3);
		    dma_channel_wait_for_finish_blocking(pio_dma_chan);

		    while (pio_sm_get_pc(pio, sm) != (pio_offset + n64send_dma_offset_stop)) {}
#else
		    uint32_t data[2] = { N64SEND_DATA(0x05, 0x00, 16), N64SEND_DATA(0x01, 0x00, 8) };

		    while((pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + sm))) != 0) {}
		    pio->txf[sm] = data[0];
		    while((pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + sm))) != 0) {}
		    pio->txf[sm] = data[1];
		    while((pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + sm))) != 0) {}
		    pio->txf[sm] = 0;

		    pio_sm_get_blocking(pio, sm);
#endif
		} else if (command == 0x01) {
#ifdef USE_PIO_DMA
		    dma_buffer[0] = N64SEND_DATA(buttons[0], buttons[1], 16);
		    dma_buffer[1] = N64SEND_DATA(sticks[0], sticks[1], 16);
		    dma_buffer[2] = 0;

		    pio_sm_exec(pio, sm, pio_encode_jmp(pio_offset + n64send_dma_offset_loop));

		    dma_channel_transfer_from_buffer_now(pio_dma_chan, dma_buffer, 3);
		    dma_channel_wait_for_finish_blocking(pio_dma_chan);

		    while (pio_sm_get_pc(pio, sm) != (pio_offset + n64send_dma_offset_stop)) {}
#else
		    uint32_t data[2] = { N64SEND_DATA(buttons[0], buttons[1], 16),  N64SEND_DATA(sticks[0], sticks[1], 16) };

		    while((pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + sm))) != 0) {}
		    pio->txf[sm] = data[0];
		    while((pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + sm))) != 0) {}
		    pio->txf[sm] = data[1];
		    while((pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + sm))) != 0) {}
		    pio->txf[sm] = 0;

		    pio_sm_get_blocking(pio, sm);
#endif
		} else {
		    printf("Unk cmd %X\n", command);
		}

		return command;
	    }
	}

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
		    memory_pak_changed = 1;
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
	uint32_t cmd = read_command();

//	if (cmd != 0x00 && cmd != 0x01) {
//	printf(": %X\n", cmd);
//	}
	console_is_off = 0;

        gpio_acknowledge_irq(gpio, events);
    } else {
	printf("unk irq\n");
    }
}

static void __not_in_flash_func(memory_pak_to_flash)(void)
{
    printf("Write memory pak to flash... ");
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_TARGET_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, memory_pak, FLASH_TARGET_SIZE);
    memory_pak_changed = 0;
    restore_interrupts (ints);
    printf("done\n");
}

static void __not_in_flash_func(main_loop)(void)
{
    while(1) {
#ifdef USE_GPIO_IRQ
	__wfi();
#else
	while(!gpio_get(N64_DIO_PIN)) {
	}

	while(gpio_get(N64_DIO_PIN)) {
	}

	uint32_t cmd = read_command();

	console_is_off = 0;
#endif
    }
}

int main(void)
{
    set_sys_clock_khz(200000, true);

    board_init();

    printf("USB to N64 adapter\n");

    if (watchdog_caused_reboot()) {
        TU_LOG2("Rebooted by Watchdog!\n");
    } else {
        TU_LOG2("Clean boot\n");
    }

    printf("clock sys = %d\n", clock_get_hz(clk_sys));

    console_is_off = 0;
    memory_pak_changed = 0;
    memmove(memory_pak, flash_target_contents, FLASH_TARGET_SIZE);

    gpio_init(N64_DIO_PIN);
    gpio_put(N64_DIO_PIN, 0);
    gpio_pull_up(N64_DIO_PIN);
    gpio_set_dir(N64_DIO_PIN, GPIO_IN);

    {
#ifdef USE_PIO_DMA
	pio = pio0;

	pio_offset = pio_add_program(pio, &n64send_dma_program);

	sm = pio_claim_unused_sm(pio, true);

	pio_dma_chan = dma_claim_unused_channel(true);

	dma_channel_config pio_dma_chan_config = dma_channel_get_default_config(pio_dma_chan);
	channel_config_set_transfer_data_size(&pio_dma_chan_config, DMA_SIZE_32);
	channel_config_set_read_increment(&pio_dma_chan_config, true);
	channel_config_set_write_increment(&pio_dma_chan_config, false);
	channel_config_set_dreq(&pio_dma_chan_config, pio_get_dreq(pio, sm, true));

	dma_channel_configure(
	    pio_dma_chan,
	    &pio_dma_chan_config,
	    &pio->txf[sm],
	    NULL,
	    0,
	    false
	);

	pio_sm_config c = n64send_dma_program_get_default_config(pio_offset);
#else

	pio = pio0;

	pio_offset = pio_add_program(pio, &n64send_program);

	sm = pio_claim_unused_sm(pio, true);

	pio_sm_config c = n64send_program_get_default_config(pio_offset);
#endif

	sm_config_set_in_shift(&c, false, false, 32);
	sm_config_set_out_shift(&c, false, false, 32);

	sm_config_set_in_pins(&c, N64_DIO_PIN);
	sm_config_set_out_pins(&c, N64_DIO_PIN, 1);
	sm_config_set_set_pins(&c, N64_DIO_PIN, 1);

	pio_gpio_init(pio, N64_DIO_PIN);

	pio_sm_set_consecutive_pindirs(pio, sm, N64_DIO_PIN, 1, false);

	sm_config_set_clkdiv(&c, 16.625f);

#ifdef USE_PIO_DMA
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
#endif

	pio_sm_init(pio, sm, pio_offset, &c);

	pio_sm_set_enabled(pio, sm, true);
    }

    multicore_reset_core1();
    multicore_launch_core1(usb_host_process);

    TU_LOG2("Controller enabled.\n");

#ifdef USE_GPIO_IRQ
    gpio_acknowledge_irq(N64_DIO_PIN, GPIO_IRQ_EDGE_FALL);
    gpio_set_irq_enabled(N64_DIO_PIN, GPIO_IRQ_EDGE_FALL, true);

    irq_set_exclusive_handler(IO_IRQ_BANK0, gpio_irq_handler);
    irq_set_enabled(IO_IRQ_BANK0, true);

    printf("GPIO IRQ Enabled\n");
#endif

    main_loop();

//    while (1) {
#if 0
	if (!gpio_get(N64_DIO_PIN)) {
	    console_is_off++;
	    if (console_is_off > 100) {
		if (memory_pak_changed) {
		    memory_pak_to_flash();
		}
		console_is_off = 0;
	    }
	}
	sleep_ms(1);
#endif
//    }

    return 0;
}
