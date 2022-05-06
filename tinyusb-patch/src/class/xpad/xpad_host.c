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
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if (TUSB_OPT_HOST_ENABLED && CFG_TUH_XPAD)

#include "host/usbh.h"
#include "host/usbh_classdriver.h"

#include "xpad_host.h"

static uint8_t serial = 0;

static xpad_ctype_t xpad_ctype = XPAD_UNKNOWN;

CFG_TUSB_MEM_SECTION static uint8_t odata[32];
CFG_TUSB_MEM_SECTION static uint8_t idata[32];

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
typedef struct {
  uint8_t itf_count;

  uint8_t itf_num[5];
  uint8_t itf_protocol[5];

  uint8_t ep_in[5];
  uint8_t ep_out[5];
} xpadh_data_t;

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
static xpadh_data_t xpadh_data[CFG_TUH_DEVICE_MAX];

static inline xpadh_data_t* get_itf(uint8_t dev_addr)
{
  return &xpadh_data[dev_addr-1];
}

bool tuh_xpad_mounted(uint8_t dev_addr)
{
  xpadh_data_t* xpad = get_itf(dev_addr);
  return xpad->ep_in[0] && xpad->ep_out[0];
}

//--------------------------------------------------------------------+
// APPLICATION API (parameter validation needed)
//--------------------------------------------------------------------+

bool tuh_xpad_send(uint8_t dev_addr, void const * p_data, uint32_t length, bool is_notify)
{
  (void) is_notify;
  TU_VERIFY( tuh_xpad_mounted(dev_addr) );
  TU_VERIFY( p_data != NULL && length, TUSB_ERROR_INVALID_PARA);

  uint8_t const ep_out = xpadh_data[dev_addr-1].ep_out[0];
  if ( usbh_edpt_busy(dev_addr, ep_out) ) return false;

  return usbh_edpt_xfer(dev_addr, ep_out, (void *) p_data, length);
}

bool tuh_xpad_receive(uint8_t dev_addr, void * p_buffer, uint32_t length, bool is_notify)
{
  (void) is_notify;
  TU_VERIFY( tuh_xpad_mounted(dev_addr) );
  TU_VERIFY( p_buffer != NULL && length, TUSB_ERROR_INVALID_PARA);

  uint8_t const ep_in = xpadh_data[dev_addr-1].ep_in[0];
  if ( usbh_edpt_busy(dev_addr, ep_in) ) return false;

  return usbh_edpt_xfer(dev_addr, ep_in, p_buffer, length);
}

//--------------------------------------------------------------------+
// USBH-CLASS DRIVER API
//--------------------------------------------------------------------+
void xpadh_init(void)
{
  tu_memclr(xpadh_data, sizeof(xpadh_data));
}

bool xpadh_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *itf_desc, uint16_t max_len)
{
  (void) max_len;

  TU_LOG2("class = %02X subclass = %02X protocol = %02X\n", itf_desc->bInterfaceClass, itf_desc->bInterfaceSubClass, itf_desc->bInterfaceProtocol);

  if (itf_desc->bInterfaceClass    == TUSB_CLASS_VENDOR_SPECIFIC &&
            itf_desc->bInterfaceSubClass == 0x47 &&
            itf_desc->bInterfaceProtocol == 0xd0) {
    xpad_ctype = XPAD_XBONE;
  } else if (itf_desc->bInterfaceClass    == TUSB_CLASS_VENDOR_SPECIFIC &&
            itf_desc->bInterfaceSubClass == 0x5d &&
            itf_desc->bInterfaceProtocol == 0x01) {
    xpad_ctype = XPAD_360_WIRED;
  } else {
    return false;
  }

  xpadh_data_t * p_xpad = get_itf(dev_addr);

  uint8_t itf_count = p_xpad->itf_count++;

  if (itf_count > 0) {
    return false;
  }

//  if (xpad_ctype == XPAD_XBONE && itf_count > 0) {
//    return false;
//  }

  p_xpad->itf_num[itf_count]      = itf_desc->bInterfaceNumber;
  p_xpad->itf_protocol[itf_count] = itf_desc->bInterfaceProtocol;
  uint32_t edpt_count             = itf_desc->bNumEndpoints;

  //------------- Communication Interface -------------//
  uint16_t drv_len = tu_desc_len(itf_desc);
  uint8_t const * p_desc = tu_desc_next(itf_desc);

  // data endpoints expected to be in pairs
  for(uint32_t i=0; i<edpt_count; i++)
  {
    tusb_desc_endpoint_t const *desc_ep = (tusb_desc_endpoint_t const *) p_desc;

    if (TUSB_DESC_ENDPOINT == desc_ep->bDescriptorType) {
//    TU_ASSERT(TUSB_DESC_ENDPOINT == desc_ep->bDescriptorType && TUSB_XFER_BULK == desc_ep->bmAttributes.xfer);
      TU_ASSERT(usbh_edpt_open(rhport, dev_addr, desc_ep));

      if ( tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN ) {
        p_xpad->ep_in[itf_count] = desc_ep->bEndpointAddress;
      } else {
        p_xpad->ep_out[itf_count] = desc_ep->bEndpointAddress;
      }
    } else {
      edpt_count += 1;
    }

    drv_len += tu_desc_len(p_desc);
    p_desc = tu_desc_next( p_desc );
  }

  return true;
}

static bool xpadh_start(uint8_t dev_addr)
{
  memset(odata, 0, sizeof(odata));

  if (xpad_ctype == XPAD_360_WIRED) {
    odata[3] = 0x40;

    if (tuh_xpad_send(dev_addr, odata, 12, false) == false) {
      TU_LOG2("xpadh_start() tuh_xpad_send error\r\n");
    }
  } else if (xpad_ctype == XPAD_XBONE) {
    odata[0] = 0x05;
    odata[1] = 0x20;
    odata[2] = serial++;
    odata[3] = 0x01;
    odata[4] = 0x00;

    if (tuh_xpad_send(dev_addr, odata, 5, false) == false) {
      TU_LOG2("xpadh_start() tuh_xpad_send error\r\n");
    }
  }

  return true;
}

static bool xpadh_set_led(uint8_t dev_addr, uint8_t cmd)
{
  memset(odata, 0, sizeof(odata));

  if (xpad_ctype == XPAD_360_WIRED) {
    odata[0] = 0x01;
    odata[1] = 0x03;
    odata[2] = cmd;

    if (tuh_xpad_send(dev_addr, odata, 3, false) == false) {
      TU_LOG2("xpadh_start() tuh_xpad_send error\r\n");
    }
  }

  return true;
}

bool xpadh_set_config(uint8_t dev_addr, uint8_t itf_num)
{
  (void) itf_num;

  sleep_ms(100);

  if (tuh_xpad_receive(dev_addr, idata, 32, false) == false) {
    TU_LOG2("tuh_xpad_receive error");
  }

  tuh_xpad_mount_cb(dev_addr);

  return true;
}

bool tuh_xpad_write(uint8_t dev_addr, uint8_t *report, int size)
{
    memmove(odata, report, size);

    return tuh_xpad_send(dev_addr, odata, size, false);
}

bool tuh_xpad_vibro(uint8_t dev_addr, bool on)
{
    if (on) {
        if (xpad_ctype == XPAD_360_WIRED) {
            uint8_t start_vibro[] = {
                0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            };

            start_vibro[3] = 0x40;
            start_vibro[4] = 0x40;
            return tuh_xpad_write(dev_addr, start_vibro, 8);
        } else if (xpad_ctype == XPAD_XBONE) {
            uint8_t start_vibro[] = {
                0x09, 0x08, 0x00,
                0x09, 0x00, 0x0f,
                0x20, 0x20, 0x20, 0x20,
                0x20, 0x00
            };

            start_vibro[5] = 0x0f;
            return tuh_xpad_write(dev_addr, start_vibro, 12);
        }
    } else {
        if (xpad_ctype == XPAD_360_WIRED) {
            uint8_t start_vibro[] = {
                0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            };

            return tuh_xpad_write(dev_addr, start_vibro, 8);
        } else if (xpad_ctype == XPAD_XBONE) {
            // stop automatically
            return true;
        }
    }

    return false;
}

bool xpadh_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes)
{
  if (ep_addr != xpadh_data[dev_addr-1].ep_out[0]) {
    if (xpad_ctype == XPAD_360_WIRED) {
      TU_LOG2_MEM(idata, xferred_bytes, 2);
      if (idata[0] == 0x01 && idata[1] == 0x03) {
        TU_LOG2("Set leds\n");
        xpadh_set_led(dev_addr, 0x06);
      }
    } else if (xpad_ctype == XPAD_XBONE) {
      if (idata[0] == 0x02 && idata[1] == 0x20) {
        TU_LOG2("Req auth\r\n");
        xpadh_start(dev_addr);
        return true;
      } else {
        TU_LOG2_MEM(idata, xferred_bytes, 2);
      }
    }
  }

  static xpad_controller_t old_info;
  xpad_controller_t info;
  memset(&info, 0, sizeof(xpad_controller_t));

  if (xpad_ctype == XPAD_360_WIRED) {
    if (idata[0] == 0x00 && idata[1] == 0x14) {
	if (idata[2] & 0x01) info.buttons |= XPAD_HAT_UP;
	if (idata[2] & 0x02) info.buttons |= XPAD_HAT_DOWN;
	if (idata[2] & 0x04) info.buttons |= XPAD_HAT_LEFT;
	if (idata[2] & 0x08) info.buttons |= XPAD_HAT_RIGHT;
	if (idata[2] & 0x10) info.buttons |= XPAD_START;
	if (idata[2] & 0x20) info.buttons |= XPAD_BACK;
	if (idata[2] & 0x40) info.buttons |= XPAD_STICK_L;
	if (idata[2] & 0x80) info.buttons |= XPAD_STICK_R;

	if (idata[3] & 0x01) info.buttons |= XPAD_PAD_LB;
	if (idata[3] & 0x02) info.buttons |= XPAD_PAD_RB;
	if (idata[3] & 0x04) info.buttons |= XPAD_XLOGO;
	if (idata[3] & 0x10) info.buttons |= XPAD_PAD_A;
	if (idata[3] & 0x20) info.buttons |= XPAD_PAD_B;
	if (idata[3] & 0x40) info.buttons |= XPAD_PAD_X;
	if (idata[3] & 0x80) info.buttons |= XPAD_PAD_Y;


	info.lx = (idata[7] << 8) | idata[6];
	info.ly = (idata[9] << 8) | idata[8];
	info.rx = (idata[11] << 8) | idata[10];
	info.ry = (idata[13] << 8) | idata[12];
	info.lt = idata[4] << 2;
	info.rt = idata[5] << 2;
    }

    if (memcmp(&info, &old_info, sizeof(xpad_controller_t))) {
	tuh_xpad_read_cb(dev_addr, idata, &info);
	memmove(&old_info, &info, sizeof(xpad_controller_t));
    }
  } else if (xpad_ctype == XPAD_XBONE) {
    if (idata[0] == 0x20) {
	if (idata[5] & 0x01) info.buttons |= XPAD_HAT_UP;
	if (idata[5] & 0x02) info.buttons |= XPAD_HAT_DOWN;
	if (idata[5] & 0x04) info.buttons |= XPAD_HAT_LEFT;
	if (idata[5] & 0x08) info.buttons |= XPAD_HAT_RIGHT;
	if (idata[5] & 0x10) info.buttons |= XPAD_PAD_LB;
	if (idata[5] & 0x20) info.buttons |= XPAD_PAD_RB;
	if (idata[5] & 0x40) info.buttons |= XPAD_STICK_L;
	if (idata[5] & 0x80) info.buttons |= XPAD_STICK_R;

	if (idata[4] & 0x10) info.buttons |= XPAD_PAD_A;
	if (idata[4] & 0x20) info.buttons |= XPAD_PAD_B;
	if (idata[4] & 0x40) info.buttons |= XPAD_PAD_X;
	if (idata[4] & 0x80) info.buttons |= XPAD_PAD_Y;
	if (idata[4] & 0x04) info.buttons |= XPAD_START;
	if (idata[4] & 0x08) info.buttons |= XPAD_BACK;

	info.lx = (idata[11] << 8) | idata[10];
	info.ly = (idata[13] << 8) | idata[12];
	info.rx = (idata[15] << 8) | idata[14];
	info.ry = (idata[17] << 8) | idata[16];
	info.lt = (idata[7] << 8) | idata[6];
	info.rt = (idata[9] << 8) | idata[8];
    } else if (idata[0] == 0x07 && idata[1] == 0x20) {
        memmove(&info, &old_info, sizeof(xpad_controller_t));
	if (idata[4] & 0x01) info.buttons |= XPAD_XLOGO; else info.buttons &= ~XPAD_XLOGO;
    }

    tuh_xpad_read_cb(dev_addr, idata, &info);
    memmove(&old_info, &info, sizeof(xpad_controller_t));
  }

  tuh_xpad_receive(dev_addr, idata, 32, true); // waiting for next data

  return true;
}

void xpadh_close(uint8_t dev_addr)
{
  TU_VERIFY(dev_addr <= CFG_TUH_DEVICE_MAX, );

  xpadh_data_t * p_xpad = get_itf(dev_addr);
  tu_memclr(p_xpad, sizeof(xpadh_data_t));
}

#endif
