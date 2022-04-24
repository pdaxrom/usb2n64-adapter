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

  TU_VERIFY(itf_desc->bInterfaceClass    == TUSB_CLASS_VENDOR_SPECIFIC &&
            itf_desc->bInterfaceSubClass == 0x47 &&
            itf_desc->bInterfaceProtocol == 0xd0);

  xpadh_data_t * p_xpad = get_itf(dev_addr);

  uint8_t itf_count = p_xpad->itf_count++;

  if (itf_count > 0) {
    return false;
  }

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
//    TU_ASSERT(TUSB_DESC_ENDPOINT == desc_ep->bDescriptorType && TUSB_XFER_BULK == desc_ep->bmAttributes.xfer);

    TU_ASSERT(usbh_edpt_open(rhport, dev_addr, desc_ep));

    if ( tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN )
    {
      p_xpad->ep_in[itf_count] = desc_ep->bEndpointAddress;
    }else
    {
      p_xpad->ep_out[itf_count] = desc_ep->bEndpointAddress;
    }

    drv_len += tu_desc_len(p_desc);
    p_desc = tu_desc_next( p_desc );
  }

  return true;
}

static bool xpadh_start(uint8_t dev_addr)
{
  memset(odata, 0, sizeof(odata));

  odata[0] = 0x05;
  odata[1] = 0x20;
  odata[2] = serial++;
  odata[3] = 0x01;
  odata[4] = 0x00;

  if (tuh_xpad_send(dev_addr, odata, 5, false) == false) {
    TU_LOG2("xpadh_start() tuh_xpad_send error\r\n");
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

bool xpadh_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes)
{
  if (ep_addr != xpadh_data[dev_addr-1].ep_out[0]) {
    if (idata[0] == 0x02 && idata[1] == 0x20) {
      TU_LOG2("Req auth\r\n");
      xpadh_start(dev_addr);
      return true;
    } else {
      TU_LOG2_MEM(idata, xferred_bytes, 2);
    }
  }

  tuh_xpad_read_cb(dev_addr, idata);

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
