/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021, Ha Thach (tinyusb.org)
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

#include "bsp/board.h"
#include "tusb.h"

#include "hid_parser.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

// If your host terminal support ansi escape code such as TeraTerm
// it can be use to simulate mouse cursor movement within terminal
#define USE_ANSI_ESCAPE   0

#define MAX_REPORT  4

static uint8_t const keycode2ascii[128][2] =  { HID_KEYCODE_TO_ASCII };

// Each HID instance can has multiple reports
static struct
{
  uint8_t report_count;
  hid_report_info_t report_info[MAX_REPORT];
} hid_info[CFG_TUH_HID];

typedef struct {
    const hid_report_item_t *lx;
    const hid_report_item_t *ly;
    const hid_report_item_t *rx;
    const hid_report_item_t *ry;
    const hid_report_item_t *hat;
    const hid_report_item_t *a;
    const hid_report_item_t *b;
    const hid_report_item_t *x;
    const hid_report_item_t *y;
    const hid_report_item_t *lb;
    const hid_report_item_t *rb;
    const hid_report_item_t *lt;
    const hid_report_item_t *rt;
    const hid_report_item_t *start;
    const hid_report_item_t *select;
    const hid_report_item_t *sl;
    const hid_report_item_t *sr;
} gamepad_items_t;

typedef struct {
    const hid_report_item_t *x;
    const hid_report_item_t *y;
    const hid_report_item_t *wheel;
    const hid_report_item_t *acpan;
    const hid_report_item_t *lb;
    const hid_report_item_t *mb;
    const hid_report_item_t *rb;
    const hid_report_item_t *bw;
    const hid_report_item_t *fw;
} mouse_items_t;

static gamepad_items_t gamepad_items;
static bool gamepad_inited;

static mouse_items_t mouse_items;
static bool mouse_inited;

extern void enable_mouse(void);
extern void update_mouse(uint8_t buttons, int8_t x, int8_t y, int8_t wheel, int8_t acpan);

static void process_kbd_report(hid_keyboard_report_t const *report);
static void process_mouse_boot_report(hid_mouse_report_t const * report);
static void process_generic_report(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len);

extern void enable_hid_gamepad(void);

void hid_app_task(void)
{
  // nothing to do
}

//--------------------------------------------------------------------+
// TinyUSB Callbacks
//--------------------------------------------------------------------+

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use. tuh_hid_parse_report_descriptor()
// can be used to parse common/simple enough descriptor.
// Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE, it will be skipped
// therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len)
{
  printf("HID device address = %d, instance = %d is mounted\r\n", dev_addr, instance);

  // Interface protocol (hid_interface_protocol_enum_t)
  const char* protocol_str[] = { "None", "Keyboard", "Mouse" };
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
  uint8_t const protocol_mode = tuh_hid_get_protocol(dev_addr, instance);

  printf("HID Interface Protocol = %s\r\n", protocol_str[itf_protocol]);
  printf("HID Interface Mode     = %s\r\n", protocol_mode ? "Report" : "Boot");

  if (protocol_mode == HID_PROTOCOL_BOOT && itf_protocol == HID_ITF_PROTOCOL_KEYBOARD) {
  } else if (protocol_mode == HID_PROTOCOL_BOOT && itf_protocol == HID_ITF_PROTOCOL_MOUSE) {
    enable_mouse();
  } else {
    hid_info[instance].report_count = hid_parse_report_descriptor(hid_info[instance].report_info, MAX_REPORT, desc_report, desc_len);
    printf("HID has %u reports \r\n", hid_info[instance].report_count);
    gamepad_inited = false;
    mouse_inited = false;
  }

  // request to receive report
  // tuh_hid_report_received_cb() will be invoked when report is available
  if ( !tuh_hid_receive_report(dev_addr, instance) )
  {
    printf("Error: cannot request to receive report\r\n");
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  uint8_t const protocol_mode = tuh_hid_get_protocol(dev_addr, instance);

  if (protocol_mode == HID_PROTOCOL_BOOT && itf_protocol == HID_ITF_PROTOCOL_KEYBOARD) {
      TU_LOG2("HID receive boot keyboard report\r\n");
      process_kbd_report( (hid_keyboard_report_t const*) report );
  } else if (protocol_mode == HID_PROTOCOL_BOOT && itf_protocol == HID_ITF_PROTOCOL_MOUSE) {
      TU_LOG2("HID receive boot mouse report\r\n");
      process_mouse_boot_report( (hid_mouse_report_t const*) report );
  } else {
      // Generic report requires matching ReportID and contents with previous parsed report info
      process_generic_report(dev_addr, instance, report, len);
  }

  // continue to request to receive report
  if ( !tuh_hid_receive_report(dev_addr, instance) )
  {
    printf("Error: cannot request to receive report\r\n");
  }
}

//--------------------------------------------------------------------+
// Keyboard
//--------------------------------------------------------------------+

// look up new key in previous keys
static inline bool find_key_in_report(hid_keyboard_report_t const *report, uint8_t keycode)
{
  for(uint8_t i=0; i<6; i++)
  {
    if (report->keycode[i] == keycode)  return true;
  }

  return false;
}

static void process_kbd_report(hid_keyboard_report_t const *report)
{
  static hid_keyboard_report_t prev_report = { 0, 0, {0} }; // previous report to check key released

  //------------- example code ignore control (non-printable) key affects -------------//
  for(uint8_t i=0; i<6; i++)
  {
    if ( report->keycode[i] )
    {
      if ( find_key_in_report(&prev_report, report->keycode[i]) )
      {
        // exist in previous report means the current key is holding
      }else
      {
        // not existed in previous report means the current key is pressed
        bool const is_shift = report->modifier & (KEYBOARD_MODIFIER_LEFTSHIFT | KEYBOARD_MODIFIER_RIGHTSHIFT);
        uint8_t ch = keycode2ascii[report->keycode[i]][is_shift ? 1 : 0];
        putchar(ch);
        if ( ch == '\r' ) putchar('\n'); // added new line for enter key

        fflush(stdout); // flush right away, else nanolib will wait for newline
      }
    }
    // TODO example skips key released
  }

  prev_report = *report;
}

//--------------------------------------------------------------------+
// Mouse
//--------------------------------------------------------------------+

static void process_mouse_boot_report(hid_mouse_report_t const * report)
{
  TU_LOG2_MEM((uint8_t *)report, sizeof(hid_mouse_report_t), 2);

  update_mouse(report->buttons, report->x, report->y, report->wheel, 0);
}

//--------------------------------------------------------------------+
// Generic Report
//--------------------------------------------------------------------+

static int32_t to_signed_value(const hid_report_item_t *item, const uint8_t *report, uint16_t len)
{
    int32_t value = 0;

    if (hid_parse_get_item_value(item, report, len, &value)) {
	int32_t midval = ((item->attributes.logical.max - item->attributes.logical.min) >> 1) + 1;
	value -= midval;
	value <<= (16 - item->bit_size);
    }

    if (value >  32767) value =  32767;
    if (value < -32767) value = -32767;

    return value;
}

static bool to_bit_value(const hid_report_item_t *item, const uint8_t *report, uint16_t len)
{
    int32_t value = 0;

    hid_parse_get_item_value(item, report, len, &value);

    return value ? true : false;
}

static int8_t to_signed_value8(const hid_report_item_t *item, const uint8_t *report, uint16_t len)
{
    int32_t value = 0;

    if (hid_parse_get_item_value(item, report, len, &value)) {
        value = (value > 127) ? 127 : (value < -127) ? -127 : value;
    }

    return value;
}

static void mouse_setup(hid_report_info_t *info)
{
    mouse_items_t *items = &mouse_items;
    memset(items, 0, sizeof(mouse_items_t));

    if (!hid_parse_find_item_by_usage(info, RI_MAIN_INPUT, HID_USAGE_DESKTOP_X, &items->x)) {
        printf("No X\n");
    }

    if (!hid_parse_find_item_by_usage(info, RI_MAIN_INPUT, HID_USAGE_DESKTOP_Y, &items->y)) {
        printf("No Y\n");
    }

    if (!hid_parse_find_item_by_usage(info, RI_MAIN_INPUT, HID_USAGE_DESKTOP_WHEEL, &items->wheel)) {
        printf("No wheel\n");
    }

    if (!hid_parse_find_item_by_usage(info, RI_MAIN_INPUT, HID_USAGE_CONSUMER_AC_PAN, &items->acpan)) {
        printf("No AC PAN\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 0, &items->lb)) {
        printf("No LB\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 1, &items->rb)) {
        printf("No RB\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 2, &items->mb)) {
        printf("No MB\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 3, &items->bw)) {
        printf("No BW\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 4, &items->fw)) {
        printf("No FW\n");
    }
}

static void process_mouse_report(hid_report_info_t *rpt_info, uint8_t const* report, uint16_t len)
{
    mouse_items_t *items = &mouse_items;
    int32_t value;
    uint8_t butts = 0;
    int8_t x, y, wheel, acpan;

    if (!mouse_inited) {
        mouse_setup(rpt_info);
        mouse_inited = true;

        enable_mouse();
    }

//    debug_dump_16(report);

    if (to_bit_value(items->lb, report, len)) butts |= 0x01;
    if (to_bit_value(items->rb, report, len)) butts |= 0x02;
    if (to_bit_value(items->mb, report, len)) butts |= 0x04;
    if (to_bit_value(items->bw, report, len)) butts |= 0x08;
    if (to_bit_value(items->fw, report, len)) butts |= 0x10;

    x = to_signed_value8(items->x, report, len);
    y = to_signed_value8(items->y, report, len);
    wheel = to_signed_value8(items->wheel, report, len);
    acpan = to_signed_value8(items->acpan, report, len);

    update_mouse(butts, x, y, wheel, acpan);
}

static void gamepad_setup(hid_report_info_t *info)
{
    gamepad_items_t *items = &gamepad_items;
    memset(items, 0, sizeof(gamepad_items_t));

    if (!hid_parse_find_item_by_usage(info, RI_MAIN_INPUT, HID_USAGE_DESKTOP_X, &items->lx)) {
        printf("No LX\n");
    }

    if (!hid_parse_find_item_by_usage(info, RI_MAIN_INPUT, HID_USAGE_DESKTOP_Y, &items->ly)) {
        printf("No LY\n");
    }

    if (!hid_parse_find_item_by_usage(info, RI_MAIN_INPUT, HID_USAGE_DESKTOP_RZ, &items->rx)) {
        printf("No RX\n");
    }

    if (!hid_parse_find_item_by_usage(info, RI_MAIN_INPUT, HID_USAGE_DESKTOP_Z, &items->ry)) {
        printf("No RY\n");
    }

    if (!hid_parse_find_item_by_usage(info, RI_MAIN_INPUT, HID_USAGE_DESKTOP_HAT_SWITCH, &items->hat)) {
        printf("No HAT\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 0, &items->a)) {
        printf("No A\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 1, &items->b)) {
        printf("No B\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 2, &items->x)) {
        printf("No X\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 3, &items->y)) {
        printf("No Y\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 4, &items->lb)) {
        printf("No LB\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 5, &items->rb)) {
        printf("No RB\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 6, &items->lt)) {
        printf("No LT\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 7, &items->rt)) {
        printf("No RT\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 9, &items->start)) {
        printf("No START\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 8, &items->select)) {
        printf("No SELECT\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 10, &items->sl)) {
        printf("No SL\n");
    }

    if (!hid_parse_find_bit_item_by_page(info, RI_MAIN_INPUT, HID_USAGE_PAGE_BUTTON, 11, &items->sr)) {
        printf("No SR\n");
    }
}

static void process_gamepad_report(hid_report_info_t *rpt_info, uint8_t const* report, uint16_t len)
{
    static xpad_controller_t old_info;
    xpad_controller_t info;
    gamepad_items_t *items = &gamepad_items;
    int32_t value;

    if (!gamepad_inited) {
        gamepad_setup(rpt_info);
        gamepad_inited = true;

        enable_hid_gamepad();
    }

//    debug_dump_16(report);

    memset(&info, 0, sizeof(xpad_controller_t));

    if (to_bit_value(items->a, report, len))  info.buttons |= XPAD_PAD_A;
    if (to_bit_value(items->b, report, len))  info.buttons |= XPAD_PAD_B;
    if (to_bit_value(items->x, report, len))  info.buttons |= XPAD_PAD_X;
    if (to_bit_value(items->y, report, len))  info.buttons |= XPAD_PAD_Y;
    if (to_bit_value(items->lb, report, len)) info.buttons |= XPAD_PAD_LB;
    if (to_bit_value(items->rb, report, len)) info.buttons |= XPAD_PAD_RB;
    if (to_bit_value(items->start, report, len)) info.buttons |= XPAD_START;
    if (to_bit_value(items->select, report, len)) info.buttons |= XPAD_XLOGO;
    if (to_bit_value(items->sl, report, len)) info.buttons |= XPAD_STICK_L;
    if (to_bit_value(items->sr, report, len)) info.buttons |= XPAD_STICK_R;

    if (to_bit_value(items->lt, report, len)) info.lt = 1027; else info.lt = 0;
    if (to_bit_value(items->rt, report, len)) info.rt = 1027; else info.rt = 0;

    if (hid_parse_get_item_value(items->hat, report, len, &value)) {
//        printf("HAT = %d\n", value);
        switch(value) {
        case 0: info.buttons |= XPAD_HAT_UP; break;
        case 1: info.buttons |= XPAD_HAT_UP | XPAD_HAT_RIGHT; break;
        case 2: info.buttons |= XPAD_HAT_RIGHT; break;
        case 3: info.buttons |= XPAD_HAT_RIGHT | XPAD_HAT_DOWN; break;
        case 4: info.buttons |= XPAD_HAT_DOWN; break;
        case 5: info.buttons |= XPAD_HAT_DOWN | XPAD_HAT_LEFT; break;
        case 6: info.buttons |= XPAD_HAT_LEFT; break;
        case 7: info.buttons |= XPAD_HAT_LEFT | XPAD_HAT_UP; break;
        }
    }

    info.lx =  to_signed_value(items->lx, report, len);
    info.ly = -to_signed_value(items->ly, report, len);
    info.rx =  to_signed_value(items->rx, report, len);
    info.ry = -to_signed_value(items->ry, report, len);

    if (memcmp(&info, &old_info, sizeof(xpad_controller_t))) {
        tuh_xpad_read_cb(-1, (uint8_t *) report, &info);
        memcpy(&old_info, &info, sizeof(xpad_controller_t));
    }
}

static void process_generic_report(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
  (void) dev_addr;

  uint8_t const rpt_count = hid_info[instance].report_count;
  hid_report_info_t* rpt_info_arr = hid_info[instance].report_info;
  hid_report_info_t* rpt_info = NULL;

  if ( rpt_count == 1 && rpt_info_arr[0].report_id == 0)
  {
    // Simple report without report ID as 1st byte
    rpt_info = &rpt_info_arr[0];
  }else
  {
    // Composite report, 1st byte is report ID, data starts from 2nd byte
    uint8_t const rpt_id = report[0];

    // Find report id in the arrray
    for(uint8_t i=0; i<rpt_count; i++)
    {
      if (rpt_id == rpt_info_arr[i].report_id )
      {
        rpt_info = &rpt_info_arr[i];
        break;
      }
    }

    report++;
    len--;
  }

  if (!rpt_info)
  {
    printf("Couldn't find the report info for this report !\r\n");
    return;
  }

  // For complete list of Usage Page & Usage checkout src/class/hid/hid.h. For examples:
  // - Keyboard                     : Desktop, Keyboard
  // - Mouse                        : Desktop, Mouse
  // - Gamepad                      : Desktop, Gamepad
  // - Consumer Control (Media Key) : Consumer, Consumer Control
  // - System Control (Power key)   : Desktop, System Control
  // - Generic (vendor)             : 0xFFxx, xx

//printf(">>>>>>>> %X %X\n", rpt_info->usage_page, rpt_info->usage);

  if ( rpt_info->usage_page == HID_USAGE_PAGE_DESKTOP )
  {
    switch (rpt_info->usage)
    {
      case HID_USAGE_DESKTOP_KEYBOARD:
        TU_LOG1("HID receive keyboard report\r\n");
        // Assume keyboard follow boot report layout
        process_kbd_report( (hid_keyboard_report_t const*) report );
      break;

      case HID_USAGE_DESKTOP_MOUSE:
        TU_LOG1("HID receive mouse report\r\n");
        // Assume mouse follow boot report layout
        process_mouse_report(rpt_info, report, len);
      break;

      case HID_USAGE_DESKTOP_JOYSTICK:
        TU_LOG1("HID receive joystick report\r\n");
        TU_LOG2_MEM((uint8_t *)report, 8, 2);
        process_gamepad_report(rpt_info, report, len);
      break;

      case HID_USAGE_DESKTOP_GAMEPAD:
        TU_LOG1("HID receive gamepad report\r\n");
        TU_LOG2_MEM((uint8_t *)report, 8, 2);
        process_gamepad_report(rpt_info, report, len);
      break;

      default: break;
    }
  }
}
