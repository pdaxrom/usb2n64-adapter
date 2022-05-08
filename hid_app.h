#ifndef _HID_APP_H_
#define _HID_APP_H_

/*--------------------------------------------------------------------
 * KEYCODE to Ascii Conversion
 *  Expand to array of [128][2] (ascii without shift, ascii with shift)
 *
 * Usage: example to convert ascii from keycode (key) and shift modifier (shift).
 * Here we assume key < 128 ( printable )
 *
 *  uint8_t const conv_table[128][2] =  { HID_KEYCODE_TO_ASCII };
 *  char ch = shift ? conv_table[chr][1] : conv_table[chr][0];
 *
 *--------------------------------------------------------------------*/
#define HID_KEYCODE_TO_RANDNET    \
    0      , /* 0x00 */ \
    0      , /* 0x01 */ \
    0      , /* 0x02 */ \
    0      , /* 0x03 */ \
    0x0D07 , /* 0x04 */ \
    0x0708 , /* 0x05 */ \
    0x0508 , /* 0x06 */ \
    0x0507 , /* 0x07 */ \
    0x0601 , /* 0x08 */ \
    0x0607 , /* 0x09 */ \
    0x0707 , /* 0x0a */ \
    0x0807 , /* 0x0b */ \
    0x0804 , /* 0x0c */ \
    0x0907 , /* 0x0d */ \
    0x0903 , /* 0x0e */ \
    0x0803 , /* 0x0f */ \
    0x0908 , /* 0x10 */ \
    0x0808 , /* 0x11 */ \
    0x0704 , /* 0x12 */ \
    0x0604 , /* 0x13 */ \
    0x0C01 , /* 0x14 */ \
    0x0701 , /* 0x15 */ \
    0x0C07 , /* 0x16 */ \
    0x0801 , /* 0x17 */ \
    0x0904 , /* 0x18 */ \
    0x0608 , /* 0x19 */ \
    0x0501 , /* 0x1a */ \
    0x0C08 , /* 0x1b */ \
    0x0901 , /* 0x1c */ \
    0x0D08 , /* 0x1d */ \
    0x0C05 , /* 0x1e */ \
    0x0505 , /* 0x1f */ \
    0x0605 , /* 0x20 */ \
    0x0705 , /* 0x21 */ \
    0x0805 , /* 0x22 */ \
    0x0905 , /* 0x23 */ \
    0x0906 , /* 0x24 */ \
    0x0806 , /* 0x25 */ \
    0x0706 , /* 0x26 */ \
    0x0606 , /* 0x27 */ \
    0x0D04 , /* 0x28 */ \
    0x0A08 , /* 0x29 */ \
    0x0D06 , /* 0x2a */ \
    0x0D01 , /* 0x2b */ \
    0x0602 , /* 0x2c */ \
    0x0506 , /* 0x2d */ \
    0x0C06 , /* 0x2e */ \
    0x0C04 , /* 0x2f */ \
    0x0406 , /* 0x30 */ \
    0x1105 , /* 0x31 */ \
    0      , /* 0x32 */ \
    0x0703 , /* 0x33 */ \
    0x0603 , /* 0x34 */ \
    0x0D05 , /* 0x35 */ \
    0x0902 , /* 0x36 */ \
    0x0802 , /* 0x37 */ \
    0x0702 , /* 0x38 */ \
    0x0F05 , /* 0x39 */ \
    0x0B01 , /* 0x3a */ \
    0x0A01 , /* 0x3b */ \
    0x0B08 , /* 0x3c */ \
    0x0A07 , /* 0x3d */ \
    0x0B07 , /* 0x3e */ \
    0x0A02 , /* 0x3f */ \
    0x0B02 , /* 0x40 */ \
    0x0A03 , /* 0x41 */ \
    0x0B03 , /* 0x42 */ \
    0x0A04 , /* 0x43 */ \
    0x0203 , /* 0x44 */ \
    0x0B06 , /* 0x45 */ \
    0      , /* 0x46 */ \
    0      , /* 0x47 */ \
    0      , /* 0x48 */ \
    0      , /* 0x49 */ \
    0xFFFF , /* 0x4a */ \
    0      , /* 0x4b */ \
    0      , /* 0x4c */ \
    0x0206 , /* 0x4d */ \
    0      , /* 0x4e */ \
    0x0405 , /* 0x4f */ \
    0x0205 , /* 0x50 */ \
    0x0305 , /* 0x51 */ \
    0x0204 , /* 0x52 */ \
    0      , /* 0x53 */ \
    0      , /* 0x54 */ \
    0      , /* 0x55 */ \
    0      , /* 0x56 */ \
    0      , /* 0x57 */ \
    0      , /* 0x58 */ \
    0      , /* 0x59 */ \
    0      , /* 0x5a */ \
    0      , /* 0x5b */ \
    0      , /* 0x5c */ \
    0      , /* 0x5d */ \
    0      , /* 0x5e */ \
    0      , /* 0x5f */ \
    0      , /* 0x60 */ \
    0      , /* 0x61 */ \
    0      , /* 0x62 */ \
    0      , /* 0x63 */ \
    0      , /* 0x64 */ \
    0      , /* 0x65 */ \
    0      , /* 0x66 */ \
    0      , /* 0x67 */ \
    0      , /* 0x68 */ \
    0      , /* 0x69 */ \
    0      , /* 0x6a */ \
    0      , /* 0x6b */ \
    0      , /* 0x6c */ \
    0      , /* 0x6d */ \
    0      , /* 0x6e */ \
    0      , /* 0x6f */ \
    0      , /* 0x70 */ \
    0      , /* 0x71 */ \
    0      , /* 0x72 */ \
    0      , /* 0x73 */ \
    0      , /* 0x74 */ \
    0      , /* 0x75 */ \
    0      , /* 0x76 */ \
    0      , /* 0x77 */ \
    0      , /* 0x78 */ \
    0      , /* 0x79 */ \
    0      , /* 0x7a */ \
    0      , /* 0x7b */ \
    0      , /* 0x7c */ \
    0      , /* 0x7d */ \
    0      , /* 0x7e */ \
    0      , /* 0x7f */ \
    0      , /* 0x80 */ \
    0      , /* 0x81 */ \
    0      , /* 0x82 */ \
    0      , /* 0x83 */ \
    0      , /* 0x84 */ \
    0      , /* 0x85 */ \
    0      , /* 0x86 */ \
    0      , /* 0x87 */ \
    0      , /* 0x88 */ \
    0      , /* 0x89 */ \
    0      , /* 0x8a */ \
    0      , /* 0x8b */ \
    0      , /* 0x8c */ \
    0      , /* 0x8d */ \
    0      , /* 0x8e */ \
    0      , /* 0x8f */ \
    0      , /* 0x90 */ \
    0      , /* 0x91 */ \
    0      , /* 0x92 */ \
    0      , /* 0x93 */ \
    0      , /* 0x94 */ \
    0      , /* 0x95 */ \
    0      , /* 0x96 */ \
    0      , /* 0x97 */ \
    0      , /* 0x98 */ \
    0      , /* 0x99 */ \
    0      , /* 0x9a */ \
    0      , /* 0x9b */ \
    0      , /* 0x9c */ \
    0      , /* 0x9d */ \
    0      , /* 0x9e */ \
    0      , /* 0x9f */ \
    0      , /* 0xa0 */ \
    0      , /* 0xa1 */ \
    0      , /* 0xa2 */ \
    0      , /* 0xa3 */ \
    0      , /* 0xa4 */ \
    0      , /* 0xa5 */ \
    0      , /* 0xa6 */ \
    0      , /* 0xa7 */ \
    0      , /* 0xa8 */ \
    0      , /* 0xa9 */ \
    0      , /* 0xaa */ \
    0      , /* 0xab */ \
    0      , /* 0xac */ \
    0      , /* 0xad */ \
    0      , /* 0xae */ \
    0      , /* 0xaf */ \
    0      , /* 0xb0 */ \
    0      , /* 0xb1 */ \
    0      , /* 0xb2 */ \
    0      , /* 0xb3 */ \
    0      , /* 0xb4 */ \
    0      , /* 0xb5 */ \
    0      , /* 0xb6 */ \
    0      , /* 0xb7 */ \
    0      , /* 0xb8 */ \
    0      , /* 0xb9 */ \
    0      , /* 0xba */ \
    0      , /* 0xbb */ \
    0      , /* 0xbc */ \
    0      , /* 0xbd */ \
    0      , /* 0xbe */ \
    0      , /* 0xbf */ \
    0      , /* 0xc0 */ \
    0      , /* 0xc1 */ \
    0      , /* 0xc2 */ \
    0      , /* 0xc3 */ \
    0      , /* 0xc4 */ \
    0      , /* 0xc5 */ \
    0      , /* 0xc6 */ \
    0      , /* 0xc7 */ \
    0      , /* 0xc8 */ \
    0      , /* 0xc9 */ \
    0      , /* 0xca */ \
    0      , /* 0xcb */ \
    0      , /* 0xcc */ \
    0      , /* 0xcd */ \
    0      , /* 0xce */ \
    0      , /* 0xcf */ \
    0      , /* 0xd0 */ \
    0      , /* 0xd1 */ \
    0      , /* 0xd2 */ \
    0      , /* 0xd3 */ \
    0      , /* 0xd4 */ \
    0      , /* 0xd5 */ \
    0      , /* 0xd6 */ \
    0      , /* 0xd7 */ \
    0      , /* 0xd8 */ \
    0      , /* 0xd9 */ \
    0      , /* 0xda */ \
    0      , /* 0xdb */ \
    0      , /* 0xdc */ \
    0      , /* 0xdd */ \
    0      , /* 0xde */ \
    0      , /* 0xdf */ \
    0x1107 , /* 0xe0 */ \
    0x0e01 , /* 0xe1 */ \
    0x1008 , /* 0xe2 */ \
    0x0F07 , /* 0xe3 */ \
    0      , /* 0xe4 */ \
    0      , /* 0xe5 */ \
    0      , /* 0xe6 */ \
    0      , /* 0xe7 */ \
    0      , /* 0xe8 */ \
    0      , /* 0xe9 */ \
    0      , /* 0xea */ \
    0      , /* 0xeb */ \
    0      , /* 0xec */ \
    0      , /* 0xed */ \
    0      , /* 0xee */ \
    0      , /* 0xef */ \
    0      , /* 0xf0 */ \
    0      , /* 0xf1 */ \
    0      , /* 0xf2 */ \
    0      , /* 0xf3 */ \
    0      , /* 0xf4 */ \
    0      , /* 0xf5 */ \
    0      , /* 0xf6 */ \
    0      , /* 0xf7 */ \
    0      , /* 0xf8 */ \
    0      , /* 0xf9 */ \
    0      , /* 0xfa */ \
    0      , /* 0xfb */ \
    0      , /* 0xfc */ \
    0      , /* 0xfd */ \
    0      , /* 0xfe */ \
    0      , /* 0xff */ \

#endif