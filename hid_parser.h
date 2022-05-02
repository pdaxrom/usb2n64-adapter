#ifndef _HID_PARSER_H_
#define _HID_PARSER_H_

#define MAX_REPORT_ITEMS	32

typedef struct {
    uint16_t page;  /**< Usage page of the report item. */
    uint16_t usage; /**< Usage of the report item. */
} hid_usage_t;

typedef struct {
    uint32_t type;     /**< Unit type (refer to HID specifications for details). */
    uint8_t  exponent; /**< Unit exponent (refer to HID specifications for details). */
} hid_unit_t;

typedef struct {
    uint32_t min; /**< Minimum value for the attribute. */
    uint32_t max; /**< Maximum value for the attribute. */
} hid_minmax_t;

typedef struct {
    hid_usage_t	 usage;
    hid_unit_t	 unit;
    hid_minmax_t logical;
    hid_minmax_t physical;
} hid_report_item_attributes_t;

typedef struct {
    uint16_t                    bit_offset;
    uint8_t                     bit_size;
    uint8_t                     item_type;
    uint16_t                    item_flags;
//    hid_collection_path_t*      collection_path;
    hid_report_item_attributes_t attributes;
} hid_report_item_t;

typedef struct {
    uint8_t  report_id;
    uint8_t  usage;
    uint16_t usage_page;

    uint8_t   num_items;
    hid_report_item_t	item[MAX_REPORT_ITEMS];

  // TODO still use the endpoint size for now
//  uint8_t in_len;      // length of IN report
//  uint8_t out_len;     // length of OUT report
} hid_report_info_t;

uint8_t hid_parse_report_descriptor(hid_report_info_t* report_info_arr, uint8_t arr_count, uint8_t const* desc_report, uint16_t desc_len);

#endif
