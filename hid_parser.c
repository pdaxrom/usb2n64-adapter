#include "bsp/board.h"
#include "tusb.h"

#include "hid_parser.h"

//--------------------------------------------------------------------+
// Report Descriptor Parser
//--------------------------------------------------------------------+

uint8_t hid_parse_report_descriptor(hid_report_info_t* report_info_arr, uint8_t arr_count, uint8_t const* desc_report, uint16_t desc_len)
{
  // Report Item 6.2.2.2 USB HID 1.11
  union TU_ATTR_PACKED
  {
    uint8_t byte;
    struct TU_ATTR_PACKED
    {
        uint8_t size : 2;
        uint8_t type : 2;
        uint8_t tag  : 4;
    };
  } header;

  tu_memclr(report_info_arr, arr_count*sizeof(tuh_hid_report_info_t));

  uint8_t report_num = 0;
  hid_report_info_t* info = report_info_arr;

  // current parsed report count & size from descriptor
  uint16_t ri_global_usage_page = 0;
  uint16_t ri_global_logical_min = 0;
  uint16_t ri_global_logical_max = 0;
  uint16_t ri_global_physical_min = 0;
  uint16_t ri_global_physical_max = 0;
  uint8_t ri_report_count = 0;
  uint8_t ri_report_size = 0;
  uint8_t ri_report_usage_count = 0;

  uint8_t ri_collection_depth = 0;

  while(desc_len && report_num < arr_count)
  {
    header.byte = *desc_report++;
    desc_len--;

    uint8_t const tag  = header.tag;
    uint8_t const type = header.type;
    uint8_t const size = header.size;

    uint32_t data;
    switch (size) {
    case 1: data = desc_report[0]; break;
    case 2: data = (desc_report[1] << 8) | desc_report[0]; break;
    case 3: data = (desc_report[3] << 24) | (desc_report[2] << 16) | (desc_report[1] << 8) | desc_report[0]; break;
    default: data = 0;
    }

    TU_LOG2("tag = %d, type = %d, size = %d, data = ", tag, type, size);
    for(uint32_t i = 0; i < size; i++) TU_LOG(3, "%02X ", desc_report[i]);
    TU_LOG2("\r\n");

    switch(type)
    {
      case RI_TYPE_MAIN:
        switch (tag)
        {
          case RI_MAIN_INPUT:
          case RI_MAIN_OUTPUT:
          case RI_MAIN_FEATURE:
            TU_LOG2("INPUT %d\n", data);
            uint16_t offset = (info->num_items == 0) ? 0 : (info->item[info->num_items - 1].bit_offset + info->item[info->num_items - 1].bit_size);
            for (int i = 0; i < ri_report_count; i++) {
                if (info->num_items + i < MAX_REPORT_ITEMS) {
                    info->item[info->num_items + i].bit_offset = offset;
                    info->item[info->num_items + i].bit_size = ri_report_size;
                    info->item[info->num_items + i].item_type = tag;
                    info->item[info->num_items + i].attributes.logical.min = ri_global_logical_min;
                    info->item[info->num_items + i].attributes.logical.max = ri_global_logical_max;
                    info->item[info->num_items + i].attributes.physical.min = ri_global_physical_min;
                    info->item[info->num_items + i].attributes.physical.max = ri_global_physical_max;
                    info->item[info->num_items + i].attributes.usage.page = ri_global_usage_page;
                    if (ri_report_usage_count != ri_report_count && ri_report_usage_count > 0) {
                        if (i >= ri_report_usage_count) {
                            info->item[info->num_items + i].attributes.usage = info->item[info->num_items + i - 1].attributes.usage;
                        }
                    }
                } else {
                    printf("%s: too much items!\n", __func__);
                }
                offset += ri_report_size;
            }
            info->num_items += ri_report_count;
            ri_report_usage_count = 0;
          break;

          case RI_MAIN_COLLECTION:
            ri_collection_depth++;
          break;

          case RI_MAIN_COLLECTION_END:
            ri_collection_depth--;
            if (ri_collection_depth == 0)
            {
              info++;
              report_num++;
            }
          break;

          default: break;
        }
      break;

      case RI_TYPE_GLOBAL:
        switch(tag)
        {
          case RI_GLOBAL_USAGE_PAGE:
            // only take in account the "usage page" before REPORT ID
            if ( ri_collection_depth == 0 ) {
                info->usage_page = data;
            }
            ri_global_usage_page = data;
          break;

          case RI_GLOBAL_LOGICAL_MIN   :
            ri_global_logical_min = data;
          break;
          case RI_GLOBAL_LOGICAL_MAX   :
            ri_global_logical_max = data;
          break;
          case RI_GLOBAL_PHYSICAL_MIN  :
            ri_global_physical_min = data;
          break;
          case RI_GLOBAL_PHYSICAL_MAX  :
            ri_global_physical_max = data;
          break;

          case RI_GLOBAL_REPORT_ID:
            info->report_id = data;
          break;

          case RI_GLOBAL_REPORT_SIZE:
            ri_report_size = data;
          break;

          case RI_GLOBAL_REPORT_COUNT:
            ri_report_count = data;
          break;

          case RI_GLOBAL_UNIT_EXPONENT : break;
          case RI_GLOBAL_UNIT          : break;
          case RI_GLOBAL_PUSH          : break;
          case RI_GLOBAL_POP           : break;

          default: break;
        }
      break;

      case RI_TYPE_LOCAL:
        switch(tag)
        {
          case RI_LOCAL_USAGE:
            // only take in account the "usage" before starting REPORT ID
            if ( ri_collection_depth == 0 ) {
                info->usage = data;
            } else {
                TU_LOG2("USAGE %02X\n", data);
                if (ri_report_usage_count < MAX_REPORT_ITEMS) {
                    info->item[info->num_items + ri_report_usage_count].attributes.usage.usage = data;
                    ri_report_usage_count++;
                } else {
                    printf("%s: too much report items!\n", __func__);
                }
            }
          break;

          case RI_LOCAL_USAGE_MIN        : break;
          case RI_LOCAL_USAGE_MAX        : break;
          case RI_LOCAL_DESIGNATOR_INDEX : break;
          case RI_LOCAL_DESIGNATOR_MIN   : break;
          case RI_LOCAL_DESIGNATOR_MAX   : break;
          case RI_LOCAL_STRING_INDEX     : break;
          case RI_LOCAL_STRING_MIN       : break;
          case RI_LOCAL_STRING_MAX       : break;
          case RI_LOCAL_DELIMITER        : break;
          default: break;
        }
      break;

      // error
      default:
        TU_LOG2("%s: Unknown type %02X\n", __func__, type);
      break;
    }

    desc_report += size;
    desc_len    -= size;
  }

  for ( uint8_t i = 0; i < report_num; i++ )
  {
    info = report_info_arr+i;
    TU_LOG2("%u: id = %u, usage_page = %u, usage = %u\r\n", i, info->report_id, info->usage_page, info->usage);
  }

  //////
  for (int i = 0; i < info->num_items; i++) {
    TU_LOG2("type %02X\n", info->item[i].item_type);
    TU_LOG2("  offset %d\n", info->item[i].bit_offset);
    TU_LOG2("  size   %d\n", info->item[i].bit_size);
    TU_LOG2("  page   %04X\n", info->item[i].attributes.usage.page);
    TU_LOG2("  usage  %04X\n", info->item[i].attributes.usage.usage);
    TU_LOG2("  logical min %d\n", info->item[i].attributes.logical.min);
    TU_LOG2("  logical max %d\n", info->item[i].attributes.logical.max);
    TU_LOG2("  physical min %d\n", info->item[i].attributes.physical.min);
    TU_LOG2("  physical max %d\n", info->item[i].attributes.physical.max);
  }
  //////

  return report_num;
}

bool hid_parse_find_item_by_page(hid_report_info_t* report_info_arr, uint8_t type, uint16_t page, const hid_report_item_t **item)
{
    for (int i = 0; i < report_info_arr->num_items; i++) {
        if (report_info_arr->item[i].item_type == type &&
            report_info_arr->item[i].attributes.usage.page == page) {
            if (item) {
                *item = &report_info_arr->item[i];
            }
            return true;
        }
    }

    return false;
}

bool hid_parse_find_item_by_usage(hid_report_info_t* report_info_arr, uint8_t type, uint16_t usage, const hid_report_item_t **item)
{
    for (int i = 0; i < report_info_arr->num_items; i++) {
        if (report_info_arr->item[i].item_type == type &&
            report_info_arr->item[i].attributes.usage.usage == usage) {
            if (item) {
                *item = &report_info_arr->item[i];
            }
            return true;
        }
    }

    return false;
}

bool hid_parse_find_bit_item_by_page(hid_report_info_t* report_info_arr, uint8_t type, uint16_t page, uint8_t bit, const hid_report_item_t **item)
{
    for (int i = 0; i < report_info_arr->num_items; i++) {
        if (report_info_arr->item[i].item_type == type &&
            report_info_arr->item[i].attributes.usage.page == page) {
            if (item) {
                if (i + bit < report_info_arr->num_items &&
                   report_info_arr->item[i + bit].item_type == type &&
                   report_info_arr->item[i + bit].attributes.usage.page == page) {
                    *item = &report_info_arr->item[i + bit];
                } else {
                    return false;
                }
            }
            return true;
        }
    }

    return false;
}

bool hid_parse_get_item_value(const hid_report_item_t *item, const uint8_t *report, uint8_t len, int32_t *value)
{
    if (item == NULL || report == NULL) {
        return false;
    }

    uint8_t boffs = item->bit_offset & 0x07;
    uint8_t pos = 8 - boffs;
    uint8_t offs  = item->bit_offset >> 3;
    uint32_t mask = ~(0xFFFFFFFF << item->bit_size);

    uint32_t val = report[offs++] >> boffs;

//printf("boffs=%d pos=%d offs=%d mask=%08X val=%02X\n", boffs, pos, offs, mask, val);

    while (item->bit_size > pos) {
        val |= (report[offs++] << pos);
        pos += 8;
//printf("val=%08X pos=%d\n", val, pos);
    }

    *value = val & mask;

    return true;
}
