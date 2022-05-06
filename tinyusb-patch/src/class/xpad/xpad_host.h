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

#ifndef _TUSB_XPAD_HOST_H_
#define _TUSB_XPAD_HOST_H_

//#include "cdc.h"

typedef enum
{
  XPAD_PIPE_NOTIFICATION , ///< Notification pipe
  XPAD_PIPE_DATA_IN      , ///< Data in pipe
  XPAD_PIPE_DATA_OUT     , ///< Data out pipe
  XPAD_PIPE_ERROR        , ///< Invalid Pipe ID
} xpad_pipeid_t;

typedef enum {
  XPAD_UNKNOWN,
  XPAD_360_WIRED,
  XPAD_XBONE
} xpad_ctype_t;

typedef enum {
    XPAD_HAT_UP    = 0x0001,
    XPAD_HAT_DOWN  = 0x0002,
    XPAD_HAT_LEFT  = 0x0004,
    XPAD_HAT_RIGHT = 0x0008,
    XPAD_START     = 0x0010,
    XPAD_BACK      = 0x0020,
    XPAD_STICK_L   = 0x0040,
    XPAD_STICK_R   = 0x0080,
    XPAD_PAD_LB    = 0x0100,
    XPAD_PAD_RB    = 0x0200,
    XPAD_XLOGO     = 0x0400,
    XPAD_PAD_A     = 0x1000,
    XPAD_PAD_B     = 0x2000,
    XPAD_PAD_X     = 0x4000,
    XPAD_PAD_Y     = 0x8000,
} xpad_pad_t;

typedef struct {
    xpad_pad_t	buttons;
    int16_t	lx;
    int16_t	ly;
    int16_t	rx;
    int16_t	ry;
    int16_t	lt;
    int16_t	rt;
} xpad_controller_t;

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------+
// CDC APPLICATION PUBLIC API
//--------------------------------------------------------------------+
/** \ingroup ClassDriver_CDC Communication Device Class (CDC)
 * \addtogroup XPAD_Serial Serial
 * @{
 * \defgroup   XPAD_Serial_Host Host
 * @{ */

/** \brief 			Check if device support CDC Serial interface or not
 * \param[in]		dev_addr	device address
 * \retval      true if device supports
 * \retval      false if device does not support or is not mounted
 */
bool tuh_xpad_serial_is_mounted(uint8_t dev_addr);

/** \brief      Check if the interface is currently busy or not
 * \param[in]   dev_addr device address
 * \param[in]   pipeid value from \ref xpad_pipeid_t to indicate target pipe.
 * \retval      true if the interface is busy, meaning the stack is still transferring/waiting data from/to device
 * \retval      false if the interface is not busy, meaning the stack successfully transferred data from/to device
 * \note        This function is used to check if previous transfer is complete (success or error), so that the next transfer
 *              can be scheduled. User needs to make sure the corresponding interface is mounted
 *              (by \ref tuh_xpad_serial_is_mounted) before calling this function.
 */
bool tuh_xpad_is_busy(uint8_t dev_addr, xpad_pipeid_t pipeid);

/** \brief 			Perform USB OUT transfer to device
 * \param[in]		dev_addr	device address
 * \param[in]	  p_data    Buffer containing data. Must be accessible by USB controller (see \ref CFG_TUSB_MEM_SECTION)
 * \param[in]		length    Number of bytes to be transferred via USB bus
 * \retval      TUSB_ERROR_NONE on success
 * \retval      TUSB_ERROR_INTERFACE_IS_BUSY if the interface is already transferring data with device
 * \retval      TUSB_ERROR_DEVICE_NOT_READY if device is not yet configured (by SET CONFIGURED request)
 * \retval      TUSB_ERROR_INVALID_PARA if input parameters are not correct
 * \note        This function is non-blocking and returns immediately. The result of USB transfer will be reported by the
 *              interface's callback function. \a p_data must be declared with \ref CFG_TUSB_MEM_SECTION.
 */
bool tuh_xpad_send(uint8_t dev_addr, void const * p_data, uint32_t length, bool is_notify);

/** \brief 			Perform USB IN transfer to get data from device
 * \param[in]		dev_addr	device address
 * \param[in]	  p_buffer  Buffer containing received data. Must be accessible by USB controller (see \ref CFG_TUSB_MEM_SECTION)
 * \param[in]		length    Number of bytes to be transferred via USB bus
 * \retval      TUSB_ERROR_NONE on success
 * \retval      TUSB_ERROR_INTERFACE_IS_BUSY if the interface is already transferring data with device
 * \retval      TUSB_ERROR_DEVICE_NOT_READY if device is not yet configured (by SET CONFIGURED request)
 * \retval      TUSB_ERROR_INVALID_PARA if input parameters are not correct
 * \note        This function is non-blocking and returns immediately. The result of USB transfer will be reported by the
 *              interface's callback function. \a p_data must be declared with \ref CFG_TUSB_MEM_SECTION.
 */
bool tuh_xpad_receive(uint8_t dev_addr, void * p_buffer, uint32_t length, bool is_notify);

//--------------------------------------------------------------------+
// XPAD APPLICATION CALLBACKS
//--------------------------------------------------------------------+

/** \brief      Callback function that is invoked when an transferring event occurred
 * \param[in]		dev_addr	Address of device
 * \param[in]   received data
 * \note        event can be one of following
 *              - XFER_RESULT_SUCCESS : previously scheduled transfer completes successfully.
 *              - XFER_RESULT_FAILED   : previously scheduled transfer encountered a transaction error.
 *              - XFER_RESULT_STALLED : previously scheduled transfer is stalled by device.
 * \note
 */

void tuh_xpad_read_cb(uint8_t dev_addr, uint8_t *report, xpad_controller_t *info);

void tuh_xpad_mount_cb(uint8_t dev_addr);

bool tuh_xpad_write(uint8_t dev_addr, uint8_t *report, int size);

bool tuh_xpad_vibro(uint8_t dev_addr, bool onoff);

/// @} // group XPAD_Serial_Host
/// @}

//--------------------------------------------------------------------+
// Internal Class Driver API
//--------------------------------------------------------------------+
void xpadh_init       (void);
bool xpadh_open       (uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *itf_desc, uint16_t max_len);
bool xpadh_set_config (uint8_t dev_addr, uint8_t itf_num);
bool xpadh_xfer_cb    (uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes);
void xpadh_close      (uint8_t dev_addr);

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_XPAD_HOST_H_ */
