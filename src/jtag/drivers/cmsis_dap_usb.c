//#define CMSIS_DAP_JTAG_DEBUG
/***************************************************************************
 *   Copyright (C) 2016 by Phillip Pearson                                 *
 *   pp@myelin.co.nz                                                       *
 *                                                                         *
 *   Copyright (C) 2014 by Paul Fertser                                    *
 *   fercerpav@gmail.com                                                   *
 *                                                                         *
 *   Copyright (C) 2013 by mike brown                                      *
 *   mike@theshedworks.org.uk                                              *
 *                                                                         *
 *   Copyright (C) 2013 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <transport/transport.h>
#include <jtag/swd.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <jtag/tcl.h>

#include <hidapi.h>

/*
 * See CMSIS-DAP documentation:
 * Version 0.01 - Beta.
 */

/* USB Config */

/* Known vid/pid pairs:
 * VID 0xc251: Keil Software
 * PID 0xf001: LPC-Link-II CMSIS_DAP
 * PID 0xf002: OPEN-SDA CMSIS_DAP (Freedom Board)
 * PID 0x2722: Keil ULINK2 CMSIS-DAP
 *
 * VID 0x0d28: mbed Software
 * PID 0x0204: MBED CMSIS-DAP
 */

#define MAX_USB_IDS 8
/* vid = pid = 0 marks the end of the list */
static uint16_t cmsis_dap_vid[MAX_USB_IDS + 1] = { 0 };
static uint16_t cmsis_dap_pid[MAX_USB_IDS + 1] = { 0 };
static wchar_t *cmsis_dap_serial;
static bool swd_mode;

#define PACKET_SIZE       (64 + 1)	/* 64 bytes plus report id */
#define USB_TIMEOUT       1000

/* CMSIS-DAP General Commands */
#define CMD_DAP_INFO              0x00
#define CMD_DAP_LED               0x01
#define CMD_DAP_CONNECT           0x02
#define CMD_DAP_DISCONNECT        0x03
#define CMD_DAP_WRITE_ABORT       0x08
#define CMD_DAP_DELAY             0x09
#define CMD_DAP_RESET_TARGET      0x0A

/* CMD_INFO */
#define INFO_ID_VID               0x00      /* string */
#define INFO_ID_PID               0x02      /* string */
#define INFO_ID_SERNUM            0x03      /* string */
#define INFO_ID_FW_VER            0x04      /* string */
#define INFO_ID_TD_VEND           0x05      /* string */
#define INFO_ID_TD_NAME           0x06      /* string */
#define INFO_ID_CAPS              0xf0      /* byte */
#define INFO_ID_PKT_CNT           0xfe      /* byte */
#define INFO_ID_PKT_SZ            0xff      /* short */

#define INFO_CAPS_SWD             0x01
#define INFO_CAPS_JTAG            0x02

/* CMD_LED */
#define LED_ID_CONNECT            0x00
#define LED_ID_RUN                0x01

#define LED_OFF                   0x00
#define LED_ON                    0x01

/* CMD_CONNECT */
#define CONNECT_DEFAULT           0x00
#define CONNECT_SWD               0x01
#define CONNECT_JTAG              0x02

/* CMSIS-DAP Common SWD/JTAG Commands */
#define CMD_DAP_DELAY             0x09
#define CMD_DAP_SWJ_PINS          0x10
#define CMD_DAP_SWJ_CLOCK         0x11
#define CMD_DAP_SWJ_SEQ           0x12

/*
 * PINS
 * Bit 0: SWCLK/TCK
 * Bit 1: SWDIO/TMS
 * Bit 2: TDI
 * Bit 3: TDO
 * Bit 5: nTRST
 * Bit 7: nRESET
 */

/* CMSIS-DAP SWD Commands */
#define CMD_DAP_SWD_CONFIGURE     0x13

/* CMSIS-DAP JTAG Commands */
#define CMD_DAP_JTAG_SEQ          0x14
#define CMD_DAP_JTAG_CONFIGURE    0x15
#define CMD_DAP_JTAG_IDCODE       0x16

/* CMSIS-DAP JTAG sequence info masks */
/* Number of bits to clock through (0 means 64) */
#define DAP_JTAG_SEQ_TCK          0x3F
/* TMS will be set during the sequence if this bit is set */
#define DAP_JTAG_SEQ_TMS          0x40
/* TDO output will be captured if this bit is set */
#define DAP_JTAG_SEQ_TDO          0x80


/* CMSIS-DAP Transfer Commands */
#define CMD_DAP_TFER_CONFIGURE    0x04
#define CMD_DAP_TFER              0x05
#define CMD_DAP_TFER_BLOCK        0x06
#define CMD_DAP_TFER_ABORT        0x07

/* DAP Status Code */
#define DAP_OK                    0
#define DAP_ERROR                 0xFF

/* CMSIS-DAP Vendor Commands
 * None as yet... */

static const char * const info_caps_str[] = {
	"SWD  Supported",
	"JTAG Supported"
};

/* max clock speed (kHz) */
#define DAP_MAX_CLOCK             5000

struct cmsis_dap {
	hid_device *dev_handle;
	uint16_t packet_size;
	uint16_t packet_count;
	uint8_t *packet_buffer;
	uint8_t caps;
	uint8_t mode;
};

struct pending_transfer_result {
	uint8_t cmd;
	uint32_t data;
	void *buffer;
};

static int pending_transfer_count, pending_queue_len;
static struct pending_transfer_result *pending_transfers;

static int queued_retval;

static struct cmsis_dap *cmsis_dap_handle;

static int cmsis_dap_usb_open(void)
{
	hid_device *dev = NULL;
	int i;
	struct hid_device_info *devs, *cur_dev;
	unsigned short target_vid, target_pid;
	wchar_t *target_serial = NULL;

	bool found = false;
	bool serial_found = false;

	target_vid = 0;
	target_pid = 0;

	/*
	 * The CMSIS-DAP specification stipulates:
	 * "The Product String must contain "CMSIS-DAP" somewhere in the string. This is used by the
	 * debuggers to identify a CMSIS-DAP compliant Debug Unit that is connected to a host computer."
	 */
	devs = hid_enumerate(0x0, 0x0);
	cur_dev = devs;
	while (NULL != cur_dev) {
		if (0 == cmsis_dap_vid[0]) {
			if (NULL == cur_dev->product_string) {
				LOG_DEBUG("Cannot read product string of device 0x%x:0x%x",
					  cur_dev->vendor_id, cur_dev->product_id);
			} else {
				if (wcsstr(cur_dev->product_string, L"CMSIS-DAP")) {
					/* if the user hasn't specified VID:PID *and*
					 * product string contains "CMSIS-DAP", pick it
					 */
					found = true;
				}
			}
		} else {
			/* otherwise, exhaustively compare against all VID:PID in list */
			for (i = 0; cmsis_dap_vid[i] || cmsis_dap_pid[i]; i++) {
				if ((cmsis_dap_vid[i] == cur_dev->vendor_id) && (cmsis_dap_pid[i] == cur_dev->product_id))
					found = true;
			}

			if (cmsis_dap_vid[i] || cmsis_dap_pid[i])
				found = true;
		}

		if (found) {
			/* we have found an adapter, so exit further checks */
			/* check serial number matches if given */
			if (cmsis_dap_serial != NULL) {
				if ((cur_dev->serial_number != NULL) && wcscmp(cmsis_dap_serial, cur_dev->serial_number) == 0) {
					serial_found = true;
					break;
				}
			} else
				break;

			found = false;
		}

		cur_dev = cur_dev->next;
	}

	if (NULL != cur_dev) {
		target_vid = cur_dev->vendor_id;
		target_pid = cur_dev->product_id;
		if (serial_found)
			target_serial = cmsis_dap_serial;
	}

	hid_free_enumeration(devs);

	if (target_vid == 0 && target_pid == 0) {
		LOG_ERROR("unable to find CMSIS-DAP device");
		return ERROR_FAIL;
	}

	if (hid_init() != 0) {
		LOG_ERROR("unable to open HIDAPI");
		return ERROR_FAIL;
	}

	dev = hid_open(target_vid, target_pid, target_serial);

	if (dev == NULL) {
		LOG_ERROR("unable to open CMSIS-DAP device 0x%x:0x%x", target_vid, target_pid);
		return ERROR_FAIL;
	}

	struct cmsis_dap *dap = malloc(sizeof(struct cmsis_dap));
	if (dap == NULL) {
		LOG_ERROR("unable to allocate memory");
		return ERROR_FAIL;
	}

	dap->dev_handle = dev;
	dap->caps = 0;
	dap->mode = 0;

	cmsis_dap_handle = dap;

	/* allocate default packet buffer, may be changed later.
	 * currently with HIDAPI we have no way of getting the output report length
	 * without this info we cannot communicate with the adapter.
	 * For the moment we ahve to hard code the packet size */

	int packet_size = PACKET_SIZE;

	/* atmel cmsis-dap uses 512 byte reports */
	/* TODO: HID report descriptor should be parsed instead of
	 * hardcoding a match by VID */
	if (target_vid == 0x03eb)
		packet_size = 512 + 1;

	cmsis_dap_handle->packet_buffer = malloc(packet_size);
	cmsis_dap_handle->packet_size = packet_size;

	if (cmsis_dap_handle->packet_buffer == NULL) {
		LOG_ERROR("unable to allocate memory");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static void cmsis_dap_usb_close(struct cmsis_dap *dap)
{
	hid_close(dap->dev_handle);
	hid_exit();

	free(cmsis_dap_handle->packet_buffer);
	free(cmsis_dap_handle);
	cmsis_dap_handle = NULL;
	free(cmsis_dap_serial);
	cmsis_dap_serial = NULL;
	free(pending_transfers);
	pending_transfers = NULL;

	return;
}

/* Send a message and receive the reply */
static int cmsis_dap_usb_xfer(struct cmsis_dap *dap, int txlen)
{
#ifdef CMSIS_DAP_JTAG_DEBUG
	LOG_DEBUG("cmsis-dap usb xfer cmd=%02X", dap->packet_buffer[1]);
#endif
	/* Pad the rest of the TX buffer with 0's */
	memset(dap->packet_buffer + txlen, 0, dap->packet_size - txlen);

	/* write data to device */
	int retval = hid_write(dap->dev_handle, dap->packet_buffer, dap->packet_size);
	if (retval == -1) {
		LOG_ERROR("error writing data: %ls", hid_error(dap->dev_handle));
		return ERROR_FAIL;
	}

	/* get reply */
	retval = hid_read_timeout(dap->dev_handle, dap->packet_buffer, dap->packet_size, USB_TIMEOUT);
	if (retval == -1 || retval == 0) {
		LOG_DEBUG("error reading data: %ls", hid_error(dap->dev_handle));
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int cmsis_dap_cmd_DAP_SWJ_Pins(uint8_t pins, uint8_t mask, uint32_t delay, uint8_t *input)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_SWJ_PINS;
	buffer[2] = pins;
	buffer[3] = mask;
	buffer[4] = delay & 0xff;
	buffer[5] = (delay >> 8) & 0xff;
	buffer[6] = (delay >> 16) & 0xff;
	buffer[7] = (delay >> 24) & 0xff;
	retval = cmsis_dap_usb_xfer(cmsis_dap_handle, 8);

	if (retval != ERROR_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_DAP_SWJ_PINS failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (input)
		*input = buffer[1];

	return ERROR_OK;
}

static int cmsis_dap_cmd_DAP_SWJ_Clock(uint32_t swj_clock)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	/* set clock in Hz */
	swj_clock *= 1000;
	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_SWJ_CLOCK;
	buffer[2] = swj_clock & 0xff;
	buffer[3] = (swj_clock >> 8) & 0xff;
	buffer[4] = (swj_clock >> 16) & 0xff;
	buffer[5] = (swj_clock >> 24) & 0xff;
	retval = cmsis_dap_usb_xfer(cmsis_dap_handle, 6);

	if (retval != ERROR_OK || buffer[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_DAP_SWJ_CLOCK failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

/* clock a sequence of bits out on TMS, to change JTAG states */
static int cmsis_dap_cmd_DAP_SWJ_Sequence(uint8_t s_len, const uint8_t* sequence)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

#ifdef CMSIS_DAP_JTAG_DEBUG
	LOG_DEBUG("cmsis-dap TMS sequence: len=%d", s_len);
	for (int i = 0; i < DIV_ROUND_UP(s_len, 8); ++i) {
		printf("%02X ", sequence[i]);
	}
	printf("\n");
#endif

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_SWJ_SEQ;
	buffer[2] = s_len;
	bit_copy(&buffer[3], 0, sequence, 0, s_len);

	retval = cmsis_dap_usb_xfer(cmsis_dap_handle, DIV_ROUND_UP(s_len, 8) + 3);

	if (retval != ERROR_OK || buffer[1] != DAP_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int cmsis_dap_cmd_DAP_Info(uint8_t info, uint8_t **data)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_INFO;
	buffer[2] = info;
	retval = cmsis_dap_usb_xfer(cmsis_dap_handle, 3);

	if (retval != ERROR_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_INFO failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	*data = &(buffer[1]);

	return ERROR_OK;
}

static int cmsis_dap_cmd_DAP_LED(uint8_t leds)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_LED;
	buffer[2] = 0x00;
	buffer[3] = leds;
	retval = cmsis_dap_usb_xfer(cmsis_dap_handle, 4);

	if (retval != ERROR_OK || buffer[1] != 0x00) {
		LOG_ERROR("CMSIS-DAP command CMD_LED failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int cmsis_dap_cmd_DAP_Connect(uint8_t mode)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_CONNECT;
	buffer[2] = mode;
	retval = cmsis_dap_usb_xfer(cmsis_dap_handle, 3);

	if (retval != ERROR_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_CONNECT failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (buffer[1] != mode) {
		LOG_ERROR("CMSIS-DAP failed to connect in mode (%d)", mode);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int cmsis_dap_cmd_DAP_Disconnect(void)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_DISCONNECT;
	retval = cmsis_dap_usb_xfer(cmsis_dap_handle, 2);

	if (retval != ERROR_OK || buffer[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_DISCONNECT failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int cmsis_dap_cmd_DAP_TFER_Configure(uint8_t idle, uint16_t retry_count, uint16_t match_retry)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_TFER_CONFIGURE;
	buffer[2] = idle;
	buffer[3] = retry_count & 0xff;
	buffer[4] = (retry_count >> 8) & 0xff;
	buffer[5] = match_retry & 0xff;
	buffer[6] = (match_retry >> 8) & 0xff;
	retval = cmsis_dap_usb_xfer(cmsis_dap_handle, 7);

	if (retval != ERROR_OK || buffer[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_TFER_Configure failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int cmsis_dap_cmd_DAP_SWD_Configure(uint8_t cfg)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_SWD_CONFIGURE;
	buffer[2] = cfg;
	retval = cmsis_dap_usb_xfer(cmsis_dap_handle, 3);

	if (retval != ERROR_OK || buffer[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_SWD_Configure failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

#if 0
static int cmsis_dap_cmd_DAP_Delay(uint16_t delay_us)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_DELAY;
	buffer[2] = delay_us & 0xff;
	buffer[3] = (delay_us >> 8) & 0xff;
	retval = cmsis_dap_usb_xfer(cmsis_dap_handle, 4);

	if (retval != ERROR_OK || buffer[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_Delay failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}
#endif

static int cmsis_dap_swd_run_queue(void)
{
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	LOG_DEBUG("Executing %d queued transactions", pending_transfer_count);

	if (queued_retval != ERROR_OK) {
		LOG_DEBUG("Skipping due to previous errors: %d", queued_retval);
		goto skip;
	}

	if (!pending_transfer_count)
		goto skip;

	size_t idx = 0;
	buffer[idx++] = 0;	/* report number */
	buffer[idx++] = CMD_DAP_TFER;
	buffer[idx++] = 0x00;	/* DAP Index */
	buffer[idx++] = pending_transfer_count;

	for (int i = 0; i < pending_transfer_count; i++) {
		uint8_t cmd = pending_transfers[i].cmd;
		uint32_t data = pending_transfers[i].data;

		LOG_DEBUG("%s %s reg %x %"PRIx32,
				cmd & SWD_CMD_APnDP ? "AP" : "DP",
				cmd & SWD_CMD_RnW ? "read" : "write",
			  (cmd & SWD_CMD_A32) >> 1, data);

		/* When proper WAIT handling is implemented in the
		 * common SWD framework, this kludge can be
		 * removed. However, this might lead to minor
		 * performance degradation as the adapter wouldn't be
		 * able to automatically retry anything (because ARM
		 * has forgotten to implement sticky error flags
		 * clearing). See also comments regarding
		 * cmsis_dap_cmd_DAP_TFER_Configure() and
		 * cmsis_dap_cmd_DAP_SWD_Configure() in
		 * cmsis_dap_init().
		 */
		if (!(cmd & SWD_CMD_RnW) &&
		    !(cmd & SWD_CMD_APnDP) &&
		    (cmd & SWD_CMD_A32) >> 1 == DP_CTRL_STAT &&
		    (data & CORUNDETECT)) {
			LOG_DEBUG("refusing to enable sticky overrun detection");
			data &= ~CORUNDETECT;
		}

		buffer[idx++] = (cmd >> 1) & 0x0f;
		if (!(cmd & SWD_CMD_RnW)) {
			buffer[idx++] = (data) & 0xff;
			buffer[idx++] = (data >> 8) & 0xff;
			buffer[idx++] = (data >> 16) & 0xff;
			buffer[idx++] = (data >> 24) & 0xff;
		}
	}

	queued_retval = cmsis_dap_usb_xfer(cmsis_dap_handle, idx);
	if (queued_retval != ERROR_OK)
		goto skip;

	idx = 2;
	uint8_t ack = buffer[idx] & 0x07;
	if (ack != SWD_ACK_OK || (buffer[idx] & 0x08)) {
		LOG_DEBUG("SWD ack not OK: %d %s", buffer[idx-1],
			  ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK");
		queued_retval = ack == SWD_ACK_WAIT ? ERROR_WAIT : ERROR_FAIL;
		goto skip;
	}
	idx++;

	if (pending_transfer_count != buffer[1])
		LOG_ERROR("CMSIS-DAP transfer count mismatch: expected %d, got %d",
			  pending_transfer_count, buffer[1]);

	for (int i = 0; i < buffer[1]; i++) {
		if (pending_transfers[i].cmd & SWD_CMD_RnW) {
			static uint32_t last_read;
			uint32_t data = le_to_h_u32(&buffer[idx]);
			uint32_t tmp = data;
			idx += 4;

			LOG_DEBUG("Read result: %"PRIx32, data);

			/* Imitate posted AP reads */
			if ((pending_transfers[i].cmd & SWD_CMD_APnDP) ||
			    ((pending_transfers[i].cmd & SWD_CMD_A32) >> 1 == DP_RDBUFF)) {
				tmp = last_read;
				last_read = data;
			}

			if (pending_transfers[i].buffer)
				*(uint32_t *)pending_transfers[i].buffer = tmp;
		}
	}

skip:
	pending_transfer_count = 0;
	int retval = queued_retval;
	queued_retval = ERROR_OK;

	return retval;
}

static void cmsis_dap_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data)
{
	if (pending_transfer_count == pending_queue_len) {
		/* Not enough room in the queue. Run the queue. */
		queued_retval = cmsis_dap_swd_run_queue();
	}

	if (queued_retval != ERROR_OK)
		return;

	pending_transfers[pending_transfer_count].data = data;
	pending_transfers[pending_transfer_count].cmd = cmd;
	if (cmd & SWD_CMD_RnW) {
		/* Queue a read transaction */
		pending_transfers[pending_transfer_count].buffer = dst;
	}
	pending_transfer_count++;
}

static void cmsis_dap_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RnW));
	cmsis_dap_swd_queue_cmd(cmd, NULL, value);
}

static void cmsis_dap_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RnW);
	cmsis_dap_swd_queue_cmd(cmd, value, 0);
}

static int cmsis_dap_get_version_info(void)
{
	uint8_t *data;

	/* INFO_ID_FW_VER - string */
	int retval = cmsis_dap_cmd_DAP_Info(INFO_ID_FW_VER, &data);
	if (retval != ERROR_OK)
		return retval;

	if (data[0]) /* strlen */
		LOG_INFO("CMSIS-DAP: FW Version = %s", &data[1]);

	return ERROR_OK;
}

static int cmsis_dap_get_caps_info(void)
{
	uint8_t *data;

	/* INFO_ID_CAPS - byte */
	int retval = cmsis_dap_cmd_DAP_Info(INFO_ID_CAPS, &data);
	if (retval != ERROR_OK)
		return retval;

	if (data[0] == 1) {
		uint8_t caps = data[1];

		cmsis_dap_handle->caps = caps;

		if (caps & INFO_CAPS_SWD)
			LOG_INFO("CMSIS-DAP: %s", info_caps_str[0]);
		if (caps & INFO_CAPS_JTAG)
			LOG_INFO("CMSIS-DAP: %s", info_caps_str[1]);
	}

	return ERROR_OK;
}

static int cmsis_dap_get_status(void)
{
	uint8_t d;

	int retval = cmsis_dap_cmd_DAP_SWJ_Pins(0, 0, 0, &d);

	if (retval == ERROR_OK) {
		LOG_INFO("SWCLK/TCK = %d SWDIO/TMS = %d TDI = %d TDO = %d nTRST = %d nRESET = %d",
			(d & (0x01 << 0)) ? 1 : 0,	/* Bit 0: SWCLK/TCK */
			(d & (0x01 << 1)) ? 1 : 0,	/* Bit 1: SWDIO/TMS */
			(d & (0x01 << 2)) ? 1 : 0,	/* Bit 2: TDI */
			(d & (0x01 << 3)) ? 1 : 0,	/* Bit 3: TDO */
			(d & (0x01 << 5)) ? 1 : 0,	/* Bit 5: nTRST */
			(d & (0x01 << 7)) ? 1 : 0);	/* Bit 7: nRESET */
	}

	return retval;
}

static int cmsis_dap_swd_switch_seq(enum swd_special_seq seq)
{
	const uint8_t *s;
	unsigned int s_len;
	int retval;

	/* First disconnect before connecting, Atmel EDBG needs it for SAMD/R/L/C */
	cmsis_dap_cmd_DAP_Disconnect();

	/* When we are reconnecting, DAP_Connect needs to be rerun, at
	 * least on Keil ULINK-ME */
	retval = cmsis_dap_cmd_DAP_Connect(seq == LINE_RESET || seq == JTAG_TO_SWD ?
					   CONNECT_SWD : CONNECT_JTAG);
	if (retval != ERROR_OK)
		return retval;

	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG("SWD line reset");
		s = swd_seq_line_reset;
		s_len = swd_seq_line_reset_len;
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		s = swd_seq_jtag_to_swd;
		s_len = swd_seq_jtag_to_swd_len;
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		s = swd_seq_swd_to_jtag;
		s_len = swd_seq_swd_to_jtag_len;
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	return cmsis_dap_cmd_DAP_SWJ_Sequence(s_len, s);
}

static int cmsis_dap_swd_open(void)
{
	int retval;

	if (cmsis_dap_handle == NULL) {
		/* SWD init */
		retval = cmsis_dap_usb_open();
		if (retval != ERROR_OK)
			return retval;

		retval = cmsis_dap_get_caps_info();
		if (retval != ERROR_OK)
			return retval;
	}

	if (!(cmsis_dap_handle->caps & INFO_CAPS_SWD)) {
		LOG_ERROR("CMSIS-DAP: SWD not supported");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	retval = cmsis_dap_cmd_DAP_Connect(CONNECT_SWD);
	if (retval != ERROR_OK)
		return retval;

	/* Add more setup here.??... */

	LOG_INFO("CMSIS-DAP: Interface Initialised (SWD)");
	return ERROR_OK;
}

static int cmsis_dap_init(void)
{
	int retval;
	uint8_t *data;

	if (swd_mode) {
		retval = cmsis_dap_swd_open();
		if (retval != ERROR_OK)
			return retval;
	}

	if (cmsis_dap_handle == NULL) {

		/* JTAG init */
		retval = cmsis_dap_usb_open();
		if (retval != ERROR_OK)
			return retval;

		retval = cmsis_dap_get_caps_info();
		if (retval != ERROR_OK)
			return retval;

		/* Connect in JTAG mode */
		if (!(cmsis_dap_handle->caps & INFO_CAPS_JTAG)) {
			LOG_ERROR("CMSIS-DAP: JTAG not supported");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		retval = cmsis_dap_cmd_DAP_Connect(CONNECT_JTAG);
		if (retval != ERROR_OK)
			return retval;

		LOG_INFO("CMSIS-DAP: Interface Initialised (JTAG)");
	}

	retval = cmsis_dap_get_version_info();
	if (retval != ERROR_OK)
		return retval;

	/* INFO_ID_PKT_SZ - short */
	retval = cmsis_dap_cmd_DAP_Info(INFO_ID_PKT_SZ, &data);
	if (retval != ERROR_OK)
		return retval;

	if (data[0] == 2) {  /* short */
		uint16_t pkt_sz = data[1] + (data[2] << 8);

		/* 4 bytes of command header + 5 bytes per register
		 * write. For bulk read sequences just 4 bytes are
		 * needed per transfer, so this is suboptimal. */
		pending_queue_len = (pkt_sz - 4) / 5;
		pending_transfers = malloc(pending_queue_len * sizeof(*pending_transfers));
		if (!pending_transfers) {
			LOG_ERROR("Unable to allocate memory for CMSIS-DAP queue");
			return ERROR_FAIL;
		}

		if (cmsis_dap_handle->packet_size != pkt_sz + 1) {
			/* reallocate buffer */
			cmsis_dap_handle->packet_size = pkt_sz + 1;
			cmsis_dap_handle->packet_buffer = realloc(cmsis_dap_handle->packet_buffer,
					cmsis_dap_handle->packet_size);
			if (cmsis_dap_handle->packet_buffer == NULL) {
				LOG_ERROR("unable to reallocate memory");
				return ERROR_FAIL;
			}
		}

		LOG_DEBUG("CMSIS-DAP: Packet Size = %" PRId16, pkt_sz);
	}

	/* INFO_ID_PKT_CNT - byte */
	retval = cmsis_dap_cmd_DAP_Info(INFO_ID_PKT_CNT, &data);
	if (retval != ERROR_OK)
		return retval;

	if (data[0] == 1) { /* byte */
		uint16_t pkt_cnt = data[1];
		cmsis_dap_handle->packet_count = pkt_cnt;
		LOG_DEBUG("CMSIS-DAP: Packet Count = %" PRId16, pkt_cnt);
	}

	retval = cmsis_dap_get_status();
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	/* Now try to connect to the target
	 * TODO: This is all SWD only @ present */
	retval = cmsis_dap_cmd_DAP_SWJ_Clock(jtag_get_speed_khz());
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	/* Ask CMSIS-DAP to automatically retry on receiving WAIT for
	 * up to 64 times. This must be changed to 0 if sticky
	 * overrun detection is enabled. */
	retval = cmsis_dap_cmd_DAP_TFER_Configure(0, 64, 0);
	if (retval != ERROR_OK)
		return ERROR_FAIL;
	/* Data Phase (bit 2) must be set to 1 if sticky overrun
	 * detection is enabled */
	retval = cmsis_dap_cmd_DAP_SWD_Configure(0);	/* 1 TRN, no Data Phase */
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	retval = cmsis_dap_cmd_DAP_LED(0x03);		/* Both LEDs on */
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	/* support connecting with srst asserted */
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
		if (jtag_reset_config & RESET_SRST_NO_GATING) {
			retval = cmsis_dap_cmd_DAP_SWJ_Pins(0, (1 << 7), 0, NULL);
			if (retval != ERROR_OK)
				return ERROR_FAIL;
			LOG_INFO("Connecting under reset");
		}
	}

	cmsis_dap_cmd_DAP_LED(0x00);			/* Both LEDs off */

	LOG_INFO("CMSIS-DAP: Interface ready");

	return ERROR_OK;
}

static int cmsis_dap_swd_init(void)
{
	swd_mode = true;
	return ERROR_OK;
}

static int cmsis_dap_quit(void)
{
	cmsis_dap_cmd_DAP_Disconnect();
	cmsis_dap_cmd_DAP_LED(0x00);		/* Both LEDs off */

	cmsis_dap_usb_close(cmsis_dap_handle);

	return ERROR_OK;
}

static void cmsis_dap_execute_reset(struct jtag_command *cmd)
{
	int retval = cmsis_dap_cmd_DAP_SWJ_Pins(cmd->cmd.reset->srst ? 0 : (1 << 7), \
			(1 << 7), 0, NULL);
	if (retval != ERROR_OK)
		LOG_ERROR("CMSIS-DAP: Interface reset failed");
}

static void cmsis_dap_execute_sleep(struct jtag_command *cmd)
{
#if 0
	int retval = cmsis_dap_cmd_DAP_Delay(cmd->cmd.sleep->us);
	if (retval != ERROR_OK)
#endif
		jtag_sleep(cmd->cmd.sleep->us);
}

/* Set TMS high for five TCK clocks, to move the TAP to the Test-Logic-Reset state */
static int cmsis_dap_execute_tlr_reset(struct jtag_command *cmd)
{
	LOG_INFO("cmsis-dap JTAG TLR_RESET");
	uint8_t seq = 0xff;
	int ret = cmsis_dap_cmd_DAP_SWJ_Sequence(8, &seq);
	if (ret == ERROR_OK)
		tap_set_state(TAP_RESET);
	return ret;
}

/* Set new end state */
static void cmsis_dap_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

#ifdef CMSIS_DAP_JTAG_DEBUG
static void sprint_binary(char* s, const uint8_t* buf, int offset, int len) {
	if (!len) return;

	/*
	buf = { 0x18 } len=5 should result in: 11000
	buf = { 0xff 0x18 } len=13 should result in: 11111111 11000
	buf = { 0xc0 0x18 } offset=3 len=10 should result in: 11000 11000
		i=3 there means i/8 = 0 so c = 0xFF, and 
	*/
	for (int i = offset; i < offset + len; ++i) {
		uint8_t c = buf[i / 8], mask = 1 << (i % 8);
		if ((i != offset) && !(i % 8)) putchar(' ');
		*s++ = (c & mask) ? '1' : '0';
	}
	*s = 0;
}
#endif

/*
static void bit_copy_debug(uint8_t *dst, unsigned dst_offset, const uint8_t *src, unsigned src_offset, unsigned bit_count) {
	printf("bit copy dst_offset=%d src_offset=%d bit_count=%d: ", dst_offset, src_offset, bit_count);
	print_binary(src, src_offset, bit_count);
	printf("\n");
	bit_copy(dst, dst_offset, src, src_offset, bit_count);
}
*/

#ifdef CMSIS_DAP_JTAG_DEBUG
static void debug_parse_cmsis_buf(const uint8_t* cmd, int cmdlen) {
	/* cmd is a usb packet to go to the cmsis-dap interface */
	printf("cmsis-dap buffer (%d b): ", cmdlen);
	for (int i = 0; i < cmdlen; ++i) { printf(" %02x", cmd[i]); }
	printf("\n");
	switch (cmd[1]) {
		case CMD_DAP_JTAG_SEQ: {
			printf("cmsis-dap jtag sequence command %02x (n=%d)\n", cmd[1], cmd[2]);
			/*
			 * #2 = number of sequences
			 * #3 = sequence info 1
			 * #4...4+n_bytes-1 = sequence 1
			 * #4+n_bytes = sequence info 2
			 * #5+n_bytes = sequence 2 (single bit)
			 */
			int pos = 3;
			for (int seq = 0; seq < cmd[2]; ++seq) {
				uint8_t info = cmd[pos++];
				int len = info & DAP_JTAG_SEQ_TCK;
				if (len == 0) len = 64;
				printf("  sequence %d starting %d: info %02x (len=%d tms=%d read_tdo=%d): ",
					seq, pos, info, len, info & DAP_JTAG_SEQ_TMS, info & DAP_JTAG_SEQ_TDO);
				for (int i = 0; i < DIV_ROUND_UP(len, 8); ++i) { printf(" %02x", cmd[pos+i]); }
				pos += DIV_ROUND_UP(len, 8);
				printf("\n");
			}
			if (pos != cmdlen) {
				printf("BUFFER LENGTH MISMATCH looks like %d but %d specified", pos, cmdlen);
				exit(-1);
			}

			break;
		}
		default:
			LOG_DEBUG("unknown cmsis-dap command %02x", cmd[1]);
			break;
	}
}
#endif


/* Move to the end state by clocking bits into TMS as appropriate using
 * the CMSIS-DAP DAP_SWJ_Sequence command.
 */
static void cmsis_dap_state_move()
{
	// LOG_DEBUG("cmsis_dap_state_move from %s to %s", tap_state_name(tap_get_state()), tap_state_name(tap_get_end_state()));

	/* tap_get_tms_path() can only handle moving between stable states, so if we're in an unstable state,
	 * move to a stable state first.
	 */
	bool exit_to_pause = false;
	switch (tap_get_state()) {
	case TAP_DREXIT1:
		// LOG_DEBUG("moving out of DREXIT1");
		exit_to_pause = true;
		tap_set_state(TAP_DRPAUSE);
		break;
	case TAP_IREXIT1:
		// LOG_DEBUG("moving out of IREXIT1");
		exit_to_pause = true;
		tap_set_state(TAP_IRPAUSE);
		break;
	default:
		break;
	}

	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
#ifdef CMSIS_DAP_JTAG_DEBUG
	LOG_DEBUG("STATE MOVE %02X (%d clocks) to get from %s to %s",
		tms_scan, tms_count,
		tap_state_name(tap_get_state()), tap_state_name(tap_get_end_state()));
#endif

	if (exit_to_pause) {
		/* need to shift in an extra 0 before the usual sequence */
		uint8_t scan[2];
		scan[0] = (uint8_t)(tms_scan << 1);
		scan[1] = (uint8_t)(tms_scan >> 7);
#ifdef CMSIS_DAP_JTAG_DEBUG
		LOG_DEBUG("add in an extra 1 to get out of ?REXIT: now %02X (%d clocks)", scan[0], tms_count + 1);
#endif
		cmsis_dap_cmd_DAP_SWJ_Sequence(tms_count + 1, scan);
	} else {
		cmsis_dap_cmd_DAP_SWJ_Sequence(tms_count, &tms_scan);
	}

	tap_set_state(tap_get_end_state());
}

/* Execute a scan: move to the TAP_IRSHIFT or TAP_DRSHIFT state, clock in/out some data, then move to the end state */
static int cmsis_dap_scan(bool ir_scan, enum scan_type type, uint8_t *sequence, int scan_size)
{
	tap_state_t saved_end_state = tap_get_end_state();

	if (!((!ir_scan &&
			(tap_get_state() == TAP_DRSHIFT)) ||
			(ir_scan && (tap_get_state() == TAP_IRSHIFT)))) {
		if (ir_scan)
			cmsis_dap_end_state(TAP_IRSHIFT);
		else
			cmsis_dap_end_state(TAP_DRSHIFT);

		cmsis_dap_state_move();
		cmsis_dap_end_state(saved_end_state);
	}

#ifdef CMSIS_DAP_JTAG_DEBUG
	char debug[2000];
	sprintf(debug, "%cR in ", ir_scan ? 'I' : 'D');
	sprint_binary(debug + strlen(debug), sequence, 0, scan_size);
#endif

	/* Split scan up into 64-bit chunks and issue a new USB request for each one.  This is suboptimal,
	 * but if we only use it for the inital scan chain enumeration, it doesn't matter.  If it turns out
	 * that drivers habitually make long scan requests, modify this to pack as much as possible into
	 * each USB request.
	 */
	for (int scan_start = 0; scan_start < scan_size; scan_start += 64) {

		/* If this is the last chunk, we need to set TMS when sending the final bit */
		bool last_chunk = (scan_start + 64) >= scan_size;
		/* Number of bits to send/receive in this chunk */
		int chunk_bits = last_chunk ? (scan_size - scan_start) : 64;
#ifdef CMSIS_DAP_JTAG_DEBUG
		// LOG_INFO("cmsis-dap jtag chunk: scan_start %d scan_size %d chunk_bits %d last_chunk %d", scan_start, scan_size, chunk_bits, last_chunk ? 1 : 0);
#endif

		/* Prepare CMSIS-DAP JTAG Sequence command:
		 *
		 * #0 = 0 (report number)
		 * #1 = CMD_DAP_JTAG_SEQ
		 * #2 = number of sequences
		 * #3 = sequence info 1
		 * #4...4+n_bytes-1 = sequence 1
		 * #4+n_bytes = sequence info 2
		 * #5+n_bytes = sequence 2 (single bit)
		 *
		 * so a 32-bit sequence will go in #4-7, leaving #8 and #9 for sequence 2
		 */
		uint8_t *buffer = cmsis_dap_handle->packet_buffer;
		buffer[0] = 0;	/* report number */
		buffer[1] = CMD_DAP_JTAG_SEQ;
		buffer[2] = 1; /* default 1 sequence */
		uint8_t buf_ptr = 3; /* data blocks start at buffer[3] */
		uint8_t first_seq_start = 4; /* start of first buffer */
		uint8_t second_seq_start = 0; /* start of second data buffer if we use one */

		if (chunk_bits == 0) {
			LOG_ERROR("BUG: Empty scan chunk");
			exit(-1);
		} else if (chunk_bits > 64) {
			LOG_ERROR("BUG: Scan chunk > 64 bits");
			exit(-1);
		} else if (!last_chunk) {
			/* We're only sending part of the scan request, so don't raise TMS on the final bit */
			/* 64 bits with TMS=0 */
			buffer[buf_ptr++] = DAP_JTAG_SEQ_TDO;
			/* copying 64 bits from sequence, starting at scan_start, into buffer[3...] */
			bit_copy(&buffer[buf_ptr], 0, sequence, scan_start, 64);
			buf_ptr += 8;
		} else if (chunk_bits == 1) {
			/* Just clock out one bit with TMS=1 */
			buffer[buf_ptr++] = DAP_JTAG_SEQ_TDO | DAP_JTAG_SEQ_TMS | 1;
			bit_copy(&buffer[buf_ptr++], 0, sequence, scan_start, 1);
		} else {
			/* Split our scan into two sequences, one with TMS=0 and (scan_size-1) bits, and one with TMS=0 and 1 bit. */
			buffer[2] = 2;

			/* Sequence 1 */
			buffer[buf_ptr++] = DAP_JTAG_SEQ_TDO | (chunk_bits - 1);
			bit_copy(&buffer[buf_ptr], 0, sequence, scan_start, chunk_bits - 1);

			/* Figure out how many bytes we just used in the buffer */
			buf_ptr += DIV_ROUND_UP((chunk_bits - 1), 8);

			/* Sequence 2: the final bit */
			buffer[buf_ptr++] = DAP_JTAG_SEQ_TDO | DAP_JTAG_SEQ_TMS | 1;
			second_seq_start = buf_ptr;
			bit_copy(&buffer[buf_ptr++], 0, sequence, scan_start + chunk_bits - 1, 1);
		}

#ifdef CMSIS_DAP_JTAG_DEBUG
		debug_parse_cmsis_buf(buffer, buf_ptr);
#endif

		/* Send command */
		int retval = cmsis_dap_usb_xfer(cmsis_dap_handle, buf_ptr);

		if (retval != ERROR_OK || buffer[1] != DAP_OK) {
			LOG_ERROR("CMSIS-DAP command CMD_DAP_JTAG_SEQ failed.");
			return ERROR_JTAG_DEVICE_ERROR;
		}

#ifdef CMSIS_DAP_JTAG_DEBUG
		printf("cmsis-dap buffer at end: ");
		for (int i = 0; i < buf_ptr; ++i) { printf(" %02x", buffer[i]); }
		printf("\n");
#endif

		/* Copy results into the sequence buffer if we want them */
		if (type == SCAN_IN || type == SCAN_IO) {
			if (!last_chunk) {
				/* 64 bit chunk, single sequence */
				bit_copy(sequence, scan_start, &buffer[first_seq_start-2], 0, 64);
			} else if (chunk_bits == 1) {
				/* 1 bit chunk, single sequence */
				bit_copy(sequence, scan_start, &buffer[first_seq_start-2], 0, 1);
			} else {
				/* last chunk, two sequences */
				bit_copy(sequence, scan_start, &buffer[first_seq_start-2], 0, chunk_bits - 1);
				bit_copy(sequence, scan_start + chunk_bits - 1, &buffer[second_seq_start-3], 0, 1);
			}
		}
	}

#ifdef CMSIS_DAP_JTAG_DEBUG
	if (type == SCAN_IN || type == SCAN_IO) {
		strcat(debug, " out ");
		sprint_binary(debug + strlen(debug), sequence, 0, scan_size);
	}
	LOG_DEBUG("%s", debug);
#endif

	/* The final bit will have transitioned us to Exit1-DR or Exit1-IR */
	tap_set_state(tap_get_state() == TAP_DRSHIFT ? TAP_DREXIT1 : TAP_IREXIT1);

	if (tap_get_state() != tap_get_end_state()) {
		cmsis_dap_state_move();
	}

	return ERROR_OK;
}

static int cmsis_dap_execute_scan(struct jtag_command *cmd)
{
#ifdef _DEBUG_JTAG_IO_
	LOG_DEBUG("%s scan end in %s",
			(cmd->cmd.scan->ir_scan) ? "IR" : "DR",
		tap_state_name(cmd->cmd.scan->end_state));
#endif
	cmsis_dap_end_state(cmd->cmd.scan->end_state);
	uint8_t *buffer;
	int scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
	// printf("scan size %d, bytes", scan_size);
	// for (int i = 0; i < DIV_ROUND_UP(scan_size, 8); ++i) printf(" %02x", buffer[i]);
	// printf("\n");
	int type = jtag_scan_type(cmd->cmd.scan);
	int retval = cmsis_dap_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
	if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
		retval = ERROR_JTAG_QUEUE_FAILED;
	if (buffer)
		free(buffer);
	return retval;
}

static void cmsis_dap_execute_command(struct jtag_command *cmd)
{
	switch (cmd->type) {
		case JTAG_RESET:
			cmsis_dap_execute_reset(cmd);
			break;
		case JTAG_SLEEP:
			cmsis_dap_execute_sleep(cmd);
			break;
		case JTAG_TLR_RESET:
			cmsis_dap_execute_tlr_reset(cmd);
			break;
		case JTAG_SCAN:
			cmsis_dap_execute_scan(cmd);
			break;
		case JTAG_RUNTEST:
		case JTAG_PATHMOVE:
		case JTAG_STABLECLOCKS:
		case JTAG_TMS:
		default:
			LOG_ERROR("BUG: unknown JTAG command type 0x%X encountered", cmd->type);
			exit(-1);
	}
}

static int cmsis_dap_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;

	while (cmd != NULL) {
		cmsis_dap_execute_command(cmd);
		cmd = cmd->next;
	}

	return ERROR_OK;
}

static int cmsis_dap_speed(int speed)
{
	if (speed > DAP_MAX_CLOCK) {
		LOG_INFO("reduce speed request: %dkHz to %dkHz maximum", speed, DAP_MAX_CLOCK);
		speed = DAP_MAX_CLOCK;
	}

	if (speed == 0) {
		LOG_INFO("RTCK not supported");
		return ERROR_JTAG_NOT_IMPLEMENTED;
	}

	return cmsis_dap_cmd_DAP_SWJ_Clock(speed);
}

static int cmsis_dap_speed_div(int speed, int *khz)
{
	*khz = speed;
	return ERROR_OK;
}

static int cmsis_dap_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;
	return ERROR_OK;
}

static int_least32_t cmsis_dap_swd_frequency(int_least32_t hz)
{
	if (hz > 0)
		cmsis_dap_speed(hz / 1000);

	return hz;
}


COMMAND_HANDLER(cmsis_dap_handle_info_command)
{
	if (cmsis_dap_get_version_info() == ERROR_OK)
		cmsis_dap_get_status();

	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_vid_pid_command)
{
	if (CMD_ARGC > MAX_USB_IDS * 2) {
		LOG_WARNING("ignoring extra IDs in cmsis_dap_vid_pid "
			"(maximum is %d pairs)", MAX_USB_IDS);
		CMD_ARGC = MAX_USB_IDS * 2;
	}
	if (CMD_ARGC < 2 || (CMD_ARGC & 1)) {
		LOG_WARNING("incomplete cmsis_dap_vid_pid configuration directive");
		if (CMD_ARGC < 2)
			return ERROR_COMMAND_SYNTAX_ERROR;
		/* remove the incomplete trailing id */
		CMD_ARGC -= 1;
	}

	unsigned i;
	for (i = 0; i < CMD_ARGC; i += 2) {
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i], cmsis_dap_vid[i >> 1]);
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], cmsis_dap_pid[i >> 1]);
	}

	/*
	 * Explicitly terminate, in case there are multiples instances of
	 * cmsis_dap_vid_pid.
	 */
	cmsis_dap_vid[i >> 1] = cmsis_dap_pid[i >> 1] = 0;

	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_serial_command)
{
	if (CMD_ARGC == 1) {
		size_t len = mbstowcs(NULL, CMD_ARGV[0], 0);
		cmsis_dap_serial = calloc(len + 1, sizeof(wchar_t));
		if (cmsis_dap_serial == NULL) {
			LOG_ERROR("unable to allocate memory");
			return ERROR_OK;
		}
		if (mbstowcs(cmsis_dap_serial, CMD_ARGV[0], len + 1) == (size_t)-1) {
			free(cmsis_dap_serial);
			cmsis_dap_serial = NULL;
			LOG_ERROR("unable to convert serial");
		}
	} else {
		LOG_ERROR("expected exactly one argument to cmsis_dap_serial <serial-number>");
	}

	return ERROR_OK;
}

static const struct command_registration cmsis_dap_subcommand_handlers[] = {
	{
		.name = "info",
		.handler = &cmsis_dap_handle_info_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "show cmsis-dap info",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration cmsis_dap_command_handlers[] = {
	{
		.name = "cmsis-dap",
		.mode = COMMAND_ANY,
		.help = "perform CMSIS-DAP management",
		.usage = "<cmd>",
		.chain = cmsis_dap_subcommand_handlers,
	},
	{
		.name = "cmsis_dap_vid_pid",
		.handler = &cmsis_dap_handle_vid_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "the vendor ID and product ID of the CMSIS-DAP device",
		.usage = "(vid pid)* ",
	},
	{
		.name = "cmsis_dap_serial",
		.handler = &cmsis_dap_handle_serial_command,
		.mode = COMMAND_CONFIG,
		.help = "set the serial number of the adapter",
		.usage = "serial_string",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct swd_driver cmsis_dap_swd_driver = {
	.init = cmsis_dap_swd_init,
	.frequency = cmsis_dap_swd_frequency,
	.switch_seq = cmsis_dap_swd_switch_seq,
	.read_reg = cmsis_dap_swd_read_reg,
	.write_reg = cmsis_dap_swd_write_reg,
	.run = cmsis_dap_swd_run_queue,
};

static const char * const cmsis_dap_transport[] = { "swd", "jtag", NULL };

struct jtag_interface cmsis_dap_interface = {
	.name = "cmsis-dap",
	.commands = cmsis_dap_command_handlers,
	.swd = &cmsis_dap_swd_driver,
	.transports = cmsis_dap_transport,

	.execute_queue = cmsis_dap_execute_queue,
	.speed = cmsis_dap_speed,
	.speed_div = cmsis_dap_speed_div,
	.khz = cmsis_dap_khz,
	.init = cmsis_dap_init,
	.quit = cmsis_dap_quit,
};
