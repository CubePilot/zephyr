/*
 * Copyright (c) 2018 Karsten Koenig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mcp2517fd

#include <kernel.h>
#include <device.h>
#include <drivers/can/transceiver.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <sys/byteorder.h>

LOG_MODULE_REGISTER(can_mcp2517fd, 3);

#include "can_mcp2517fd.h"
#include "can_utils.h"


#define SP_IS_SET(inst) DT_INST_NODE_HAS_PROP(inst, sample_point) ||

/* Macro to exclude the sample point algorithm from compilation if not used
 * Without the macro, the algorithm would always waste ROM
 */
#define USE_SP_ALGO (DT_INST_FOREACH_STATUS_OKAY(SP_IS_SET) 0)

#define SP_AND_TIMING_NOT_SET(inst) \
	(!DT_INST_NODE_HAS_PROP(inst, sample_point) && \
	!(DT_INST_NODE_HAS_PROP(inst, prop_seg) && \
	DT_INST_NODE_HAS_PROP(inst, phase_seg1) && \
	DT_INST_NODE_HAS_PROP(inst, phase_seg2))) ||
#if DT_INST_FOREACH_STATUS_OKAY(SP_AND_TIMING_NOT_SET) 0
#error You must either set a sampling-point or timings (phase-seg* and prop-seg)
#endif
//! Look-up table for CRC calculation
const uint16_t crc16_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};

static uint16_t mcp2517_calculate_crc16(uint8_t* data, uint16_t size, uint16_t* crc_in)
{
	uint16_t crc;
	if (crc_in == NULL) {
    	crc = 0xFFFF;
	} else {
		crc = *crc_in;
	}
    uint8_t index;

    while (size-- != 0) {
        index = ((uint8_t*) &crc)[1] ^ *data++;
        crc = (crc << 8) ^ crc16_table[index];
    }

    return crc;
}

static int mcp2517_cmd_soft_reset(const struct device *dev)
{
	const struct mcp251xfd_config *dev_cfg = dev->config;
	uint16_t cmd = MCP251XFD_SPI_INSTRUCTION_RESET;

	const struct spi_buf tx_buf = {
		.buf = &cmd, .len = sizeof(cmd),
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf, .count = 1U
	};

    return spi_write_dt(&dev_cfg->bus, &tx);
}

// SPI Access Functions
static int mcp2517_cmd_read_words(const struct device *dev, uint16_t addr,
				uint32_t *words, uint8_t num_words)
{
	const struct mcp251xfd_config *dev_cfg = dev->config;

	uint8_t cmd_buf[] = { (MCP251XFD_SPI_INSTRUCTION_READ << 4) | (addr >> 8),
						  addr & 0xFF};

	struct spi_buf tx_buf[] = {
		{ .buf = cmd_buf, .len = sizeof(cmd_buf) },
		{ .buf = NULL, .len = num_words*sizeof(uint32_t) }
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf, .count = ARRAY_SIZE(tx_buf)
	};
	struct spi_buf rx_buf[] = {
		{ .buf = NULL, .len = sizeof(cmd_buf) },
		{ .buf = words, .len = num_words*sizeof(uint32_t) }
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf, .count = ARRAY_SIZE(rx_buf)
	};

	return spi_transceive_dt(&dev_cfg->bus, &tx, &rx);
}

static int mcp2517_cmd_write_words(const struct device *dev, uint16_t addr,
				 uint32_t *words, uint8_t num_words)
{
	const struct mcp251xfd_config *dev_cfg = dev->config;

	uint8_t cmd_buf[] = { (MCP251XFD_SPI_INSTRUCTION_WRITE << 4) | (addr >> 8),
						  addr & 0xFF};
	struct spi_buf tx_buf[] = {
		{ .buf = cmd_buf, .len = sizeof(cmd_buf) },
		{ .buf = words, .len = num_words*sizeof(uint32_t) }
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf, .count = ARRAY_SIZE(tx_buf)
	};

	return spi_write_dt(&dev_cfg->bus, &tx);
}

static int mcp2517_cmd_write_words_safe(const struct device *dev, uint16_t addr,
				uint8_t *buf_data, uint8_t buf_len)
{
	const struct mcp251xfd_config *dev_cfg = dev->config;

	uint8_t cmd_buf[] = { (MCP251XFD_SPI_INSTRUCTION_WRITE_CRC_SAFE << 4) | (addr >> 8),
						  addr & 0xFF};

    // Add CRC
    uint16_t crc = mcp2517_calculate_crc16(cmd_buf, sizeof(cmd_buf), NULL);
	crc = mcp2517_calculate_crc16(buf_data, buf_len, &crc);
	// convert crc to be16
	crc = sys_cpu_to_be16(crc);
	struct spi_buf tx_buf[] = {
		{ .buf = cmd_buf, .len = sizeof(cmd_buf) },
		{ .buf = buf_data, .len = buf_len },
		{ .buf = &crc, .len = sizeof(crc) },
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf, .count = ARRAY_SIZE(tx_buf)
	};
	return spi_write_dt(&dev_cfg->bus, &tx);
}

static int mcp2517_cmd_read_words_crc(const struct device *dev, uint8_t addr,
				bool fromram, uint8_t *words, uint8_t num_words)
{
	const struct mcp251xfd_config *dev_cfg = dev->config;
	uint8_t N = num_words*sizeof(uint32_t);
	uint8_t buf_len = num_words*sizeof(uint32_t);
	if (fromram) {
		N >>= 2;
	}

	uint8_t cmd_buf[] = { (MCP251XFD_SPI_INSTRUCTION_READ_CRC << 4) | (addr >> 8),
						  addr & 0xFF,
						  N};

	// read
	struct spi_buf tx_buf[] = {
		{ .buf = cmd_buf, .len = sizeof(cmd_buf) },
		{ .buf = NULL, .len = buf_len + 2 }
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf, .count = ARRAY_SIZE(tx_buf)
	};
	struct spi_buf rx_buf[] = {
		{ .buf = NULL, .len = sizeof(cmd_buf) },
		{ .buf = words, .len = buf_len + 2 }
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf, .count = ARRAY_SIZE(rx_buf)
	};
	int ret = spi_transceive_dt(&dev_cfg->bus, &tx, &rx);
	if (ret != 0) {
		return ret;
	}

	// check CRC
	uint16_t crc = mcp2517_calculate_crc16(cmd_buf, sizeof(cmd_buf), NULL);
	crc = mcp2517_calculate_crc16(words, buf_len, &crc);
	
	if (crc != ((words[buf_len] << 8) | words[buf_len + 1])) {
		LOG_ERR("CRC error");
		return -EIO;
	}
	// all good
	return 0;
}


static int mcp2517_get_mode_int(const struct device *dev, uint8_t *mode)
{
	uint32_t con;
	int ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_CON, &con, 1);
	if (ret < 0) {
		return ret;
	}
	*mode = (con & MCP251XFD_CON_OPMOD_MASK) >> 21;
	LOG_INF("Control: 0x%08x mode: %d", con, *mode);
	return 0;
}

static int mcp2517_set_mode_int(const struct device *dev, uint8_t mode)
{
	uint32_t conreg;
	uint8_t curr_mode;
	int ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_CON, &conreg, 1);
	if (ret < 0) {
		return ret;
	}
	conreg = (conreg & ~(MCP251XFD_CON_REQOP_MASK)) | (mode << 24);
	ret = mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_CON, &conreg, 1);
	if (ret < 0) {
		return ret;
	}
	LOG_INF("Waiting for mode change");
	// wait for mode change
	while (1)
	{
		if (mcp2517_get_mode_int(dev, &curr_mode) < 0) {
			LOG_ERR("Failed to get mode");
			return -EIO;
		}
		if (mode == curr_mode) {
			return 0;
		}
		k_msleep(3);
	}
	
}

static int mcp2517_enable_ecc(const struct device *dev)
{
	uint32_t reg = 0;
	// read ECC reg
	int ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_ECCCON, &reg, 1);
	if (ret != 0) {
		return ret;
	}
	// set ECC enable bit
	reg = reg | MCP251XFD_ECCCON_ECCEN;
	// write ECC reg
	return mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_ECCCON, &reg, 1);
}

// convert zcanframe to mcp2517frame
static void mcp2517_convert_zcanframe_to_mcp2517frame(const struct zcan_frame
						      *source, struct mcp2517frame *target)
{
	memset(target, 0, sizeof(struct mcp2517frame));
	if (source->id_type == CAN_STANDARD_IDENTIFIER) {
		target->id = source->id & CAN_STD_ID_MASK;
	} else {
		target->id = ((source->id >> 18) & CAN_STD_ID_MASK) | ((source->id & 0x3FFFF) << 11);
	}

	target->ctrl.DLC = source->dlc;
	target->ctrl.IDE = source->id_type;
	target->ctrl.RTR = source->rtr;
	target->ctrl.BRS = source->brs;
	target->ctrl.FDF = source->fd;

	memcpy(target->data, source->data, can_dlc_to_bytes(source->dlc));
	return;
}

// convert mcp2517frame to zcanframe
static void mcp2517_convert_mcp2517frame_to_zcanframe(const struct mcp2517frame *source,
						      struct zcan_frame *target)
{
	memset(target, 0, sizeof(struct zcan_frame));
	if (source->ctrl.IDE == CAN_STANDARD_IDENTIFIER) {
		target->id = source->id & CAN_STD_ID_MASK;
	} else {
		target->id = ((source->id >> 11) & 0x3FFFF) | ((source->id & CAN_STD_ID_MASK) << 18);
	}

	target->dlc = source->ctrl.DLC;
	target->id_type = source->ctrl.IDE;
	target->rtr = source->ctrl.RTR;
	target->brs = source->ctrl.BRS;
	target->fd = source->ctrl.FDF;

	memcpy(target->data, source->data, can_dlc_to_bytes(source->ctrl.DLC));
	return;
}

// read fifo status
static int mcp2517_get_fifo_status(const struct device *dev, uint8_t fifo_id, uint32_t *status)
{
	return mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_FIFOSTA(fifo_id), status, 1);
}

// read recieve fifo
static int mcp2517_read_rx_fifo(const struct device *dev,
				struct zcan_frame *frame)
{
	// get FIFO status
	uint32_t fifo_status;
	int ret = mcp2517_get_fifo_status(dev, MCP251XFD_RX_FIFO_INDEX, &fifo_status);
	if (ret != 0) {
		LOG_ERR("Failed to get FIFO status");
		return ret;
	}
	if (!(fifo_status & MCP251XFD_FIFOSTA_TFNRFNIF)) {
		// rx fifo empty
		return 0;
	}
	// get rx fifo address
	uint32_t rx_fifo_addr;
	ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_FIFOUA(MCP251XFD_RX_FIFO_INDEX), &rx_fifo_addr, 1);

	// adjust rx_fifo_addr to 4 byte boundary and read size
	uint8_t rx_fifo_addr_adj = rx_fifo_addr & 0x3;
	rx_fifo_addr &= 0xFFFFFFFC;

	rx_fifo_addr += MCP251XFD_RAM_START;

	uint8_t read_buf[sizeof(struct mcp2517frame)+4];

	// read rx fifo, with 3 retries
	for (int i = 0; i < 3; i++) {
		ret = mcp2517_cmd_read_words(dev, rx_fifo_addr, read_buf, sizeof(read_buf)/4);
		if (ret == 0) {
			break;
		}
	}

	// increment rx fifo
	uint32_t fifo_ctrl;
	ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_FIFOCON(MCP251XFD_RX_FIFO_INDEX), &fifo_ctrl, 1);
	fifo_ctrl |= MCP251XFD_FIFOCON_UINC;
	ret = mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_FIFOCON(MCP251XFD_RX_FIFO_INDEX), &fifo_ctrl, 1);

	struct mcp2517frame *mcp2517frame = (struct mcp2517frame *)&read_buf[rx_fifo_addr_adj];
	mcp2517_convert_mcp2517frame_to_zcanframe(mcp2517frame, frame);
	return 0;
}

// write transmit fifo
static int32_t mcp2517_write_tx_fifo(const struct device *dev,
				 const struct zcan_frame *frame)
{
	// get FIFO status
	uint32_t fifo_status;
	int ret = mcp2517_get_fifo_status(dev, MCP251XFD_TX_FIFO_INDEX, &fifo_status);
	if (ret != 0) {
		LOG_ERR("Failed to get FIFO status");
		return ret;
	}
	if (!(fifo_status & MCP251XFD_FIFOSTA_TFNRFNIF)) {
		// tx fifo full
		LOG_ERR("TX FIFO full");
		return -EBUSY;
	}
	// LOG_INF("FIFO status: 0x%08x", fifo_status);


	// // Print Current TX IF
	// uint32_t diag0, diag1, tec, irq;
	// ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_INT, &irq, 1);
	// if (ret != 0) {
	// 	LOG_ERR("Failed to read INT");
	// 	return ret;
	// }
	// LOG_INF("INT: 0x%08x", irq);

	// ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_TREC, &tec, 1);
	// if (ret != 0) {
	// 	LOG_ERR("Failed to read TEC");
	// 	return ret;
	// }
	// LOG_INF("TEC: %d", tec);
	// ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_BDIAG0, &diag0, 1);
	// if (ret != 0) {
	// 	LOG_ERR("Failed to get INT");
	// 	return ret;
	// }
	// LOG_INF("DIAG0: %x", diag0);

	// ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_BDIAG1, &diag1, 1);
	// if (ret != 0) {
	// 	LOG_ERR("Failed to get INT");
	// 	return ret;
	// }
	// LOG_INF("DIAG1: %x", diag1);

	// get tx fifo address
	uint32_t tx_fifo_addr;
	ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_FIFOUA(MCP251XFD_TX_FIFO_INDEX), &tx_fifo_addr, 1);

	tx_fifo_addr += MCP251XFD_RAM_START;

	// adjust numwords to 4 byte boundary
	uint8_t num_words = 2 + ((can_dlc_to_bytes(frame->dlc) + 3) / 4);


	// set frame
	struct mcp2517frame mcp2517frame;
	mcp2517_convert_zcanframe_to_mcp2517frame(frame, &mcp2517frame);

	// write tx fifo with crc, with 3 retries
	for (int i = 0; i < 3; i++) {
		ret = mcp2517_cmd_write_words(dev, tx_fifo_addr, (uint32_t*)&mcp2517frame, num_words);
		if (ret == 0) {
			// // read same bytes back to verify
			// struct mcp2517frame mcp2517frame_verify;
			// ret = mcp2517_cmd_read_words(dev, tx_fifo_addr, (uint32_t*)&mcp2517frame_verify, num_words);
			// char buf[256], buf2[256];
			// uint8_t buflen = 0;
			// LOG_INF("Frame %lx Wrote %d words to Addr %lx:", frame->id, num_words, tx_fifo_addr);
			// for (int i = 0; i < num_words; i++) {
			// 	snprintf(&buf[buflen], 10, " %08x", ((uint32_t*)&mcp2517frame)[i]);
			// 	snprintf(&buf2[buflen], 10, " %08x", ((uint32_t*)&mcp2517frame_verify)[i]);
			// 	buflen += 9;
			// }
			// buf[buflen] = 0;
			// LOG_INF("%s", buf);
			// buf2[buflen] = 0;
			// LOG_INF("%s", buf2);
			break;
		}
	}

	// read fifo control
	uint32_t fifo_ctrl;
	ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_FIFOCON(MCP251XFD_TX_FIFO_INDEX), &fifo_ctrl, 1);
	// set tx fifo request
	fifo_ctrl |= MCP251XFD_FIFOCON_UINC | MCP251XFD_FIFOCON_TXREQ;
	ret = mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_FIFOCON(MCP251XFD_TX_FIFO_INDEX), &fifo_ctrl, 1);

	if (ret < 0) {
		LOG_ERR("Failed to write FIFO control");
		return ret;
	}
	return tx_fifo_addr;
}

static int mcp2517_convert_plsize_to_mcp_plsize(uint8_t plsize)
{
	switch (plsize) {
	case CAN_PLSIZE_8:
		return MCP251XFD_FIFOCON_PLSIZE_8;
	case CAN_PLSIZE_12:
		return MCP251XFD_FIFOCON_PLSIZE_12;
	case CAN_PLSIZE_16:
		return MCP251XFD_FIFOCON_PLSIZE_16;
	case CAN_PLSIZE_20:
		return MCP251XFD_FIFOCON_PLSIZE_20;
	case CAN_PLSIZE_24:
		return MCP251XFD_FIFOCON_PLSIZE_24;
	case CAN_PLSIZE_32:
		return MCP251XFD_FIFOCON_PLSIZE_32;
	case CAN_PLSIZE_48:
		return MCP251XFD_FIFOCON_PLSIZE_48;
	case CAN_PLSIZE_64:
		return MCP251XFD_FIFOCON_PLSIZE_64;
	default:
		return -EINVAL;
	}
}

static int mcp2517_setup_tx_fifo(const struct device *dev)
{
	const struct mcp251xfd_config *dev_cfg = dev->config;

	// disable TXQ
	uint32_t con = 0;
	int ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_CON, &con, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read CON");
		return ret;
	}
	if (con & MCP251XFD_CON_TXQEN) {
		con &= ~MCP251XFD_CON_TXQEN;
		ret = mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_CON, &con, 1);
		if (ret < 0) {
			LOG_ERR("Failed to write CON");
			return ret;
		}	
	}

	uint32_t tx_fifo_ctrl;
	ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_FIFOCON(MCP251XFD_TX_FIFO_INDEX), &tx_fifo_ctrl, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read FIFO control");
		return ret;
	}
	int plsize = mcp2517_convert_plsize_to_mcp_plsize(dev_cfg->plsize);
	if (plsize < 0) {
		LOG_ERR("Invalid payload size");
		return plsize;
	}
	tx_fifo_ctrl = (tx_fifo_ctrl & ~MCP251XFD_FIFOCON_PLSIZE_MASK) | plsize << 29;
	tx_fifo_ctrl |= MCP251XFD_FIFOCON_TXEN;
	// set fifo size
	tx_fifo_ctrl = (tx_fifo_ctrl & ~MCP251XFD_FIFOCON_FSIZE_MASK) | (MCP2517FD_TX_FIFO_SIZE << 24);

	ret = mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_FIFOCON(MCP251XFD_TX_FIFO_INDEX), &tx_fifo_ctrl, 1);
	if (ret < 0) {
		LOG_ERR("Failed to write FIFO control");
		return ret;
	}
	// setup tx event fifo
	uint32_t tx_event_fifo_ctrl;
	ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_TEFCON, &tx_event_fifo_ctrl, 1);
	if (ret != 0) {
		LOG_ERR("Failed to read TX event fifo");
		return ret;
	}
	tx_event_fifo_ctrl = (tx_event_fifo_ctrl & ~(MCP251XFD_TEFCON_FSIZE_MASK)) | MCP2517FD_TXEVENT_FIFO_SIZE << 24;

	tx_event_fifo_ctrl = tx_event_fifo_ctrl | MCP251XFD_TEFCON_TEFNEIE;

	// write register
	ret = mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_TEFCON, &tx_event_fifo_ctrl, 1);
	if (ret != 0) {
		LOG_ERR("Failed to setup TX event fifo");
		return ret;
	}

	return ret;
}

static int mcp2517_setup_rx_fifo(const struct device *dev)
{
	const struct mcp251xfd_config *dev_cfg = dev->config;

	uint32_t rx_fifo_ctrl;
	int ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_FIFOCON(MCP251XFD_RX_FIFO_INDEX),
						 &rx_fifo_ctrl, 1);
	if (ret < 0) {
		return ret;
	}
	// enable rx
	rx_fifo_ctrl = rx_fifo_ctrl & ~MCP251XFD_FIFOCON_TXEN;
	int plsize = mcp2517_convert_plsize_to_mcp_plsize(dev_cfg->plsize);
	if (plsize < 0) {
		LOG_ERR("Invalid payload size");
		return plsize;
	}
	// setup payload size
	rx_fifo_ctrl = (rx_fifo_ctrl & ~MCP251XFD_FIFOCON_PLSIZE_MASK) | plsize << 29;

	// setup fifo size
	rx_fifo_ctrl = (rx_fifo_ctrl & ~(MCP251XFD_FIFOCON_FSIZE_MASK)) | MCP2517FD_RX_FIFO_SIZE << 24;
	// enable interrupt
	rx_fifo_ctrl = rx_fifo_ctrl | MCP251XFD_FIFOCON_RXOVIE | MCP251XFD_FIFOCON_TFNRFNIE;
	ret = mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_FIFOCON(MCP251XFD_RX_FIFO_INDEX), &rx_fifo_ctrl, 1);
	if (ret < 0) {
		return ret;
	}
	// enable rx fifo interrupt
	uint32_t int_enable;
	ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_INT, &int_enable, 1);
	if (ret < 0) {
		return ret;
	}
	int_enable |= MCP251XFD_INT_RXOVIE | MCP251XFD_INT_RXIE;
	ret = mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_INT, &int_enable, 1);

	// enable all messages on Filter 0
	uint32_t flt_obj = 0;
	ret = mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_FLTOBJ(MCP251XFD_RX_FIFO_INDEX), &flt_obj, 1);
	if (ret < 0) {
		return ret;
	}
	uint32_t flt_mask = 0;
	ret = mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_FLTMASK(MCP251XFD_RX_FIFO_INDEX), &flt_mask, 1);
	if (ret < 0) {
		return ret;
	}
	// enable filter
	uint32_t rx_filter_ctrl;
	ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_FLTCON(0), &rx_filter_ctrl, 1);
	if (ret < 0) {
		return ret;
	}
	rx_filter_ctrl = (rx_fifo_ctrl & ~MCP251XFD_FLTCON_FLT_MASK(0)) |
					MCP251XFD_FLTCON_FBP(0, MCP251XFD_RX_FIFO_INDEX + 1) |
					MCP251XFD_FLTCON_FLTEN(0);
	ret = mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_FLTCON(0), &rx_filter_ctrl, 1);
	if (ret < 0) {
		return ret;
	}
	return ret;
}

static int mcp2517_send(const struct device *dev,
			const struct zcan_frame *frame,
			k_timeout_t timeout, can_tx_callback_t callback,
			void *user_data)
{
	struct mcp251xfd_data *dev_data = dev->data;

	if (k_sem_take(&dev_data->tx_sem, timeout) != 0) {
		return -EAGAIN;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	// send to tx fifo
	int32_t ret = mcp2517_write_tx_fifo(dev, frame);
	if (ret < 0) {
		LOG_ERR("Failed to write tx fifo");
		k_mutex_unlock(&dev_data->mutex);
		k_sem_give(&dev_data->tx_sem);
		return ret;
	}
	ret = 0;
	k_mutex_unlock(&dev_data->mutex);

	// find free tx callback buf
	bool found = false;
	for (int i = 0; i < MCP2517FD_TX_FIFO_SIZE; i++) {
		if (dev_data->tx_cb[i].tx_fifo_addr == ret) {
			dev_data->tx_cb[i].cb = callback;
			dev_data->tx_cb[i].cb_arg = user_data;
			dev_data->tx_cb[i].tx_fifo_addr = ret;
			found = true;
			break;
		}
	}
	if (!found) {
		// find free tx callback buf
		for (int i = 0; i < MCP2517FD_TX_FIFO_SIZE; i++) {
			if (dev_data->tx_cb[i].tx_fifo_addr == 0) {
				dev_data->tx_cb[i].cb = callback;
				dev_data->tx_cb[i].cb_arg = user_data;
				dev_data->tx_cb[i].tx_fifo_addr = ret;
				found = true;
				break;
			}
		}
	}
	if (!found) {
		LOG_ERR("Failed to find free tx callback buf");
		k_mutex_unlock(&dev_data->mutex);
		k_sem_give(&dev_data->tx_sem);
		return -ENOMEM;
	}
	k_sem_give(&dev_data->tx_sem);
	return ret;
}

static int mcp2517_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct mcp251xfd_config *dev_cfg = dev->config;

	*rate = dev_cfg->osc_freq;
	return 0;
}

static int mcp2517_set_timing(const struct device *dev,
			      const struct can_timing *timing,
			      const struct can_timing *timing_data)
{
	// set nominal timing
	uint32_t ntreg = 0;
	ntreg = (ntreg & ~(MCP251XFD_NBTCFG_BRP_MASK)) | ((timing->prescaler - 1) << 24);
	ntreg = (ntreg & ~(MCP251XFD_NBTCFG_TSEG1_MASK)) | ((timing->prop_seg + timing->phase_seg1 - 1) << 16);
	ntreg = (ntreg & ~(MCP251XFD_NBTCFG_TSEG2_MASK)) | ((timing->phase_seg2 - 1) << 8);
	ntreg = (ntreg & ~(MCP251XFD_NBTCFG_SJW_MASK)) | (timing->sjw - 1);
	int ret = mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_NBTCFG, &ntreg, 1);
	if (ret < 0) {
		return ret;
	}
	LOG_INF("ntreg: 0x%08x sjw: %d tseg1: %d tseg2: %d brp: %d", ntreg, timing->sjw, timing->prop_seg + timing->phase_seg1, timing->phase_seg2, timing->prescaler);
	// set data timing
	if (timing_data != NULL) { 
		uint32_t tdcoreg = 0;
		uint32_t dtreg = 0;
		dtreg = (dtreg & ~(MCP251XFD_DBTCFG_TSEG2_MASK)) | ((timing_data->phase_seg2 - 1) << 16);
		dtreg = (dtreg & ~(MCP251XFD_DBTCFG_TSEG1_MASK)) | ((timing_data->prop_seg + timing_data->phase_seg1 - 1) << 8);
		dtreg = (dtreg & ~(MCP251XFD_DBTCFG_SJW_MASK)) | (timing_data->sjw - 1);
		dtreg = (dtreg & ~(MCP251XFD_DBTCFG_BRP_MASK)) | ((timing_data->prescaler - 1) << 24);
		ret = mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_DBTCFG, &dtreg, 1);
		if (ret < 0) {
			return ret;
		}
		// set transmit delay compensation
		int16_t tdco = timing_data->prescaler * (timing_data->prop_seg + timing_data->phase_seg1);
		tdcoreg = MCP251XFD_TDC_TDCMOD_AUTO << 16;
		tdcoreg = (tdcoreg & ~(MCP251XFD_TDC_TDCO_MASK)) | (tdco & 0x7F);
	
		return mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_TDC, &tdcoreg, 1);
	}
	return ret;
}

static int mcp2517_wait_for_osc(const struct device *dev, bool pll_enable)
{
	uint32_t osc;
	int ret = 0;
	ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_OSC, &osc, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read osc");
		return ret;
	}
	if (pll_enable) {
		osc |= MCP251XFD_OSC_PLLEN;
	} else {
		osc &= ~MCP251XFD_OSC_PLLEN;
	}
	ret = mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_OSC, &osc, 1);
	if (ret < 0) {
		LOG_ERR("Failed to write osc");
		return ret;
	}
	uint8_t retries = MCP2517FD_NUM_SPI_RETRIES;
	while (retries--) {
		ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_OSC, &osc, 1);
		if (ret < 0) {
			LOG_ERR("Failed to read osc");
			return ret;
		}
		if (osc & MCP251XFD_OSC_OSCRDY && ((osc & MCP251XFD_OSC_PLLRDY) || !pll_enable)) {
			return 0;
		}
		k_msleep(3);
	}
	return -EIO;
}

static int mcp2517_set_mode(const struct device *dev, enum can_mode mode)
{
	const struct mcp251xfd_config *dev_cfg = dev->config;
	struct mcp251xfd_data *dev_data = dev->data;
	uint8_t mcp2517_mode;
	int ret;

	switch (mode) {
	case CAN_NORMAL_MODE:
		mcp2517_mode = MCP251XFD_CON_MODE_CAN2_0;
		break;
	case CAN_SILENT_MODE:
		mcp2517_mode = MCP251XFD_CON_MODE_LISTENONLY;
		break;
	case CAN_LOOPBACK_MODE:
		mcp2517_mode = MCP251XFD_CON_MODE_EXT_LOOPBACK;
		break;
	default:
		LOG_ERR("Unsupported CAN Mode %u", mode);
		return -ENOTSUP;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	if (dev_cfg->phy != NULL) {
		ret = can_transceiver_enable(dev_cfg->phy);
		if (ret != 0) {
			LOG_ERR("failed to enable CAN transceiver (err %d)", ret);
			goto done;
		}
	}

	ret = mcp2517_set_mode_int(dev, mcp2517_mode);
	if (ret < 0) {
		LOG_ERR("Failed to set the mode [%d]", ret);

		if (dev_cfg->phy != NULL) {
			/* Attempt to disable the CAN transceiver in case of error */
			(void)can_transceiver_disable(dev_cfg->phy);
		}
	}

done:
	k_mutex_unlock(&dev_data->mutex);
	return ret;
}

static int mcp2517_add_rx_filter(const struct device *dev,
				 can_rx_callback_t rx_cb,
				 void *cb_arg,
				 const struct zcan_filter *filter)
{
	struct mcp251xfd_data *dev_data = dev->data;
	int filter_id = 0;

	__ASSERT(rx_cb != NULL, "response_ptr can not be null");

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	/* find free filter */
	while ((BIT(filter_id) & dev_data->filter_usage)
	       && (filter_id < CONFIG_CAN_MAX_FILTER)) {
		filter_id++;
	}

	/* setup filter */
	if (filter_id < CONFIG_CAN_MAX_FILTER) {
		dev_data->filter_usage |= BIT(filter_id);

		dev_data->filter[filter_id] = *filter;
		dev_data->rx_cb[filter_id] = rx_cb;
		dev_data->cb_arg[filter_id] = cb_arg;

	} else {
		filter_id = -ENOSPC;
	}

	k_mutex_unlock(&dev_data->mutex);

	return filter_id;
}

static void mcp2517_remove_rx_filter(const struct device *dev, int filter_id)
{
	struct mcp251xfd_data *dev_data = dev->data;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);
	dev_data->filter_usage &= ~BIT(filter_id);
	k_mutex_unlock(&dev_data->mutex);
}

static void mcp2517_set_state_change_callback(const struct device *dev,
					      can_state_change_callback_t cb,
					      void *user_data)
{
	struct mcp251xfd_data *dev_data = dev->data;

	dev_data->state_change_cb = cb;
	dev_data->state_change_cb_data = user_data;
}

static void mcp2517_rx_filter(const struct device *dev,
			      struct zcan_frame *frame)
{
	struct mcp251xfd_data *dev_data = dev->data;
	uint8_t filter_id = 0U;
	can_rx_callback_t callback;
	struct zcan_frame tmp_frame;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	for (; filter_id < CONFIG_CAN_MAX_FILTER; filter_id++) {
		if (!(BIT(filter_id) & dev_data->filter_usage)) {
			continue; /* filter slot empty */
		}

		if (!can_utils_filter_match(frame,
					    &dev_data->filter[filter_id])) {
			continue; /* filter did not match */
		}

		callback = dev_data->rx_cb[filter_id];
		/*Make a temporary copy in case the user modifies the message*/
		tmp_frame = *frame;

		callback(dev, &tmp_frame, dev_data->cb_arg[filter_id]);
	}

	k_mutex_unlock(&dev_data->mutex);
}


static int mcp2517_get_state(const struct device *dev, enum can_state *state,
			     struct can_bus_err_cnt *err_cnt)
{
	// read TREC reg
	uint32_t trec;
	int ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_TREC, &trec, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read error register [%d]", ret);
		return -EIO;
	}

	if (state != NULL) {
		if (trec & MCP251XFD_TREC_TXBO) {
			*state = CAN_BUS_OFF;
		} else if ((trec & MCP251XFD_TREC_TXBP) || (trec & MCP251XFD_TREC_RXBP)) {
			*state = CAN_ERROR_PASSIVE;
		} else if ((trec & MCP251XFD_TREC_EWARN) || (trec & MCP251XFD_TREC_TXWARN) || (trec & MCP251XFD_TREC_RXWARN)) {
			*state = CAN_ERROR_WARNING;
		} else {
			*state = CAN_ERROR_ACTIVE;
		}
	}

	if (err_cnt != NULL) {
		err_cnt->tx_err_cnt = (trec & MCP251XFD_TREC_TEC_MASK) >> 8;
		err_cnt->rx_err_cnt = (trec & MCP251XFD_TREC_REC_MASK);
	}
	return ret;
}

#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
static int mcp2517_recover(const struct device *dev, k_timeout_t timeout)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(timeout);

	return -ENOTSUP;
}
#endif


static int mcp2517_get_max_filters(const struct device *dev, enum can_ide id_type)
{
	ARG_UNUSED(id_type);

	return CONFIG_CAN_MAX_FILTER;
}

static int mcp2517_get_max_bitrate(const struct device *dev, uint32_t *max_bitrate)
{
	const struct mcp251xfd_config *dev_cfg = dev->config;

	*max_bitrate = dev_cfg->max_bitrate;

	return 0;
}


static void mcp2517_rx(const struct device *dev)
{
	struct zcan_frame frame;

	/* Fetch rx buffer */
	mcp2517_read_rx_fifo(dev, &frame);
	mcp2517_rx_filter(dev, &frame);
}

static void mcp2517_handle_errors(const struct device *dev)
{
	struct mcp251xfd_data *dev_data = dev->data;
	can_state_change_callback_t state_change_cb = dev_data->state_change_cb;
	void *state_change_cb_data = dev_data->state_change_cb_data;
	enum can_state state;
	struct can_bus_err_cnt err_cnt;
	int err;

	err = mcp2517_get_state(dev, &state, state_change_cb ? &err_cnt : NULL);
	if (err != 0) {
		LOG_ERR("Failed to get CAN controller state [%d]", err);
		return;
	}

	if (state_change_cb && dev_data->old_state != state) {
		dev_data->old_state = state;
		state_change_cb(dev, state, err_cnt, state_change_cb_data);
	}
}

static void mcp2517_handle_interrupts(const struct device *dev)
{
	const struct mcp251xfd_config *dev_cfg = dev->config;
	int ret;
	uint32_t canintf;
	uint32_t rxif;
	/* Loop until INT pin is inactive (all interrupt flags handled) */
	while (1) {
		ret = mcp2517_cmd_read_words(dev, MCP251XFD_ADDR_INT, &canintf, 1);
		if (ret != 0) {
			LOG_ERR("Couldn't read INTF register %d", ret);
			continue;
		}
		if ((canintf & 0xFFFF0000) == 0) {
			/* No interrupt flags set */
			break;
		}

		if (canintf & MCP251XFD_INT_RXIF || canintf & MCP251XFD_INT_RXOVIF) {
			mcp2517_rx(dev);

			/* RX0IF flag cleared automatically during read */
			canintf &= ~MCP251XFD_INT_RXIF;
			canintf &= ~MCP251XFD_INT_RXOVIF;
		}

		if (canintf & MCP251XFD_INT_CERRIF) {
			mcp2517_handle_errors(dev);
		}

		if ((canintf & 0xFFFF) != 0) {
			canintf &= 0xFFFF0000;
			/* Clear remaining flags */
			mcp2517_cmd_write_words(dev, MCP251XFD_ADDR_INT, &canintf, 1);
		}

		/* Break from loop if INT pin is inactive */
		ret = gpio_pin_get_dt(&dev_cfg->int_gpio);
		if (ret < 0) {
			LOG_ERR("Couldn't read INT pin");
		} else if (ret == 0) {
			/* All interrupt flags handled */
			break;
		}
	}
}

static void mcp2517_int_thread(const struct device *dev)
{
	struct mcp251xfd_data *dev_data = dev->data;

	while (1) {
		k_sem_take(&dev_data->int_sem, K_FOREVER);
		mcp2517_handle_interrupts(dev);
	}
}

static void mcp2517_int_gpio_callback(const struct device *dev,
				      struct gpio_callback *cb, uint32_t pins)
{
	struct mcp251xfd_data *dev_data =
		CONTAINER_OF(cb, struct mcp251xfd_data, int_gpio_cb);

	k_sem_give(&dev_data->int_sem);
}


static const struct can_driver_api can_api_funcs = {
	.set_timing = mcp2517_set_timing,
	.set_mode = mcp2517_set_mode,
	.send = mcp2517_send,
	.add_rx_filter = mcp2517_add_rx_filter,
	.remove_rx_filter = mcp2517_remove_rx_filter,
	.get_state = mcp2517_get_state,
#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	.recover = mcp2517_recover,
#endif
	.set_state_change_callback = mcp2517_set_state_change_callback,
	.get_core_clock = mcp2517_get_core_clock,
	.get_max_filters = mcp2517_get_max_filters,
	.get_max_bitrate = mcp2517_get_max_bitrate,
	.timing_min = {
		.sjw = 0x1,
		.prop_seg = 0x01,
		.phase_seg1 = 0x01,
		.phase_seg2 = 0x01,
		.prescaler = 0x01
	},
	.timing_max = {
		.sjw = 0x10,
		.prop_seg = 0x10,
		.phase_seg1 = 0x10,
		.phase_seg2 = 0x10,
		.prescaler = 0x100
	},
#ifdef CONFIG_CAN_FD_MODE
	.timing_min_data = {
		.sjw = 0x1,
		.prop_seg = 0x01,
		.phase_seg1 = 0x01,
		.phase_seg2 = 0x01,
		.prescaler = 0x01
	},
	.timing_max_data = {
		.sjw = 0x10,
		.prop_seg = 0x10,
		.phase_seg1 = 0x10,
		.phase_seg2 = 0x10,
		.prescaler = 0x100
	}
#endif
};


static int mcp2517_init(const struct device *dev)
{
	struct mcp251xfd_config *dev_cfg = dev->config;
	struct mcp251xfd_data *dev_data = dev->data;
	struct can_timing timing
#ifdef CONFIG_CAN_FD_MODE
		, timing_data	/* Data phase timing */
#endif
		;
	int ret;
	int i;
	uint32_t max_spi_freq;

	k_sem_init(&dev_data->int_sem, 0, 1);
	k_mutex_init(&dev_data->mutex);
	k_sem_init(&dev_data->tx_sem, MCP2517FD_TX_FIFO_SIZE, MCP2517FD_TX_FIFO_SIZE);

	for (i = 0; i < MCP2517FD_TX_FIFO_SIZE; i++) {
		k_sem_init(&dev_data->tx_cb[i].sem, 0, 1);
		dev_data->tx_cb[i].cb = NULL;
	}

	if (dev_cfg->phy != NULL) {
		if (!device_is_ready(dev_cfg->phy)) {
			LOG_ERR("CAN transceiver not ready");
			return -ENODEV;
		}
	}

	if (!spi_is_ready(&dev_cfg->bus)) {
		LOG_ERR("SPI bus %s not ready", dev_cfg->bus.bus->name);
		return -ENODEV;
	}

	if (dev_cfg->pll_enable) {
		if (dev_cfg->osc_freq > 4000000) {
			LOG_ERR("Oscillator frequency too high for pll");
			return -EINVAL;
		}
		dev_data->osc_freq = dev_cfg->osc_freq * 10; // 10x multiplier
	} else {
		dev_data->osc_freq = dev_cfg->osc_freq;
	}

	// record user defined spi frequency for later use
	max_spi_freq = dev_cfg->bus.config.frequency;
	if (max_spi_freq > (dev_data->osc_freq/2)) {
		LOG_WRN("SPI frequency too high, reducing to %d", dev_data->osc_freq/2);
		max_spi_freq = dev_data->osc_freq/2;
	}

	// set spi to 1MHz for init
	dev_cfg->bus.config.frequency = 1000000;

	// wait for OSCRDY PLLRDY
	if (mcp2517_wait_for_osc(dev, dev_cfg->pll_enable) < 0) {
		LOG_ERR("failed to wait for Osc ready (err %d)", ret);
		return -EIO;
	}

	/* Reset MCP2517 */
	if (mcp2517_cmd_soft_reset(dev) < 0) {
		LOG_ERR("Soft-reset failed");
		return -EIO;
	}

	// reset spi frequency to user defined value
	dev_cfg->bus.config.frequency = max_spi_freq;

	// clear ram
	uint32_t val = 0;
	for (uint32_t i = 0; i < MCP251XFD_RAM_SIZE; i += 4) {
		if (mcp2517_cmd_write_words(dev, MCP251XFD_RAM_START + i, &val, 1) < 0) {
			LOG_ERR("failed to clear ram (err %d)", ret);
			return -EIO;
		}
	}
	// check ram cleared
	for (uint32_t i = 0; i < MCP251XFD_RAM_SIZE; i += 4) {
		mcp2517_cmd_read_words(dev, MCP251XFD_RAM_START + i, &val, 1);
		if (ret < 0) {
			LOG_ERR("failed to check ram cleared (err %d)", ret);
			return -EIO;
		}
		if (val != 0) {
			LOG_ERR("RAM not cleared");
			return -EIO;
		}
	}

	// check in configuration mode
	uint8_t curr_mode;
	if (mcp2517_get_mode_int(dev, &curr_mode) < 0) {
		LOG_ERR("failed to get mode (err %d)", ret);
		return -EIO;
	}
	if (curr_mode != MCP251XFD_CON_MODE_CONFIG) {
		LOG_ERR("not in configuration mode after reset");
		return -EIO;
	}

	// enable ecc
	if (mcp2517_enable_ecc(dev) < 0) {
		LOG_ERR("Enable ECC failed");
		return -EIO;
	}

	/* Initialize interrupt handling  */
	if (!device_is_ready(dev_cfg->int_gpio.port)) {
		LOG_ERR("Interrupt GPIO port not ready");
		return -ENODEV;
	}

	if (gpio_pin_configure_dt(&dev_cfg->int_gpio, GPIO_INPUT)) {
		LOG_ERR("Unable to configure interrupt GPIO");
		return -EINVAL;
	}

	gpio_init_callback(&(dev_data->int_gpio_cb), mcp2517_int_gpio_callback,
			   BIT(dev_cfg->int_gpio.pin));

	if (gpio_add_callback(dev_cfg->int_gpio.port, &(dev_data->int_gpio_cb))) {
		return -EINVAL;
	}

	if (gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio,
					    GPIO_INT_EDGE_TO_ACTIVE)) {
		return -EINVAL;
	}

	k_thread_create(&dev_data->int_thread, dev_data->int_thread_stack,
			dev_cfg->int_thread_stack_size,
			(k_thread_entry_t) mcp2517_int_thread, (void *)dev,
			NULL, NULL, K_PRIO_COOP(dev_cfg->int_thread_priority),
			0, K_NO_WAIT);

	(void)memset(dev_data->rx_cb, 0, sizeof(dev_data->rx_cb));
	(void)memset(dev_data->filter, 0, sizeof(dev_data->filter));
	dev_data->old_state = CAN_ERROR_ACTIVE;

	// timing nominal
	timing.sjw = dev_cfg->tq_sjw;
	if (dev_cfg->sample_point && USE_SP_ALGO) {
		ret = can_calc_timing(dev, &timing, dev_cfg->bus_speed,
				      dev_cfg->sample_point);
		if (ret == -EINVAL) {
			LOG_ERR("Can't find timing for given param");
			return -EIO;
		}
		LOG_DBG("Presc: %d, BS1: %d, BS2: %d",
			timing.prescaler, timing.phase_seg1, timing.phase_seg2);
		LOG_DBG("Sample-point err : %d", ret);
	} else {
		timing.prop_seg = dev_cfg->tq_prop;
		timing.phase_seg1 = dev_cfg->tq_bs1;
		timing.phase_seg2 = dev_cfg->tq_bs2;
		ret = can_calc_prescaler(dev, &timing, dev_cfg->bus_speed);
		if (ret) {
			LOG_WRN("Bitrate error: %d", ret);
		}
	}

	// timing data
#ifdef CONFIG_CAN_FD_MODE
	if (dev_cfg->bus_speed_data != 0) {
		timing_data.sjw = dev_cfg->tq_sjw_data;
		if (dev_cfg->sample_point && USE_SP_ALGO) {
			ret = can_calc_timing(dev, &timing_data, dev_cfg->bus_speed_data,
						dev_cfg->sample_point_data);
			if (ret == -EINVAL) {
				LOG_ERR("Can't find timing_data for given param");
				return -EIO;
			}
			LOG_DBG("Presc: %d, BS1: %d, BS2: %d",
				timing_data.prescaler, timing_data.phase_seg1, timing_data.phase_seg2);
			LOG_DBG("Sample-point err : %d", ret);
		} else {
			timing_data.prop_seg = dev_cfg->tq_prop_data;
			timing_data.phase_seg1 = dev_cfg->tq_bs1_data;
			timing_data.phase_seg2 = dev_cfg->tq_bs2_data;
			ret = can_calc_prescaler(dev, &timing_data, dev_cfg->bus_speed_data);
			if (ret) {
				LOG_WRN("Data Bitrate error: %d", ret);
			}
		}
	}
#endif
	// setup tx fifo
	if (mcp2517_setup_tx_fifo(dev) < 0) {
		LOG_ERR("Set TX FIFO failed");
		return -EIO;
	}
	// setup rx fifo
	if (mcp2517_setup_rx_fifo(dev) < 0) {
		LOG_ERR("Set RX FIFO failed");
		return -EIO;
	}

#ifdef CONFIG_CAN_FD_MODE
	if (dev_cfg->bus_speed_data != 0) {
		ret = can_set_timing(dev, &timing, &timing_data);
	} else 
#endif
	{
		ret = can_set_timing(dev, &timing, NULL);
	}
	if (ret < 0) {
		LOG_ERR("Set timing failed");
		return ret;
	}

	ret = can_set_mode(dev, CAN_NORMAL_MODE);
	return ret;
}

#if DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay)

static K_KERNEL_STACK_DEFINE(mcp2517_int_thread_stack,
			     CONFIG_CAN_MCP2517_INT_THREAD_STACK_SIZE);

static struct mcp251xfd_data mcp2517_data_1 = {
	.int_thread_stack = mcp2517_int_thread_stack,
	.filter_usage = 0U,
};

static const struct mcp251xfd_config mcp2517_config_1 = {
	.bus = SPI_DT_SPEC_INST_GET(0, SPI_WORD_SET(8), 0),
	.int_gpio = GPIO_DT_SPEC_INST_GET(0, int_gpios),
	.int_thread_stack_size = CONFIG_CAN_MCP2517_INT_THREAD_STACK_SIZE,
	.int_thread_priority = CONFIG_CAN_MCP2517_INT_THREAD_PRIO,
	.tq_sjw = DT_INST_PROP(0, sjw),
	.tq_prop = DT_INST_PROP_OR(0, prop_seg, 0),
	.tq_bs1 = DT_INST_PROP_OR(0, phase_seg1, 0),
	.tq_bs2 = DT_INST_PROP_OR(0, phase_seg2, 0),
	.bus_speed = DT_INST_PROP(0, bus_speed),
#ifdef CONFIG_CAN_FD_MODE
	.tq_sjw_data = DT_INST_PROP(0, sjw_data),
	.tq_prop_data = DT_INST_PROP_OR(0, prop_seg_data, 0),
	.tq_bs1_data = DT_INST_PROP_OR(0, phase_seg1_data, 0),
	.tq_bs2_data = DT_INST_PROP_OR(0, phase_seg2_data, 0),
	.bus_speed_data = DT_INST_PROP(0, bus_speed_data),
#endif
	.osc_freq = DT_INST_PROP(0, osc_freq),
	.sample_point = DT_INST_PROP_OR(0, sample_point, 0),
	.phy = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(0, phys)),
	.max_bitrate = DT_INST_CAN_TRANSCEIVER_MAX_BITRATE(0, 8000000),
	.pll_enable = DT_INST_PROP_OR(0, pll_enable, false),
};

DEVICE_DT_INST_DEFINE(0, &mcp2517_init, NULL,
		    &mcp2517_data_1, &mcp2517_config_1, POST_KERNEL,
		    CONFIG_CAN_INIT_PRIORITY, &can_api_funcs);

#endif /* DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay) */
